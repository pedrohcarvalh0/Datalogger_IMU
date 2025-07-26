#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/rtc.h"
#include "pico/bootrom.h"
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"
#include "ssd1306.h"
#include "font.h"

// ========== DEFINIÇÕES DE PINOS ==========

// MPU6050 I2C
#define I2C_PORT_MPU i2c0
#define I2C_SDA_MPU 0
#define I2C_SCL_MPU 1
#define MPU6050_ADDR 0x68

// Display OLED I2C
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define ENDERECO_DISP 0x3C

// LEDs e controles
#define BUZZER_PIN 10
#define LED_RGB_R 13
#define LED_RGB_G 11
#define LED_RGB_B 12

// Botões
#define BOTAO_A 5    // Iniciar/Parar captura
#define BOTAO_B 6    // Montar/Desmontar SD

// ========== VARIÁVEIS GLOBAIS ==========
typedef enum {
    ESTADO_INICIALIZANDO,
    ESTADO_PRONTO,
    ESTADO_GRAVANDO,
    ESTADO_ACESSANDO_SD,
    ESTADO_ERRO
} sistema_estado_t;

static sistema_estado_t estado_atual = ESTADO_INICIALIZANDO;
static bool sd_montado = false;
static bool captura_ativa = false;
static uint32_t contador_amostras = 0;
static absolute_time_t tempo_inicio_gravacao;
static char current_data_filename[32]; // Variável global para o nome do arquivo de dados
static uint32_t next_recording_id = 1; // ID para o próximo arquivo de gravação

// Display
ssd1306_t ssd;

// Debounce dos botões
static volatile bool botao_a_pressionado = false;
static volatile bool botao_b_pressionado = false;
static absolute_time_t ultimo_debounce_a;
static absolute_time_t ultimo_debounce_b;
#define DEBOUNCE_DELAY_MS 200

// ========== DECLARAÇÕES DE FUNÇÕES ==========
void set_led_rgb(bool r, bool g, bool b);
void buzzer_init(void);
void play_sound(int frequency, int duration_ms);
void beep_curto(void);
void beep_duplo(void);
static void mpu6050_reset(void);
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]);
static sd_card_t *sd_get_by_name(const char *const name);
static FATFS *sd_get_fs_by_name(const char *name);
static void find_next_recording_id(void);
bool montar_sd(void);
bool desmontar_sd(void);
bool criar_arquivo_csv(void);
bool gravar_dados_imu(uint32_t sample_num, int16_t accel[3], int16_t gyro[3]);
void atualizar_leds(void);
void atualizar_display(void);
void gpio_irq_handler(uint gpio, uint32_t events);
void processar_botoes(void);
void inicializar_sistema(void);

// ========== FUNÇÕES AUXILIARES ==========

// Controla LED RGB
void set_led_rgb(bool r, bool g, bool b) {
    gpio_put(LED_RGB_R, r);
    gpio_put(LED_RGB_G, g);
    gpio_put(LED_RGB_B, b);
}

// Inicializa buzzer
void buzzer_init() {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

// Toca som no buzzer
void play_sound(int frequency, int duration_ms) {
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    if (frequency <= 0) {
        pwm_set_gpio_level(BUZZER_PIN, 0);
        sleep_ms(duration_ms);
        return;
    }
    float divider = 20.0f;
    pwm_set_clkdiv(slice_num, divider);
    uint16_t wrap = (125000000 / (frequency * divider)) - 1;
    pwm_set_wrap(slice_num, wrap);
    pwm_set_gpio_level(BUZZER_PIN, wrap / 2);
    sleep_ms(duration_ms);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

// Beep simples
void beep_curto() {
    play_sound(1000, 100);
}

// Dois beeps
void beep_duplo() {
    play_sound(1000, 100);
    sleep_ms(50);
    play_sound(1000, 100);
}

// ========== FUNÇÕES DO MPU6050 ==========
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT_MPU, MPU6050_ADDR, buf, 2, false);
    sleep_ms(100);
    
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT_MPU, MPU6050_ADDR, buf, 2, false);
    sleep_ms(10);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[6];
    
    // Lê aceleração
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT_MPU, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(I2C_PORT_MPU, MPU6050_ADDR, buffer, 6, false);
    
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    
    // Lê giroscópio
    val = 0x43;
    i2c_write_blocking(I2C_PORT_MPU, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(I2C_PORT_MPU, MPU6050_ADDR, buffer, 6, false);
    
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

// ========== FUNÇÕES DO CARTÃO SD ==========
static sd_card_t *sd_get_by_name(const char *const name) {
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    return NULL;
}

static FATFS *sd_get_fs_by_name(const char *name) {
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    return NULL;
}

// Função para encontrar o próximo ID de gravação
static void find_next_recording_id() {
    FRESULT fr;
    DIR dj;
    FILINFO fno;
    uint32_t max_id = 0;
    char fname_buffer[FF_LFN_BUF]; // Buffer para nomes de arquivos longos

    // Inicia a busca por arquivos no diretório raiz
    fr = f_findfirst(&dj, &fno, "", "imu_data_*.csv");

    while (fr == FR_OK && fno.fname[0]) {
        // Verifica se é um arquivo e se corresponde ao padrão "imu_data_*.csv"
        if (!(fno.fattrib & AM_DIR)) { // Se não for um diretório
            uint32_t current_id;
            // Tenta extrair o número do nome do arquivo
            // Exemplo: "imu_data_123.csv"
            if (sscanf(fno.fname, "imu_data_%lu.csv", &current_id) == 1) {
                if (current_id > max_id) {
                    max_id = current_id;
                }
            }
        }
        fr = f_findnext(&dj, &fno); // Próximo arquivo
    }
    f_closedir(&dj); // Fecha o diretório

    next_recording_id = max_id + 1; // Define o próximo ID como o máximo encontrado + 1
}


// ========== CONTROLE DE ESTADO E DISPLAY ==========
void atualizar_leds() {
    static absolute_time_t ultimo_pisca = {0};
    static bool estado_pisca = false;
    absolute_time_t agora = get_absolute_time();
    
    switch (estado_atual) {
        case ESTADO_INICIALIZANDO:
            set_led_rgb(true, true, false); // Amarelo
            break;
        case ESTADO_PRONTO:
            set_led_rgb(false, true, false); // Verde
            break;
        case ESTADO_GRAVANDO:
            set_led_rgb(true, false, false); // Vermelho
            break;
        case ESTADO_ACESSANDO_SD:
            // Azul piscando
            if (absolute_time_diff_us(ultimo_pisca, agora) >= 200000) {
                estado_pisca = !estado_pisca;
                set_led_rgb(false, false, estado_pisca);
                ultimo_pisca = agora;
            }
            break;
        case ESTADO_ERRO:
            // Roxo piscando
            if (absolute_time_diff_us(ultimo_pisca, agora) >= 300000) {
                estado_pisca = !estado_pisca;
                set_led_rgb(estado_pisca, false, estado_pisca);
                ultimo_pisca = agora;
            }
            break;
    }
}

void atualizar_display() {
    ssd1306_fill(&ssd, false);
    
    // Título
    ssd1306_draw_string(&ssd, "Sistema IMU", 20, 0);
    ssd1306_line(&ssd, 0, 10, 127, 10, true);
    
    // Status
    char status_str[32];
    switch (estado_atual) {
        case ESTADO_INICIALIZANDO:
            strcpy(status_str, "Inicializando...");
            break;
        case ESTADO_PRONTO:
            strcpy(status_str, "Pronto");
            break;
        case ESTADO_GRAVANDO:
            strcpy(status_str, "Gravando...");
            break;
        case ESTADO_ACESSANDO_SD:
            strcpy(status_str, "Acessando SD");
            break;
        case ESTADO_ERRO:
            strcpy(status_str, "ERRO!");
            break;
    }
    ssd1306_draw_string(&ssd, status_str, 5, 15);
    
    // Status SD
    ssd1306_draw_string(&ssd, sd_montado ? "SD: OK" : "SD: --", 5, 25);
    
    // Contador de amostras
    if (captura_ativa) {
        char contador_str[32];
        snprintf(contador_str, sizeof(contador_str), "Amostras: %lu", contador_amostras);
        ssd1306_draw_string(&ssd, contador_str, 5, 35);
        
        // Tempo de gravação
        uint32_t tempo_ms = absolute_time_diff_us(tempo_inicio_gravacao, get_absolute_time()) / 1000;
        char tempo_str[32];
        snprintf(tempo_str, sizeof(tempo_str), "Tempo: %lu.%lus", tempo_ms/1000, (tempo_ms%1000)/100);
        ssd1306_draw_string(&ssd, tempo_str, 5, 45);
    }
    
    // Instruções
    ssd1306_draw_string(&ssd, "A:Start/Stop B:SD", 5, 55);
    
    ssd1306_send_data(&ssd);
}

bool montar_sd() {
    estado_atual = ESTADO_ACESSANDO_SD;
    atualizar_display();
    
    const char *drive = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(drive);
    if (!p_fs) return false;
    
    FRESULT fr = f_mount(p_fs, drive, 1);
    if (FR_OK != fr) {
        estado_atual = ESTADO_ERRO;
        return false;
    }
    
    sd_card_t *pSD = sd_get_by_name(drive);
    if (pSD) {
        pSD->mounted = true;
        sd_montado = true;
        find_next_recording_id(); // Novo: Encontra o próximo ID de gravação ao montar o SD
        return true;
    }
    return false;
}

bool desmontar_sd() {
    estado_atual = ESTADO_ACESSANDO_SD;
    atualizar_display();
    
    const char *drive = sd_get_by_num(0)->pcName;
    FRESULT fr = f_unmount(drive);
    if (FR_OK != fr) {
        estado_atual = ESTADO_ERRO;
        return false;
    }
    
    sd_card_t *pSD = sd_get_by_name(drive);
    if (pSD) {
        pSD->mounted = false;
        pSD->m_Status |= STA_NOINIT;
        sd_montado = false;
        return true;
    }
    return false;
}

// ========== FUNÇÕES DE GRAVAÇÃO ==========
bool criar_arquivo_csv() {
    FIL file;
    
    // Usa o next_recording_id para o nome do arquivo
    snprintf(current_data_filename, sizeof(current_data_filename), "imu_data_%lu.csv", next_recording_id);
    
    FRESULT res = f_open(&file, current_data_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) return false;
    
    // Cabeçalho CSV
    const char *header = "numero_amostra,timestamp,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\n";
    UINT bw;
    res = f_write(&file, header, strlen(header), &bw);
    f_close(&file);
    
    return (res == FR_OK);
}

bool gravar_dados_imu(uint32_t sample_num, int16_t accel[3], int16_t gyro[3]) {
    FIL file;
    // Usa o nome do arquivo armazenado na variável global
    FRESULT res = f_open(&file, current_data_filename, FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) return false;
    
    // Prepara linha CSV
    char buffer[128];
    uint32_t timestamp = to_ms_since_boot(get_absolute_time());
    snprintf(buffer, sizeof(buffer), "%lu,%lu,%d,%d,%d,%d,%d,%d\n",
             sample_num, timestamp, accel[0], accel[1], accel[2], 
             gyro[0], gyro[1], gyro[2]);
    
    UINT bw;
    res = f_write(&file, buffer, strlen(buffer), &bw);
    f_close(&file);
    
    return (res == FR_OK);
}

// ========== INTERRUPÇÕES DOS BOTÕES ==========
void gpio_irq_handler(uint gpio, uint32_t events) {
    absolute_time_t agora = get_absolute_time();
    
    if (gpio == BOTAO_A) {
        if (absolute_time_diff_us(ultimo_debounce_a, agora) > DEBOUNCE_DELAY_MS * 1000) {
            botao_a_pressionado = true;
            ultimo_debounce_a = agora;
        }
    } else if (gpio == BOTAO_B) {
        if (absolute_time_diff_us(ultimo_debounce_b, agora) > DEBOUNCE_DELAY_MS * 1000) {
            botao_b_pressionado = true;
            ultimo_debounce_b = agora;
        }
    }
}

// ========== PROCESSAMENTO DOS BOTÕES ==========
void processar_botoes() {
    if (botao_a_pressionado) {
        botao_a_pressionado = false;
        
        if (estado_atual == ESTADO_PRONTO && sd_montado) {
            // Iniciar captura
            captura_ativa = true;
            contador_amostras = 0; // Reinicia o contador para uma nova gravação
            tempo_inicio_gravacao = get_absolute_time();
            estado_atual = ESTADO_GRAVANDO;
            beep_curto();
            criar_arquivo_csv(); // Cria o arquivo com o novo ID e escreve o cabeçalho
            next_recording_id++; // Incrementa o ID para a PRÓXIMA gravação
        } else if (estado_atual == ESTADO_GRAVANDO) {
            // Parar captura
            captura_ativa = false;
            estado_atual = ESTADO_PRONTO;
            beep_duplo();
        }
    }
    
    if (botao_b_pressionado) {
        botao_b_pressionado = false;
        
        if (!sd_montado) {
            if (montar_sd()) {
                estado_atual = ESTADO_PRONTO;
                beep_curto();
            } else {
                estado_atual = ESTADO_ERRO;
                play_sound(200, 500);
            }
        } else {
            if (desmontar_sd()) {
                estado_atual = ESTADO_INICIALIZANDO;
                beep_duplo();
            } else {
                estado_atual = ESTADO_ERRO;
                play_sound(200, 500);
            }
        }
    }
}

// ========== INICIALIZAÇÃO ==========
void inicializar_sistema() {
    stdio_init_all();
    sleep_ms(2000);
    
    // Inicializa RTC (ainda usado para o timestamp interno do CSV)
    rtc_init();
    datetime_t t = {
        .year = 2024, .month = 1, .day = 1,
        .dotw = 1, .hour = 0, .min = 0, .sec = 0
    };
    rtc_set_datetime(&t);
    
    // Inicializa buzzer
    buzzer_init();
    
    // Inicializa LEDs RGB
    gpio_init(LED_RGB_R);
    gpio_set_dir(LED_RGB_R, GPIO_OUT);
    gpio_init(LED_RGB_G);
    gpio_set_dir(LED_RGB_G, GPIO_OUT);
    gpio_init(LED_RGB_B);
    gpio_set_dir(LED_RGB_B, GPIO_OUT);
    
    // Inicializa botões
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    // Inicializa I2C para display
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);
    
    // Inicializa display
    ssd1306_init(&ssd, 128, 64, false, ENDERECO_DISP, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
    
    // Inicializa I2C para MPU6050
    i2c_init(I2C_PORT_MPU, 400 * 1000);
    gpio_set_function(I2C_SDA_MPU, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_MPU, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_MPU);
    gpio_pull_up(I2C_SCL_MPU);
    
    bi_decl(bi_2pins_with_func(I2C_SDA_MPU, I2C_SCL_MPU, GPIO_FUNC_I2C));
    mpu6050_reset();
    
    // Som de inicialização
    play_sound(1000, 200);
    sleep_ms(100);
    play_sound(1500, 200);
}

// ========== LOOP PRINCIPAL ==========
int main() {
    inicializar_sistema();
    
    int16_t accel[3], gyro[3];
    absolute_time_t ultimo_update_display = get_absolute_time();
    absolute_time_t ultima_leitura = get_absolute_time();
    
    while (true) {
        absolute_time_t agora = get_absolute_time();
        
        // Processa botões
        processar_botoes();
        
        // Lê dados do MPU6050 a cada 50ms
        if (absolute_time_diff_us(ultima_leitura, agora) >= 50000) {
            mpu6050_read_raw(accel, gyro);
            ultima_leitura = agora;
            
            // Se estiver gravando, salva os dados
            if (captura_ativa && sd_montado) {
                contador_amostras++; // Incrementa o contador ANTES de gravar para que seja o número da amostra atual
                if (gravar_dados_imu(contador_amostras, accel, gyro)) {
                    // Sucesso na gravação
                } else {
                    estado_atual = ESTADO_ERRO;
                    captura_ativa = false;
                }
            }
        }
        
        // Atualiza display a cada 200ms
        if (absolute_time_diff_us(ultimo_update_display, agora) >= 200000) {
            atualizar_display();
            ultimo_update_display = agora;
        }
        
        // Atualiza LEDs
        atualizar_leds();
        
        sleep_ms(10);
    }
    
    return 0;
}
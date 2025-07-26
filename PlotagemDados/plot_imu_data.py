import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_imu_data_single_window(file_path):
    """

    Execução do script: Abra um terminal ou prompt de comando, navegue até o diretório onde
    estão salvos os arquivos e execute o script, passando o nome do seu arquivo CSV como argumento:

            ex: python plot_imu_data.py imu_data_1.csv

            ou

            ex: python PlotagemDados/plot_imu_data.py PlotagemDados/imu_data_2.csv
    """
    try:
        # Carrega o arquivo CSV
        df = pd.read_csv(file_path)

        # Converte o timestamp de milissegundos para segundos para o eixo X
        df['timestamp_s'] = df['timestamp'] / 1000.0

        # Cria uma figura e um conjunto de subplots (2 linhas, 1 coluna)
        fig, axes = plt.subplots(2, 1, figsize=(12, 10)) # 2 linhas, 1 coluna, tamanho da figura

        # --- Plotar Aceleração no primeiro subplot ---
        axes[0].plot(df['timestamp_s'], df['accel_x'], label='Accel X')
        axes[0].plot(df['timestamp_s'], df['accel_y'], label='Accel Y')
        axes[0].plot(df['timestamp_s'], df['accel_z'], label='Accel Z')
        axes[0].set_title('Dados de Aceleração do MPU6050 ao Longo do Tempo')
        axes[0].set_xlabel('Tempo (segundos)')
        axes[0].set_ylabel('Aceleração (unidades brutas)')
        axes[0].legend()
        axes[0].grid(True)

        # --- Plotar Giroscópio no segundo subplot ---
        axes[1].plot(df['timestamp_s'], df['giro_x'], label='Giro X')
        axes[1].plot(df['timestamp_s'], df['giro_y'], label='Giro Y')
        axes[1].plot(df['timestamp_s'], df['giro_z'], label='Giro Z')
        axes[1].set_title('Dados de Giroscópio do MPU6050 ao Longo do Tempo')
        axes[1].set_xlabel('Tempo (segundos)')
        axes[1].set_ylabel('Velocidade Angular (unidades brutas)')
        axes[1].legend()
        axes[1].grid(True)

        # Ajusta o layout para evitar sobreposição de títulos/rótulos
        plt.tight_layout()
        plt.show()

    except FileNotFoundError:
        print(f"Erro: O arquivo '{file_path}' não foi encontrado.")
    except pd.errors.EmptyDataError:
        print(f"Erro: O arquivo '{file_path}' está vazio ou não contém dados.")
    except Exception as e:
        print(f"Ocorreu um erro ao processar o arquivo: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python plot_imu_data.py <nome_do_arquivo.csv>")
        print("Exemplo: python plot_imu_data.py imu_data_5.csv")
    else:
        csv_file = sys.argv[1]
        plot_imu_data_single_window(csv_file)
import pandas as pd
import os


def calcular_media_movil(nombre_csv, puntos, output_path, archivo):
    # Cargar CSV
    df = pd.read_csv(nombre_csv)

    # Verificar que las columnas existan
    cols = ['noBond_height 1 (mm)', 'Bond_height 2 (mm)']
    for col in cols:
        if col not in df.columns:
            raise ValueError(f"La columna '{col}' no existe en el archivo.")

    # Reemplazar las columnas con sus medias móviles
    for col in cols:
        df[col] = df[col].rolling(window=puntos).mean()

    # Crear nombre de salida
    base, ext = os.path.splitext(nombre_csv)
    nuevo_nombre = f"{output_path}/{archivo}_Media_movil_{puntos}{ext}"

    # Guardar CSV
    
    df.to_csv(nuevo_nombre, index=False)
    # print(f"Archivo guardado como: {nuevo_nombre}")

carpeta = "C:/eWave/eWave/Datasets/II Semester 2025/Raw_Data"

window = [3, 4 ,5]

for num_media in window:
    for archivo in os.listdir(carpeta):
        path = os.path.join(carpeta,archivo)
        file_name, ext = os.path.splitext(archivo)
        if os.path.isfile(path) and not archivo == "Results.csv" and not archivo == "Past data/":
            output_path = f"C:/eWave/eWave/Datasets/II Semester 2025/15_Hz_Media_{num_media}"
            calcular_media_movil(path, num_media, output_path, file_name)
        

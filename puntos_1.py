import pyrealsense2 as rs  # Importa la librería para interactuar con cámaras Intel RealSense
import numpy as np  # Importa la librería para manejo de arrays y operaciones numéricas
import open3d as o3d  # Importa la librería para visualización y procesamiento de nubes de puntos
import csv  # Importa la librería para manejo de archivos CSV

# Configurar el flujo de datos
pipeline = rs.pipeline()  # Crea un objeto pipeline para manejar el flujo de datos de la cámara
config = rs.config()  # Crea un objeto de configuración para definir los parámetros del stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Habilita el stream de profundidad con resolución 640x480, formato Z16 y 30 FPS

# Iniciar la transmisión
pipeline.start(config)  # Inicia la transmisión de datos con la configuración definida

# Ruta para guardar el archivo CSV
csv_file_path = r"C:\Users\dsplab_1\Desktop\Leche\captura_ss.csv"  # Define la ruta donde se guardará el archivo CSV

# Abrir el archivo CSV para guardar los puntos
with open(csv_file_path, mode='w', newline='') as file:  # Abre el archivo CSV en modo escritura
    writer = csv.writer(file)  # Crea un objeto writer para escribir en el archivo CSV
    writer.writerow(['x', 'y', 'z'])  # Escribe los encabezados de las columnas (x, y, z)

    # Bucle de streaming
    frame_count = 0  # Inicializa un contador de frames
    try:
        while frame_count < 1:  # Bucle que se ejecuta hasta que se capture un frame
            # Obtener el conjunto de frames
            frames = pipeline.wait_for_frames()  # Espera y obtiene un conjunto de frames sincronizados
            depth_frame = frames.get_depth_frame()  # Extrae el frame de profundidad del conjunto de frames
           
            if not depth_frame:  # Si no se obtiene un frame de profundidad, continúa con la siguiente iteración
                continue
           
            # Convertir a numpy array
            depth_image = np.asanyarray(depth_frame.get_data())  # Convierte el frame de profundidad a un array de numpy
           
            # Crear una nube de puntos
            pc = rs.pointcloud()  # Crea un objeto para generar la nube de puntos
            points = pc.calculate(depth_frame)  # Calcula los puntos 3D a partir del frame de profundidad
            vtx = np.asanyarray(points.get_vertices())  # Obtiene los vértices de la nube de puntos como un array de numpy
            xyz = np.array(vtx).view(np.float32).reshape(-1, 3)  # Convierte los vértices a un array de coordenadas (x, y, z)

            # Filtrar los puntos por distancia (por ejemplo, entre 0.3 y 0.63 metros)
            xyz_filtered = xyz[(xyz[:, 2] > 0.3) & (xyz[:, 2] < 0.63)]  # Filtra los puntos en el eje Z (distancia) para mantener solo aquellos dentro del rango especificado

            # Guardar los puntos filtrados en el archivo CSV
            for point in xyz_filtered:  # Itera sobre cada punto filtrado
                writer.writerow(point)  # Escribe las coordenadas (x, y, z) del punto en el archivo CSV

            # Crear un objeto Open3D PointCloud
            pcd = o3d.geometry.PointCloud()  # Crea un objeto de nube de puntos de Open3D
            pcd.points = o3d.utility.Vector3dVector(xyz_filtered)  # Asigna los puntos filtrados al objeto PointCloud

            # Visualizar la nube de puntos
            o3d.visualization.draw_geometries([pcd])  # Muestra la nube de puntos en una ventana interactiva
           
            frame_count += 1  # Incrementa el contador de frames capturados

    finally:
        # Detener la transmisión
        pipeline.stop()  # Detiene la transmisión de datos de la cámara

/*
TEAM PAMBACODE
ESTE CÓDIGO RECOPILA LA INFORMACION PROPORCIONADA POR DOS SENSORES Y UN POTENCIOMETRO, JUNTO CON UN INDICADOR DE LUZ CON EL
FIN DE IMPLEMENTAR MANERAS DE REGISTRAR LOS DATOS EN UNA TABLA Y GRABARLOS EN UN ARCHIVO CSV, ESTE SE GENERA CADA SE EJECUTA EL CODIGO.
2025/03/09 - V.1.2.1
TRABAJARON: CESAR ARTURO / CesarDAlvin | SARA CRYSTEL / Sara130401 | CERON DAUZON / Juryelcd
 */ 
import serial #Para la comunicacion
import time
import pandas as pd #Para la generacion de tablas

# Configuramos el puerto serie
arduino = serial.Serial ("COM5", 9600, timeout=1)


# Generamos un nombre de archivo CSV único usando la fecha y la hora
timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
file_path = f"C:/Users/cesar/Documentos/Escuela y librerias/Materias (trabajos)/Programacion mixta y pruebas/datos_{timestamp}.csv"

# Creamos un DataFrame vacío con columnas ya etiquetadas
columnas = ["Tiempo", "Temperatura (°C)", "Humedad (%)", "Intensidad de Luz", "Voltaje (V)"]
df = pd.DataFrame(columns=columnas)

#Los valores de los sensores se resetean (Medida de seguridad)
temperatura = None
humedad = None
luz = None
voltaje = None

try:
    while True:
        if arduino.in_waiting > 0:
            linea = arduino.readline().decode('utf-8').strip()
            tiempo = time.strftime("%Y-%m-%d %H:%M:%S") #Se da la hora y la fecha por cada captura
            #Se extrae datos segun la etiqueta que posean
            if linea.startswith("TEMP:"):
                temperatura = float(linea.split(":")[1])  # Extrae el valor numérico
                print(f"Temperatura recibida: {temperatura} °C")
                  
            elif linea.startswith("HUM:"):
                  humedad = float(linea.split(":")[1])
                  print(f"Humedad recibida: {humedad} %")
        
            elif linea.startswith("LUM:"):
                  luz = float(linea.split(":")[1])
                  print(f"Intensidad de luz: {luz}")
        
            elif linea.startswith("VOLT:"):
                  voltaje = float(linea.split(":")[1])
                  print(f"Voltaje: {voltaje} V")
            
            #Gestion de errores y validacion
            # Si todas las lecturas están definidas, se agregan al DataFrame
            if None not in [temperatura, humedad, luz, voltaje]:
                nueva_fila = {
                    'Tiempo': tiempo,
                    'Temperatura (°C)': temperatura,
                    'Humedad (%)': humedad,
                    'Intensidad de Luz': luz,
                    'Voltaje (V)': voltaje
                }
                
                # Añadir la nueva fila al DataFrame
                #La funcion concat toma en cuenta los valores a los que asocia una etiqueta, y se anexan al Dataframe
                df = pd.concat([df, pd.DataFrame([nueva_fila])], ignore_index=True) #Se excluyen los indices de la tabla que se genera por default
                
                # Guardar en CSV inmediatamente para evitar pérdida de datos
                df.to_csv(file_path, index=False)
                
                #Verificacion
                # Imprimimos la fila que generamos con el fin de verificar que los datos sean colocados de manera adecuada
                print(nueva_fila)
                
                # Restablecer las variables para la próxima lectura (De esta forma evitamos datos erroneos por falla, que si los hay el proceso se frena al inicio de este ciclo)
                temperatura = None
                humedad = None
                luz = None
                voltaje = None
                
            # Breve pausa para evitar sobrecarga
            time.sleep(0.25)

#Opcion de finalizacion del procesos al presionar Ctrl+C en el que realiza una accion antes de finalizar el proceso
except KeyboardInterrupt:
    print("Interrupción por usuario. Cerrando conexión.")
    arduino.close()

#Procesos finales
finally:
    # Guarda el archivo final y cierra el puerto serie
    df.to_csv(file_path, index=False)
    arduino.close()
    print(f"Datos guardados correctamente en '{file_path}'")

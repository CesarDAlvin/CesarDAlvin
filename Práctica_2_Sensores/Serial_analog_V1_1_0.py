import serial
import time

# Configura el puerto serie (cambia 'COM4' por tu puerto Arduino)
arduino = serial.Serial ('COM5', 9600, timeout=1)

try:
    while True:
        if arduino.in_waiting > 0:
            linea = arduino.readline().decode('utf-8').strip()
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
        
except KeyboardInterrupt:
    print("Interrupción por usuario. Cerrando conexión.")
    arduino.close()

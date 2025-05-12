"""
TEAM PAMBACODE

Este programa importa la informacion de diversos archivos y grafica sus datos (filtros pasa bajas, filtro pasa bandas y FFT)
Fecha: 2025/05/11
Versión: 1.1.0

Autores:
- César Arturo / CesarDAlvin
"""

#Inicio
#Importacion de librerias
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from windrose import WindroseAxes
from scipy.signal import butter, filtfilt
from scipy.fft import fft, fftfreq
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

#Carga de archvos CSV (Cambie la direccion de sus archivos)
dfH = pd.read_csv('C:/Users/cesar/Hum.csv')
dfT = pd.read_csv('C:/Users/cesar/Tempe.csv')
dfV = pd.read_csv('C:/Users/cesar/Viento.csv')

#Carga de variables
Tiempo = dfH['Tiempo']*5
Hum_Rel = dfH['Humedad_Relativa_%']
Temp = dfT['Temperatura_C']
Vel_Vie= dfV['Velocidad_Viento_mps']
Dir_Vie = dfV['Direccion_Viento_deg']


"""
Desea comprobar sus archivos?

print(dfH.head())
print(dfT.head())
print(dfV.head())
"""
#1. Graficacion de datos sin tratamiento
"""
#Para Hum.csv
# Graficar columnas específicas
plt.plot(Tiempo, Hum_Rel,'g', label='Humedad Relativa %')
# Etiquetas y título
plt.xlabel('Tiempo')
plt.ylabel('Humedad Relativa %')
plt.title('Gráfico de Tiempo y Humedad Relativa %')
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Para Tempe.csv
# Graficar columnas específicas
plt.plot(Tiempo, Temp,'g', label='Temperatura °C')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Temperatura °C')
plt.title('Gráfico de Tiempo y Temperatura °C')
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Para Viento.csv
# Graficar columnas específicas
plt.plot(Tiempo, Vel_Vie,'g', label='Velocidad Viento m/s')
plt.plot(Tiempo, Dir_Vie,'b', label='Direccion del viento')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad Viento m/s \nDireccion del viento')
plt.title('Gráfico de Tiempo/Velocidad Viento (m/s) y Direccion del viento ')
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Grafico adicional circular
# Asegurarse de que las columnas existen
direcciones = Dir_Vie
velocidades = Vel_Vie

# Convertir a radianes
direcciones_rad = np.deg2rad(direcciones)

# Escalado de velocidad (ajusta este valor si necesario)
factor_escala = 1  # Amplifica la longitud de las flechas
velocidades_escaladas = velocidades * factor_escala

# Crear gráfico polar
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)

# Dibujar vectores
for theta, r in zip(direcciones_rad, velocidades_escaladas):
    ax.annotate('', xy=(theta, r), xytext=(0, 0),
                arrowprops=dict(arrowstyle='->', color='tab:blue', lw=1.3))

# Mostrar ticks originales para referencia real
max_vel = velocidades.max()
tick_vals = [v for v in np.linspace(0, max_vel, 5)]
tick_labels = [f"{v:.1f}" for v in tick_vals]
ax.set_rticks([v * factor_escala for v in tick_vals])
ax.set_yticklabels(tick_labels)

# Título y ajustes
ax.set_title("Vectores de Viento Escalados\n(Dirección y Velocidad)", va='bottom')
ax.grid(True)

plt.show()

#Grafico vectorizado de la velociad y direccion
import matplotlib.cm as cm
# Asegurarse de que las columnas existen
direcciones = Dir_Vie
velocidades = Vel_Vie
# Conversión
direcciones_rad = np.deg2rad(direcciones)
factor_escala = 10
velocidades_escaladas = velocidades * factor_escala

# Normalizar para mapa de color
norm = plt.Normalize(velocidades.min(), velocidades.max())
colors = cm.viridis(norm(velocidades))  # o 'plasma', 'inferno', etc.

# Gráfico polar con flechas coloreadas
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)

for theta, r, color in zip(direcciones_rad, velocidades_escaladas, colors):
    ax.annotate('', xy=(theta, r), xytext=(0, 0),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.5))

# Escala radial etiquetada correctamente
max_vel = velocidades.max()
tick_vals = [v for v in np.linspace(0, max_vel, 5)]
tick_labels = [f"{v:.1f}" for v in tick_vals]
ax.set_rticks([v * factor_escala for v in tick_vals])
ax.set_yticklabels(tick_labels)

# Colorbar
sm = plt.cm.ScalarMappable(cmap=cm.viridis, norm=norm)
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.1)
cbar.set_label("Velocidad (m/s)")

ax.set_title("Vectores de Viento (Color por Velocidad)", va='bottom')
plt.show()


#Version esferica de los vectores
# Variables
direcciones = Dir_Vie
velocidades = Vel_Vie

# Convertir a radianes
theta = np.deg2rad(direcciones)  # ángulo azimutal

# Escalar velocidad a altura (ángulo phi de elevación sobre la esfera)
# Hacemos que el ángulo phi (elevación) varíe en 0–π/2 según velocidad
norm = plt.Normalize(velocidades.min(), velocidades.max())
phi = norm(velocidades) * (np.pi / 2)  # 0 a 90°

# Coordenadas esféricas a cartesianas
radio_maximo = 10
r = radio_maximo * (velocidades / velocidades.max())
x = r * np.sin(phi) * np.cos(theta)
y = r * np.sin(phi) * np.sin(theta)
z = r * np.cos(phi)

# Gráfico 3D
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Dibujar vectores desde el origen
for xi, yi, zi, vi in zip(x, y, z, velocidades):
    ax.quiver(0, 0, 0, xi, yi, zi, length=1, normalize=False, color=cm.viridis(norm(vi)))

# Ejes y estética
ax.set_xlabel('X (Este)')
ax.set_ylabel('Y (Norte)')
ax.set_zlabel('Altura sobre hemisferio')
ax.set_title('Representación Esférica de Vectores de Viento')
# Límites y aspecto del gráfico
ax.set_xlim(-radio_maximo, radio_maximo)
ax.set_ylim(-radio_maximo, radio_maximo)
ax.set_zlim(0, radio_maximo)
# Escalado uniforme
ax.set_box_aspect([1, 1, 1])
ax.grid(True)

# Barra de colores
sm = plt.cm.ScalarMappable(cmap=cm.viridis, norm=norm)
sm.set_array([])
plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.1, label='Velocidad (m/s)')

plt.tight_layout()
plt.show()
"""
#2. Promediado movil (rango 3)
"""
#Para Hum.csv
#Definir promedio movil
dfH['pro_mov_Hum'] = Hum_Rel.rolling(window=3).mean() #Columna de valores promediados
Pro_MoH = dfH['pro_mov_Hum'].fillna(0)
# Graficar columnas específicas
plt.plot(Tiempo, Hum_Rel,'g', label='Humedad Relativa %')
plt.plot(Tiempo + 5, Pro_MoH,'b', label='Humedad Relativa % (suavizada)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Humedad Relativa %')
plt.title('Tiempo y Humedad Relativa % (Normal y suavizada)')
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Para Tempe.csv
#Definir promedio movil
dfT['pro_mov_Temp'] = Temp.rolling(window=3).mean() #Columna de valores promediados
Pro_MoT = dfT['pro_mov_Temp'].fillna(0)
# Graficar columnas específicas
plt.plot(Tiempo, Temp,'g', label='Temperatura °C')
plt.plot(Tiempo + 5, Pro_MoT,'b', label='Temperatura °C (suavizada)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Temperatura °C')
plt.title('Tiempo y Temperatura °C (Normal y suavizada)')
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Para Viento.csv
#Definir promedio movil
dfV['pro_mov_Vel_Vie'] = Vel_Vie.rolling(window=3).mean() #Columna de valores promediados
Pro_MoVV = dfV['pro_mov_Vel_Vie'].fillna(0)
dfV['pro_mov_Dir_Vie'] = Dir_Vie.rolling(window=3).mean() #Columna de valores promediados
Pro_MoVD = dfV['pro_mov_Dir_Vie'].fillna(0)
# Graficar columnas específicas
#Tiempo/Velocidad
plt.plot(Tiempo, Vel_Vie,'b', label='Velocidad Viento m/s')
plt.plot(Tiempo, Pro_MoVV,'orange', label='Velocidad Viento m/s (suavizado)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad Viento m/s')
plt.title('Tiempo /Velocidad Viento (m/s) (normal y suavizado)')
plt.legend(loc="best")
# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Tiempo/Direccion
plt.plot(Tiempo, Dir_Vie,'b', label='Direccion del viento')
plt.plot(Tiempo, Pro_MoVD,'orange', label='Direccion del viento (suavizado)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Direccion del viento')
plt.title('Tiempo /Direccion del viento (normal y suavizado)')
plt.legend(loc="best")
# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()
"""

#3. Filtro Pasa Bajas
"""
#Para Hum.csv
# Parámetros del filtro
orden = 4
frecuencia_corte = 0.1  # Frecuencia normalizada (0 a 1), siendo 1 = Nyquist

# Crear coeficientes del filtro
b, a = butter(N=orden, Wn=frecuencia_corte, btype='low', analog=False)
#Aplicar filtro
se_filH = filtfilt(b, a, Hum_Rel)
# Graficar columnas específicas
plt.plot(Tiempo, Hum_Rel,'g', label='Humedad Relativa %')
plt.plot(Tiempo + 5, se_filH,'b', label='Humedad Relativa % (Filtro Pasa Bajas Butterworth)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Humedad Relativa %')
plt.title('Tiempo y Humedad Relativa % (Filtro Pasa Bajas Butterworth)')
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Para Tempe.csv
# Parámetros del filtro
orden = 4
frecuencia_corte = 0.1  # Frecuencia normalizada (0 a 1), siendo 1 = Nyquist

# Crear coeficientes del filtro
b, a = butter(N=orden, Wn=frecuencia_corte, btype='low', analog=False)
#Aplicar filtro
se_filT = filtfilt(b, a, Temp)
# Graficar columnas específicas
plt.plot(Tiempo, Temp,'g', label='Temperatura °C')
plt.plot(Tiempo + 5, se_filT,'b', label='Temperatura °C (Filtro Pasa Bajas Butterworth)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Temperatura °C')
plt.title('Tiempo y Temperatura °C (Filtro Pasa Bajas Butterworth)')
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()


#Para Viento.csv
# Parámetros del filtro
orden = 4
frecuencia_corte = 0.1  # Frecuencia normalizada (0 a 1), siendo 1 = Nyquist

# Crear coeficientes del filtro
b, a = butter(N=orden, Wn=frecuencia_corte, btype='low', analog=False)
#Aplicar filtro
se_filVV = filtfilt(b, a, Vel_Vie)
se_filDV = filtfilt(b, a, Dir_Vie)
# Graficar columnas específicas
#Tiempo/Velocidad
plt.plot(Tiempo, Vel_Vie,'b', label='Velocidad Viento m/s')
plt.plot(Tiempo, se_filVV,'orange', label='Velocidad Viento m/s (Filtro Pasa Bajas Butterworth)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad Viento m/s')
plt.title('Tiempo /Velocidad Viento (m/s) (normal y Filtro Pasa Bajas Butterworth)')
plt.legend(loc="best")
# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Tiempo/Direccion
plt.plot(Tiempo, Dir_Vie,'b', label='Direccion del viento')
plt.plot(Tiempo, se_filDV,'orange', label='Direccion del viento (Filtro Pasa Bajas Butterworth)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Direccion del viento')
plt.title('Tiempo /Direccion del viento (normal y Filtro Pasa Bajas Butterworth)')
plt.legend(loc="best")
# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()
"""

#4. Filtro pasa bandas

"""
#Para Hum.csv
# Parámetros del filtro
fs = 9000  # Frecuencia de muestreo en Hz
lowcut = 25.0   # Límite inferior de paso
highcut = 90.0  # Límite superior de paso
orden = 4
# Convertir a frecuencia normalizada (0–1)
nyquist = fs / 2
low = lowcut / nyquist
high = highcut / nyquist

# Crear filtro pasa bandas
b, a = butter(N=orden, Wn=[low, high], btype='band')
#Aplicar filtro
se_filHb = filtfilt(b, a, Hum_Rel)
# Graficar columnas específicas
plt.plot(Tiempo, Hum_Rel,'g', label='Humedad Relativa %')
plt.plot(Tiempo, se_filHb,'b', label='Humedad Relativa % (Filtro Pasa Bandas Butterworth)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Humedad Relativa %')
plt.title('Tiempo y Humedad Relativa % (Filtro Pasa Bandas Butterworth)')
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Para Tempe.csv
# Parámetros del filtro
fs = 9000  # Frecuencia de muestreo en Hz
lowcut = 25.0   # Límite inferior de paso
highcut = 90.0  # Límite superior de paso
orden = 4
# Convertir a frecuencia normalizada (0–1)
nyquist = fs / 2
low = lowcut / nyquist
high = highcut / nyquist

# Crear filtro pasa bandas
b, a = butter(N=orden, Wn=[low, high], btype='band')
#Aplicar filtro
se_filTb = filtfilt(b, a, Temp)
# Graficar columnas específicas
plt.plot(Tiempo, Temp,'g', label='Temperatura °C')
plt.plot(Tiempo, se_filTb,'b', label='Temperatura °C(Filtro Pasa Bandas Butterworth)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Temperatura °C')
plt.title('Tiempo y Temperatura °C(Filtro Pasa Bandas Butterworth)')
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Para Viento.csv
# Parámetros del filtro
fs = 2100 # Frecuencia de muestreo en Hz
lowcut = 1.0 #Límite inferior de paso
highcut = 90.0  # Límite superior de paso
orden = 4
# Convertir a frecuencia normalizada (0–1)
nyquist = fs / 2
low = lowcut / nyquist
high = highcut / nyquist

# Crear coeficientes del filtro
b, a = butter(N=orden, Wn=[low, high], btype='band')
#Aplicar filtro
se_filVVb = filtfilt(b, a, Vel_Vie)
se_filDVb = filtfilt(b, a, Dir_Vie)
# Graficar columnas específicas
#Tiempo/Velocidad
plt.plot(Tiempo, Vel_Vie,'b', label='Velocidad Viento m/s')
plt.plot(Tiempo, se_filVVb,'orange', label='Velocidad Viento m/s (Filtro Pasa Bandas Butterworth)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad Viento m/s')
plt.title('Tiempo /Velocidad Viento (m/s) (normal y Filtro Pasa Bandas Butterworth)')
plt.legend(loc="best")
# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Tiempo/Direccion
plt.plot(Tiempo, Dir_Vie,'b', label='Direccion del viento')
plt.plot(Tiempo, se_filDVb,'orange', label='Direccion del viento (Filtro Pasa Bandas Butterworth)')
# Etiquetas y título
plt.xlabel('Tiempo (s)')
plt.ylabel('Direccion del viento')
plt.title('Tiempo /Direccion del viento (normal y Filtro Pasa Bandas Butterworth)')
plt.legend(loc="best")
# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()
"""
#5. Análisis de Fourier (FFT)
"""
#Para Hum.csv
# Parámetros del filtro
fs = 120  # Frecuencia de muestreo en Hz
# Aplicar FFT
N = len(Hum_Rel)
frecuencias = np.fft.rfftfreq(N, d=1/fs)
espectro = np.abs(np.fft.rfft(Hum_Rel)) / N

# Graficar espectro
plt.plot(frecuencias, espectro)
# Etiquetas y título
plt.title("Espectro de Frecuencia (FFT) Humedad")
plt.xlabel("Frecuencia (Hz)")
plt.ylabel("Magnitud")
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Para Tempe.csv
# Parámetros del filtro
fs = 120  # Frecuencia de muestreo en Hz
# Aplicar FFT
N = len(Temp)
frecuencias = np.fft.rfftfreq(N, d=1/fs)
espectro = np.abs(np.fft.rfft(Temp)) / N

# Graficar espectro
plt.plot(frecuencias, espectro)
# Etiquetas y título
plt.title("Espectro de Frecuencia (FFT) Temperatura")
plt.xlabel("Frecuencia (Hz)")
plt.ylabel("Magnitud")
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

"""
#Para Viento.csv
# Parámetros del filtro
fs = 120  # Frecuencia de muestreo en Hz
# Aplicar FFT
N = len(Vel_Vie)
frecuencias = np.fft.rfftfreq(N, d=1/fs)
espectro = np.abs(np.fft.rfft(Vel_Vie)) / N

# Graficar espectro
plt.plot(frecuencias, espectro)
# Etiquetas y título
plt.title("Espectro de Frecuencia (FFT) Velocidad del viento")
plt.xlabel("Frecuencia (Hz)")
plt.ylabel("Magnitud")
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

#Para Dir_Vie
# Aplicar FFT
N = len(Dir_Vie)
frecuencias = np.fft.rfftfreq(N, d=1/fs)
espectro = np.abs(np.fft.rfft(Dir_Vie)) / N

# Graficar espectro
plt.plot(frecuencias, espectro)
# Etiquetas y título
plt.title("Espectro de Frecuencia (FFT) Direccion del viento")
plt.xlabel("Frecuencia (Hz)")
plt.ylabel("Magnitud")
plt.legend(loc="best")

# Mostrar la gráfica
plt.grid(True)
plt.tight_layout()
plt.show()

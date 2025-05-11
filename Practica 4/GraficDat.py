"""
TEAM PAMBACODE

Este programa importa la informacion de diversos archivos y grafica sus datos
Fecha: 2025/05/10
Versión: 1.0.0

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

#Carga de archvos CSV
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
"""
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
"""
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
"""

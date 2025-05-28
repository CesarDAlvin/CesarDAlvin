import numpy as np
import matplotlib.pyplot as plt

# Parámetros
T_max = 0.1 * 0.09807  # Torque máximo [Nm]
r = 0.02  # Radio de la rueda [m]
RPM_max = 3000  # RPM motor sin carga
gear_ratio = 10  # Relación de transmisión motor:rueda
dt = 1.0  # Paso de tiempo [s]

# Generar 100 datos aleatorios de PWM
np.random.seed(0)
PWM = np.random.randint(0, 1024, size=100)

# Inicializar arrays
time = np.arange(0, 100*dt, dt)
T = (PWM / 1023) * T_max  # Torque [Nm]
RPM_wheel = (PWM / 1023) * RPM_max / gear_ratio  # RPM de rueda
omega = RPM_wheel * 2 * np.pi / 60  # Velocidad angular [rad/s]
v = omega * r  # Velocidad lineal [m/s]
a = np.zeros_like(v)
s = np.zeros_like(v)

# Calcular aceleración y distancia recorrida
for i in range(1, len(v)):
    a[i] = (v[i] - v[i-1]) / dt
    s[i] = s[i-1] + v[i] * dt

# Graficar torque vs tiempo
plt.figure()
plt.plot(time, T)
plt.title('Torque vs Tiempo')
plt.xlabel('Tiempo [s]')
plt.ylabel('Torque [Nm]')
plt.grid(True)
plt.show()

# Graficar velocidad vs tiempo
plt.figure()
plt.plot(time, v)
plt.title('Velocidad Lineal vs Tiempo')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad [m/s]')
plt.grid(True)
plt.show()

# Graficar aceleración vs tiempo
plt.figure()
plt.plot(time, a)
plt.title('Aceleración vs Tiempo')
plt.xlabel('Tiempo [s]')
plt.ylabel('Aceleración [m/s²]')
plt.grid(True)
plt.show()

# Graficar distancia recorrida vs tiempo
plt.figure()
plt.plot(time, s)
plt.title('Distancia Recorrida vs Tiempo')
plt.xlabel('Tiempo [s]')
plt.ylabel('Distancia [m]')
plt.grid(True)
plt.show()

import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parámetros de la pista
R = 1.0   # radio de los semicírculos
L = 4.0   # longitud de los tramos rectos
P = 2 * np.pi * R + 2 * L  # perímetro total
seg_length = 0.6  # longitud de cada segmento recto

def track_position(s):
    s_mod = s % P
    if s_mod < L:
        x = -L/2 + s_mod;    y = R
    elif s_mod < L + np.pi * R:
        theta = np.pi/2 + (s_mod - L) / R
        x = L/2 + R * np.cos(theta);    y = R * np.sin(theta)
    elif s_mod < 2 * L + np.pi * R:
        x = L/2 - (s_mod - (L + np.pi * R));    y = -R
    else:
        theta = 3 * np.pi/2 + (s_mod - (2 * L + np.pi * R)) / R
        x = -L/2 + R * np.cos(theta);    y = R * np.sin(theta)
    return x, y

def track_tangent(s, eps=1e-3):
    x1, y1 = track_position(s)
    x2, y2 = track_position(s + eps)
    vx, vy = x2 - x1, y2 - y1
    norm = math.hypot(vx, vy)
    return (vx / norm, vy / norm)

# Preparar datos y guardar CSV
s_values = np.arange(0, P, seg_length)
angles = []
for s in s_values:
    tx, ty = track_tangent(s)
    px, py = -ty, tx
    angle_deg = math.degrees(math.atan2(py, px))
    angles.append(angle_deg)

angle_changes = []
for i in range(len(angles)):
    next_i = (i + 1) % len(angles)
    delta = angles[next_i] - angles[i]
    delta = (delta + 180) % 360 - 180
    angle_changes.append(delta)

df = pd.DataFrame({
    'step_index': np.arange(len(s_values)),
    's_position': s_values,
    'angle_deg': angles,
    'angle_change_deg': angle_changes
})

csv_path = 'C:/Users/cesar/Escritorio/ Registropista.csv'
df.to_csv(csv_path, index=False)
print(f"Archivo CSV guardado en: {csv_path}")

# Configurar animación
t_vals = np.linspace(0, P, 500)
track_coords = np.array([track_position(t) for t in t_vals])

fig, ax = plt.subplots()
ax.plot(track_coords[:, 0], track_coords[:, 1], lw=2, color='orange')

segment, = ax.plot([], [], lw=3, color='blue')
ax.set_aspect('equal')
ax.set_xlim(-L/2 - R - 0.5, L/2 + R + 0.5)
ax.set_ylim(-R - 0.5, R + 0.5)
ax.axis('off')
ax.set_title('Simulación: Segmento siguiendo la pista')

def update(frame):
    x, y = track_position(frame)
    tx, ty = track_tangent(frame)
    px, py = -ty, tx
    dx, dy = (px * seg_length/2, py * seg_length/2)
    x_vals = [x - dx, x + dx]
    y_vals = [y - dy, y + dy]
    segment.set_data(x_vals, y_vals)
    return segment,

ani = FuncAnimation(fig, update, frames=np.linspace(0, P, 400), interval=50, blit=False)
plt.show()

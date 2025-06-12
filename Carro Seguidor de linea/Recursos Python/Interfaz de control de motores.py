#!/usr/bin/env python3
"""
TEAM PAMBACODE

Módulo: Interfas de control de motores.py
Descripción:
  Interfaz gráfica para controlar un motor DC mediante el driver TB6612FNG.
  Funciona en conjunto con el sketch Arduino correspondiente que acepta
  comandos serie y devuelve datos CSV. Permite:

    - Selección de puerto serial y conexión.
    - Envío de comandos PWM manuales.
    - Ejecución de rampas de velocidad entre dos valores durante un tiempo.
    - Activación de un bucle de display local que calcula RPM, torque,
      velocidad lineal, aceleración y distancia.
    - Visualización en tiempo real de datos calculados y gráficas embebidas
      en una ventana de Tkinter.
    - Recepción y muestra de la última línea cruda enviada por Arduino.

Uso:
    Monitoreo

Fecha:     2025/06/01
Versión:   4.0.0

Autores:
  - César Arturo       / CesarDAlvin
  - Sara Crystel       / Sara130401
  - Ceron Dauzon       / Juryelcd

Requisitos:
  - El sketch Arduino debe estar cargado y responder con "READY" al iniciar.
  - Python 3 con las librerías: tkinter, pyserial, matplotlib.
"""

import tkinter as tk                        # Librería base para interfaces gráficas
from tkinter import ttk, messagebox        # Widgets avanzados y diálogos
import serial, threading, time, math       # Serial, hilos, temporización, matemáticas
import serial.tools.list_ports             # Para listar puertos serie
import matplotlib.pyplot as plt            # Para generar gráficas
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # Canvas de Matplotlib en Tkinter

# -------------------------------------------------------------
# Constantes del modelo físico y límites de señal
# -------------------------------------------------------------
PWM_MAX = 1023              # Valor máximo de ciclo PWM que acepta Arduino
PWM_MIN = 200               # Valor mínimo para arrancar el motor
TORQUE_MAX = 0.0081         # Torque máximo teórico del motor en Nm
MOTOR_MAX_RPM = 2500        # RPM sin carga a PWM_MAX
GEAR_RATIO = 10.0           # Relación de reducción 10:1
MOTOR_RADIUS = 0.02         # Radio de la rueda en metros


class MotorGUI(tk.Tk):
    """
    Clase principal de la interfaz gráfica:
      - Configura ventana y widgets.
      - Gestiona conexión y comunicación serial con Arduino.
      - Envía comandos de control y rampas.
      - Procesa datos recibidos y calcula parámetros dinámicos.
      - Muestra valores numéricos y gráficas en tiempo real.
    """
    def __init__(self):
        """
        Inicialización de la ventana:
          - Título, tamaño y variables de estado.
          - Invoca _build_interface() para crear todos los widgets.
        """
        super().__init__()
        self.title("Control TB6612FNG")
        self.geometry("1000x800")

        # --- Estado de conexión y bucle de lectura ---
        self.serial = None         # Instancia serial.Serial
        self.running = False       # Flag para el hilo lector

        # --- Parámetros de display local ---
        self.display_on = False    # Flag para activar/desactivar display local
        self.display_job = None    # ID de la tarea 'after' programada
        self.sample_interval = 50  # Intervalo de muestreo en ms

        # --- Buffers y acumuladores para datos ---
        self.data = []             # Lista de tuplas con (t, PWM, RPM, torque, vel, accel, dist)
        self.last_time = None      # Timestamp de la última muestra
        self.last_vel = 0.0        # Velocidad anterior
        self.total_dist = 0.0      # Distancia total recorrida

        # --- Parámetros de rampa progresiva ---
        self.ramping = False
        self.ramp_start = 0.0
        self.ramp_dur = 0.0
        self.ramp_p0 = 0
        self.ramp_p1 = 0

        # Construcción de todos los widgets
        self._build_interface()


    def toggle_display(self):
        """
        Activa o detiene el loop de display local:
          - Reinicia datos y temporizadores en activación.
          - Cancela la tarea 'after' en detención.
        """
        if not self.display_on:
            self.display_on = True
            self.display_btn.config(text="Detener Display")
            self.data.clear()
            self.last_time = time.time()
            self.last_vel = 0.0
            self.total_dist = 0.0
            self._update_local_display()
        else:
            self.display_on = False
            self.display_btn.config(text="Display Real")
            if self.display_job:
                self.after_cancel(self.display_job)
                self.display_job = None


    def _build_interface(self):
        """
        Construye la interfaz gráfica:
          1. Selector de puerto y botones de conexión.
          2. Panel de operaciones (manual, rampa, display, stop).
          3. Panel de datos calculados.
          4. Panel de última línea cruda.
          5. Área de gráficas en 3 subplots.
        """
        # -----------------------
        # Selector y conexión
        # -----------------------
        top = tk.Frame(self)
        ttk.Label(top, text="Puerto:").pack(side='left')
        self.cmb = ttk.Combobox(top, state='readonly', width=10)
        self.refresh_ports()
        self.cmb.pack(side='left', padx=5)
        ttk.Button(top, text="Refrescar", command=self.refresh_ports).pack(side='left')
        ttk.Button(top, text="Conectar", command=self.connect).pack(side='left', padx=5)
        top.pack(pady=10, fill='x')

        # -----------------------
        # Panel de operaciones
        # -----------------------
        ctrl = tk.LabelFrame(self, text="Operaciones", padx=10, pady=10)
        # PWM manual
        ttk.Label(ctrl, text="PWM manual:").grid(row=0, column=0, sticky='e')
        self.pwm_var = tk.IntVar(value=PWM_MIN)
        ttk.Scale(ctrl, from_=PWM_MIN, to=PWM_MAX, variable=self.pwm_var,
                  orient='horizontal').grid(row=0, column=1, sticky='we', padx=5)
        ttk.Entry(ctrl, textvariable=self.pwm_var, width=6).grid(row=0, column=2, padx=5)
        ttk.Button(ctrl, text="Aplicar PWM", command=self.manual).grid(row=0, column=3, padx=10)
        # Rampa de PWM
        ttk.Label(ctrl, text="RPM inicio:").grid(row=1, column=0, sticky='e')
        self.r0_var = tk.IntVar(value=0)
        ttk.Entry(ctrl, textvariable=self.r0_var, width=6).grid(row=1, column=1)
        ttk.Label(ctrl, text="RPM fin:").grid(row=1, column=2, sticky='e')
        self.r1_var = tk.IntVar(value=0)
        ttk.Entry(ctrl, textvariable=self.r1_var, width=6).grid(row=1, column=3)
        ttk.Label(ctrl, text="Duración(s):").grid(row=1, column=4, sticky='e')
        self.secs_var = tk.IntVar(value=5)
        ttk.Entry(ctrl, textvariable=self.secs_var, width=6).grid(row=1, column=5, padx=(0,10))
        ttk.Button(ctrl, text="Iniciar Rampa", command=self.ramp).grid(row=1, column=6)
        # Display y Stop
        self.display_btn = ttk.Button(ctrl, text="Display Real", command=self.toggle_display)
        self.display_btn.grid(row=2, column=1, pady=10)
        ttk.Button(ctrl, text="Stop Motor", command=self.stop).grid(row=2, column=2, pady=10)
        for c in (1, 3, 5):
            ctrl.columnconfigure(c, weight=1)
        ctrl.pack(fill='x', padx=10, pady=5)

        # -----------------------
        # Panel de datos calculados
        # -----------------------
        panel = tk.LabelFrame(self, text="Datos Calculados", padx=10, pady=10)
        labels = ["PWM", "RPM", "Torque", "Vel", "Accel", "Dist"]
        self.vars = {lbl: tk.StringVar(value="0.00") for lbl in labels}
        for i, lbl in enumerate(labels):
            ttk.Label(panel, text=lbl + ":").grid(row=0, column=2*i, sticky='e')
            ttk.Label(panel, textvariable=self.vars[lbl]).grid(row=0, column=2*i+1, sticky='w')
        panel.pack(fill='x', padx=10, pady=5)

        # -----------------------
        # Panel de línea cruda
        # -----------------------
        raw_frame = tk.LabelFrame(self, text="Última línea cruda", padx=10, pady=5)
        self.raw_var = tk.StringVar(value="")
        ttk.Label(raw_frame, textvariable=self.raw_var, foreground="gray40").pack(fill='x')
        raw_frame.pack(fill='x', padx=10, pady=5)

        # -----------------------
        # Área de gráficas
        # -----------------------
        self.fig, self.axs = plt.subplots(3, 1, figsize=(6, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=10, pady=10)


    # ---------------------------------------------------------
    # Modelo matemático y conversiones de unidades
    # ---------------------------------------------------------
    def torque_from_pwm(self, p):
        """Calcula el torque (Nm) basado en el ciclo de trabajo PWM."""
        return (p / PWM_MAX) * TORQUE_MAX

    def motor_rpm(self, p):
        """Calcula las RPM del motor sin carga a partir de PWM."""
        return (p / PWM_MAX) * MOTOR_MAX_RPM

    def wheel_rpm(self, rpm):
        """Convierte RPM del motor a RPM de la rueda tras la reducción."""
        return rpm / GEAR_RATIO

    def rad_per_sec(self, wrpm):
        """Convierte RPM de rueda a radianes por segundo."""
        return wrpm * 2 * math.pi / 60

    def linear_vel(self, omega):
        """Calcula velocidad lineal (m/s) de la rueda."""
        return omega * MOTOR_RADIUS


    # ---------------------------------------------------------
    # Gestión de puerto serial y comunicación con Arduino
    # ---------------------------------------------------------
    def refresh_ports(self):
        """Actualiza la lista de puertos serie disponibles."""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb['values'] = ports
        if ports:
            self.cmb.current(0)

    def connect(self):
        """
        Conecta al puerto seleccionado y lanza el hilo de lectura que
        captura la última línea cruda enviada por Arduino.
        """
        port = self.cmb.get()
        if not port:
            return messagebox.showwarning("Puerto", "Selecciona un puerto.")
        try:
            self.serial = serial.Serial(port, 9600, timeout=1)
            # Esperar el mensaje de lista "READY"
            while self.serial.readline().decode().strip() != "READY":
                pass
            self.running = True
            # Hilo demonio para leer sin bloquear la GUI
            threading.Thread(target=self.idle_reader, daemon=True).start()
            messagebox.showinfo("Serial", f"Conectado a {port}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def idle_reader(self):
        """
        Hilo en background que lee continuamente del serial y
        actualiza la etiqueta con la última línea recibida.
        """
        while self.running:
            line = self.serial.readline().decode(errors='ignore').strip()
            if line:
                self.after(0, self.raw_var.set, line)


    # ---------------------------------------------------------
    # Envío de comandos al Arduino
    # ---------------------------------------------------------
    def send_cmd(self, cmd):
        """Envía un comando (con '\n') al Arduino si está conectado."""
        if self.serial and self.serial.is_open:
            self.serial.write((cmd + "\n").encode())

    def manual(self):
        """Envía un comando de PWM manual y cancela cualquier rampa."""
        self.ramping = False
        pwm = self.pwm_var.get()
        self.send_cmd(f"M {pwm}")

    def ramp(self):
        """Configura parámetros de rampa y la inicia."""
        r0, r1 = self.r0_var.get(), self.r1_var.get()
        # Convertir RPM a PWM dentro de [PWM_MIN, PWM_MAX]
        self.ramp_p0 = int(min(max(r0 / MOTOR_MAX_RPM * PWM_MAX, PWM_MIN), PWM_MAX))
        self.ramp_p1 = int(min(max(r1 / MOTOR_MAX_RPM * PWM_MAX, PWM_MIN), PWM_MAX))
        self.ramp_dur = self.secs_var.get()
        self.ramp_start = time.time()
        self.ramping = True
        # Asegurar que el display esté activo para ver la rampa
        if not self.display_on:
            self.toggle_display()
        else:
            # Reiniciar datos acumulados
            self.data.clear()
            self.last_time = time.time()
            self.last_vel = 0.0
            self.total_dist = 0.0

    def stop(self):
        """Detiene el motor, cancela rampa y apaga el display local."""
        self.ramping = False
        self.send_cmd("S")
        self.running = False
        if self.display_on:
            self.toggle_display()


    # ---------------------------------------------------------
    # Bucle de rampa cíclica y cálculo de PWM actual
    # ---------------------------------------------------------
    def _current_pwm(self, now):
        """
        Si hay rampa activa, calcula el PWM interpolado en función
        del tiempo transcurrido y lo envía. Si no, devuelve el PWM manual.
        """
        if self.ramping:
            elapsed = now - self.ramp_start
            # Ajustar ciclos completos de rampa
            if elapsed >= self.ramp_dur:
                cycles = int(elapsed // self.ramp_dur)
                self.ramp_start += cycles * self.ramp_dur
                elapsed -= cycles * self.ramp_dur
            pwm = int(self.ramp_p0 + (self.ramp_p1 - self.ramp_p0) * (elapsed / self.ramp_dur))
            self.send_cmd(f"M {pwm}")
            return pwm
        return self.pwm_var.get()


    # ---------------------------------------------------------
    # Display local: cálculos y actualizaciones periódicas
    # ---------------------------------------------------------
    def _update_local_display(self):
        """
        Bucle que:
          - Mide dt y calcula parámetros (RPM, torque, vel, accel, dist).
          - Actualiza etiquetas y buffer de datos.
          - Llama a update_plots() y reprograma la siguiente iteración.
        """
        if not self.display_on:
            return
        now = time.time()
        dt = now - (self.last_time or now)
        self.last_time = now

        pwm = self._current_pwm(now)
        mrpm = self.motor_rpm(pwm)
        torque = self.torque_from_pwm(pwm)
        wrpm = self.wheel_rpm(mrpm)
        omega = self.rad_per_sec(wrpm)
        vel = self.linear_vel(omega)
        accel = (vel - self.last_vel) / dt if dt > 0 else 0.0
        self.total_dist += vel * dt
        self.last_vel = vel

        # Almacenar y mostrar datos
        self.data.append((now, pwm, mrpm, torque, vel, accel, self.total_dist))
        for lbl, val in zip(["PWM","RPM","Torque","Vel","Accel","Dist"],
                            [pwm, mrpm, torque, vel, accel, self.total_dist]):
            self.vars[lbl].set(f"{val:.2f}")

        self.update_plots()
        self.display_job = self.after(self.sample_interval, self._update_local_display)


    def update_plots(self):
        """
        Redibuja las tres gráficas:
          1) RPM del motor.
          2) Velocidad lineal.
          3) Aceleración.
        Basado en los datos acumulados en self.data.
        """
        if not self.data:
            return
        base_time = self.data[0][0]
        xs = [pt[0] - base_time for pt in self.data]

        # Gráfica de RPM
        self.axs[0].cla()
        self.axs[0].plot(xs, [pt[2] for pt in self.data])
        self.axs[0].set_ylabel("Motor RPM")

        # Gráfica de velocidad
        self.axs[1].cla()
        self.axs[1].plot(xs, [pt[4] for pt in self.data])
        self.axs[1].set_ylabel("Vel (m/s)")

        # Gráfica de aceleración
        self.axs[2].cla()
        self.axs[2].plot(xs, [pt[5] for pt in self.data])
        self.axs[2].set_ylabel("Accel (m/s²)")

        self.fig.tight_layout()
        self.canvas.draw()


# -------------------------------------------------------------
# Punto de entrada del script
# -------------------------------------------------------------
if __name__ == "__main__":
    MotorGUI().mainloop()

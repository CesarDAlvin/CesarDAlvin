import tkinter as tk
from tkinter import ttk
import serial, threading, time, csv
import matplotlib.pyplot as plt

# Configuración Puerto Serial
PORT = 'COM3'
BAUD  = 9600

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Monitor Motor TB6612FNG')
        self.geometry('800x600')
        self.serial = serial.Serial(PORT, BAUD, timeout=1)
        self.running = True
        self.data = []  # almacena [timestamp, PWM, rpm, torque, vel]

        # Widgets
        self.tree = ttk.Treeview(self, columns=('PWM','RPM','Torque','Vel'), show='headings')
        for c in self.tree['columns']:
            self.tree.heading(c, text=c)
        self.tree.pack(fill='x')

        # Botones
        btn_frame = tk.Frame(self)
        tk.Button(btn_frame, text='Iniciar Grabación CSV', command=self.start_csv).pack(side='left')
        tk.Button(btn_frame, text='Detener', command=self.stop).pack(side='left')
        btn_frame.pack()

        # Lienzo Matplotlib
        self.fig, self.axs = plt.subplots(3,1, figsize=(5,6))
        self.canvas = None

        # Hilo de lectura
        threading.Thread(target=self.read_loop, daemon=True).start()
        self.after(1000, self.update_plot)

    def read_loop(self):
        while self.running:
            line = self.serial.readline().decode().strip()
            if line:
                parts = line.split()
                pwm = int(parts[0]); rpm = float(parts[1]); torque = float(parts[2]); vel = float(parts[3])
                ts = time.time()
                self.data.append((ts,pwm,rpm,torque,vel))
                self.tree.insert('',0, values=(pwm,rpm,torque,vel))

    def update_plot(self):
        if self.data:
            times = [d[0]-self.data[0][0] for d in self.data]
            rpms  = [d[2] for d in self.data]
            vels  = [d[4] for d in self.data]
            accs  = [(rpms[i]-rpms[i-1])/(times[i]-times[i-1]) if i>0 else 0 for i in range(len(rpms))]
            self.axs[0].cla(); self.axs[0].plot(times, rpms)
            self.axs[1].cla(); self.axs[1].plot(times, vels)
            self.axs[2].cla(); self.axs[2].plot(times, accs)
            plt.tight_layout()
            if not self.canvas:
                from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
                self.canvas = FigureCanvasTkAgg(self.fig, master=self)
                self.canvas.get_tk_widget().pack(fill='both', expand=True)
            self.canvas.draw()
        self.after(1000, self.update_plot)

    def start_csv(self):
        with open('motor_data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time','pwm','rpm','torque','vel'])
            for row in self.data:
                writer.writerow(row)
        print('CSV generado.')

    def stop(self):
        self.running = False
        self.serial.close()

if __name__ == '__main__':
    App().mainloop()
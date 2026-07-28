import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import os

class SerialLogger:

    def __init__(self, root):
        self.root = root
        self.root.title("UWB Serial Logger")

        self.ser = None
        self.running = False
        self.log_file = None

        # COM Port
        ttk.Label(root, text="COM Port:").grid(row=0, column=0, padx=5, pady=5)

        self.port_var = tk.StringVar(value="/dev/cu.usbmodem1403")
        self.port_combo = ttk.Combobox(
            root,
            textvariable=self.port_var,
            width=15
        )
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        # Baud Rate
        ttk.Label(root, text="Baud:").grid(row=0, column=2, padx=5, pady=5)

        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(root, textvariable=self.baud_var, width=10).grid(
            row=0, column=3, padx=5, pady=5
        )

        ttk.Button(
            root,
            text="Refresh Ports",
            command=self.refresh_ports
        ).grid(row=0, column=4, padx=5)

        # Buttons
        self.start_btn = ttk.Button(
            root,
            text="Start Logging",
            command=self.start_logging
        )
        self.start_btn.grid(row=1, column=0, padx=5, pady=5)

        self.stop_btn = ttk.Button(
            root,
            text="Stop Logging",
            command=self.stop_logging,
            state="disabled"
        )
        self.stop_btn.grid(row=1, column=1, padx=5, pady=5)

        # Text area
        self.text = scrolledtext.ScrolledText(
            root,
            width=120,
            height=30
        )
        self.text.grid(
            row=2,
            column=0,
            columnspan=5,
            padx=10,
            pady=10
        )

        self.refresh_ports()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports

    def get_next_filename(self):
        index = 1

        while True:
            filename = f"log_{index:03d}.txt"

            if not os.path.exists(filename):
                return filename

            index += 1

    def start_logging(self):

        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())

            self.ser = serial.Serial(
                port,
                baud,
                timeout=1
            )

            filename = self.get_next_filename()
            self.log_file = open(filename, "w", encoding="utf-8")

            self.text.insert(
                tk.END,
                f"\n===== Logging to {filename} =====\n"
            )

            self.running = True

            threading.Thread(
                target=self.read_serial,
                daemon=True
            ).start()

            self.start_btn.config(state="disabled")
            self.stop_btn.config(state="normal")

        except Exception as e:
            messagebox.showerror("Error", str(e))

    def stop_logging(self):

        self.running = False

        if self.ser:
            self.ser.close()

        if self.log_file:
            self.log_file.close()

        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")

    def read_serial(self):

        while self.running:

            try:
                line = self.ser.readline().decode(
                    "utf-8",
                    errors="replace"
                )

                if line:

                    self.text.insert(tk.END, line)
                    self.text.see(tk.END)

                    self.log_file.write(line)
                    self.log_file.flush()

            except Exception as e:
                self.text.insert(
                    tk.END,
                    f"\nERROR: {e}\n"
                )
                break


root = tk.Tk()
app = SerialLogger(root)
root.mainloop()
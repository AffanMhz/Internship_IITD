import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import os
from datetime import datetime
import queue

class SerialLogger:
    def __init__(self, root):
        self.root = root
        self.root.title("UWB Serial Logger")
        self.root.geometry("1000x700")
        self.root.minsize(800, 500)

        # Try to use native macOS styling if available
        style = ttk.Style()
        if "aqua" in style.theme_names():
            style.theme_use("aqua")

        # Variables
        self.ser = None
        self.running = False
        self.log_file = None
        self.data_queue = queue.Queue()
        
        # UI Font defaults for Mac
        self.ui_font = ("Helvetica Neue", 13)
        self.mono_font = ("Menlo", 12)

        # --- Top Header Frame ---
        header_frame = ttk.Frame(root, padding="10 10 10 5")
        header_frame.pack(fill=tk.X)

        ttk.Label(header_frame, text="Port:", font=self.ui_font).pack(side=tk.LEFT, padx=(0, 5))
        
        self.port_var = tk.StringVar(value="COM12")
        self.port_combo = ttk.Combobox(header_frame, textvariable=self.port_var, width=25, font=self.ui_font)
        self.port_combo.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(header_frame, text="Baud Rate:", font=self.ui_font).pack(side=tk.LEFT, padx=(0, 5))
        
        # Upgraded to a Combobox for standard baud rates
        self.baud_var = tk.StringVar(value="115200")
        baud_rates = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]
        self.baud_combo = ttk.Combobox(header_frame, textvariable=self.baud_var, values=baud_rates, width=10, font=self.ui_font)
        self.baud_combo.pack(side=tk.LEFT, padx=(0, 15))

        refresh_btn = ttk.Button(header_frame, text="↻ Refresh Ports", command=self.refresh_ports)
        refresh_btn.pack(side=tk.LEFT)

        # --- Toolbar Frame ---
        toolbar_frame = ttk.Frame(root, padding="10 5 10 10")
        toolbar_frame.pack(fill=tk.X)

        self.start_btn = ttk.Button(toolbar_frame, text="▶ Start Logging", command=self.start_logging)
        self.start_btn.pack(side=tk.LEFT, padx=(0, 10))

        self.stop_btn = ttk.Button(toolbar_frame, text="■ Stop", command=self.stop_logging, state="disabled")
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 10))

        # NEW FEATURE: Clear Button
        clear_btn = ttk.Button(toolbar_frame, text="⎚ Clear Console", command=self.clear_console)
        clear_btn.pack(side=tk.LEFT, padx=(0, 20))

        # NEW FEATURE: Toggle options
        self.timestamp_var = tk.BooleanVar(value=False)
        timestamp_cb = ttk.Checkbutton(toolbar_frame, text="Add Timestamps", variable=self.timestamp_var)
        timestamp_cb.pack(side=tk.LEFT, padx=(0, 15))

        self.autoscroll_var = tk.BooleanVar(value=True)
        autoscroll_cb = ttk.Checkbutton(toolbar_frame, text="Auto-Scroll", variable=self.autoscroll_var)
        autoscroll_cb.pack(side=tk.LEFT)

        # --- Text Area Frame ---
        text_frame = ttk.Frame(root, padding="10 0 10 10")
        text_frame.pack(fill=tk.BOTH, expand=True)

        self.text = scrolledtext.ScrolledText(
            text_frame, 
            wrap=tk.WORD, 
            font=self.mono_font,
            bg="#1E1E1E", # Dark mode background
            fg="#D4D4D4", # Light text for contrast
            insertbackground="white", # Cursor color
            highlightthickness=0,
            padx=10,
            pady=10
        )
        self.text.pack(fill=tk.BOTH, expand=True)

        # --- Status Bar ---
        status_frame = ttk.Frame(root)
        status_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        self.status_var = tk.StringVar(value="Status: Disconnected | No file active")
        status_label = ttk.Label(status_frame, textvariable=self.status_var, font=("Helvetica Neue", 11), padding=5, foreground="#555555")
        status_label.pack(side=tk.LEFT)

        self.refresh_ports()
        
        # Start the GUI update loop
        self.root.after(100, self.process_queue)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def get_next_filename(self):
        index = 1
        while True:
            filename = f"log_{index:03d}.txt"
            if not os.path.exists(filename):
                return filename
            index += 1

    def clear_console(self):
        """Clears the text area without stopping the stream."""
        self.text.delete(1.0, tk.END)

    def start_logging(self):
        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())

            self.ser = serial.Serial(port, baud, timeout=1)
            
            filename = self.get_next_filename()
            self.log_file = open(filename, "w", encoding="utf-8")

            # Update UI
            self.clear_console()
            self.start_btn.config(state="disabled")
            self.stop_btn.config(state="normal")
            self.status_var.set(f"Status: Connected to {port} | Logging to: {os.path.abspath(filename)}")

            # Visual header in text
            self.data_queue.put(f"===== Connected at {baud} baud =====\n")
            self.data_queue.put(f"===== Saving logs to {filename} =====\n\n")

            self.running = True
            
            # Start background reading thread
            threading.Thread(target=self.read_serial, daemon=True).start()

        except Exception as e:
            messagebox.showerror("Connection Error", f"Could not connect to port.\n\nError: {str(e)}")

    def stop_logging(self):
        self.running = False

        if self.ser and self.ser.is_open:
            self.ser.close()

        if self.log_file:
            self.log_file.close()

        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_var.set("Status: Disconnected | Ready")
        
        self.data_queue.put(f"\n===== Disconnected =====\n")

    def read_serial(self):
        """Runs in a background thread to read hardware serial."""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="replace")
                    if line:
                        # Add timestamp if requested
                        if self.timestamp_var.get():
                            timestamp = datetime.now().strftime("[%H:%M:%S.%f]")[:-3] + "] "
                            line = timestamp + line

                        # Write to file immediately
                        if self.log_file and not self.log_file.closed:
                            self.log_file.write(line)
                            self.log_file.flush()

                        # Send to GUI safely
                        self.data_queue.put(line)
            except Exception as e:
                if self.running:
                    self.data_queue.put(f"\n[ERROR]: {e}\n")
                    self.root.after(0, self.stop_logging)
                break

    def process_queue(self):
        """Runs on the main GUI thread to safely update the text area."""
        while not self.data_queue.empty():
            try:
                line = self.data_queue.get_nowait()
                self.text.insert(tk.END, line)
                
                # Auto-scroll if enabled
                if self.autoscroll_var.get():
                    self.text.see(tk.END)
                    
            except queue.Empty:
                break
                
        # Schedule the next queue check
        self.root.after(50, self.process_queue)

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialLogger(root)
    root.mainloop()
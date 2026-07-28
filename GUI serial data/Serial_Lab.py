import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import os
import queue
import re
from datetime import datetime
from collections import deque
from tkinter import filedialog
import statistics

# Matplotlib imports for plotting
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

class SerialLab:
    def __init__(self, root):
        self.root = root
        self.root.title("SerialLab Console")
        self.root.geometry("900x700")
        self.root.minsize(600, 400)

        style = ttk.Style()
        if "aqua" in style.theme_names():
            style.theme_use("aqua")

        # Variables for Serial
        self.ser = None
        self.running = False
        self.data_queue = queue.Queue()
        self.full_log_buffer = [] # Stores all data for saving later
        
        # Variables for Plotting
        self.max_points = 2000 # Increased to allow better analysis
        self.x_counter = 0
        self.plot_data = {}         
        self.plot_lines = {}        
        self.toggles = {}           
        
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
                       '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
        self.color_idx = 0
        self.needs_plot_update = False
        self.plotter_auto_opened = False

        self.ui_font = ("Helvetica Neue", 12)
        self.mono_font = ("Menlo", 11)

        self._build_main_ui()
        self._build_plotter_window()
        
        self.root.after(50, self.process_queue)
        self.root.after(50, self.update_plot) 

    def _build_main_ui(self):
        header_frame = ttk.Frame(self.root, padding="10 10 10 5")
        header_frame.pack(fill=tk.X)

        ttk.Label(header_frame, text="Port:", font=self.ui_font).pack(side=tk.LEFT, padx=(0, 5))
        self.port_var = tk.StringVar(value="COM13")
        self.port_combo = ttk.Combobox(header_frame, textvariable=self.port_var, width=25, font=self.ui_font)
        self.port_combo.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(header_frame, text="Baud:", font=self.ui_font).pack(side=tk.LEFT, padx=(0, 5))
        self.baud_var = tk.StringVar(value="115200")
        baud_rates = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]
        self.baud_combo = ttk.Combobox(header_frame, textvariable=self.baud_var, values=baud_rates, width=10, font=self.ui_font)
        self.baud_combo.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Button(header_frame, text="↻ Refresh", command=self.refresh_ports).pack(side=tk.LEFT)

        toolbar_frame = ttk.Frame(self.root, padding="10 5 10 10")
        toolbar_frame.pack(fill=tk.X)

        self.start_btn = ttk.Button(toolbar_frame, text="▶ Start", command=self.start_logging)
        self.start_btn.pack(side=tk.LEFT, padx=(0, 10))

        self.stop_btn = ttk.Button(toolbar_frame, text="■ Stop", command=self.stop_logging, state="disabled")
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.save_btn = ttk.Button(toolbar_frame, text="💾 Save Log As...", command=self.save_data)
        self.save_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.analyze_btn = ttk.Button(toolbar_frame, text="📊 Analyze Data", command=self.analyze_data)
        self.analyze_btn.pack(side=tk.LEFT, padx=(0, 20))

        ttk.Button(toolbar_frame, text="⎚ Clear", command=self.clear_all_data).pack(side=tk.LEFT, padx=(0, 20))

        self.timestamp_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(toolbar_frame, text="Timestamps", variable=self.timestamp_var).pack(side=tk.LEFT, padx=(0, 15))
        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar_frame, text="Auto-Scroll", variable=self.autoscroll_var).pack(side=tk.LEFT, padx=(0, 20))

        ttk.Button(toolbar_frame, text="📈 Open Plotter", command=self.show_plotter_window).pack(side=tk.RIGHT)

        console_frame = ttk.Frame(self.root, padding="10 0 10 10")
        console_frame.pack(fill=tk.BOTH, expand=True)

        self.text = scrolledtext.ScrolledText(
            console_frame, wrap=tk.WORD, font=self.mono_font,
            bg="#1E1E1E", fg="#D4D4D4", insertbackground="white", 
            highlightthickness=0, padx=10, pady=10
        )
        self.text.pack(fill=tk.BOTH, expand=True)

        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, side=tk.BOTTOM)
        self.status_var = tk.StringVar(value="Status: Disconnected | Ready")
        ttk.Label(status_frame, textvariable=self.status_var, font=self.ui_font, foreground="#555555").pack(side=tk.LEFT, padx=10, pady=5)

        self.refresh_ports()

    def _build_plotter_window(self):
        self.plot_window = tk.Toplevel(self.root)
        self.plot_window.title("SerialLab - Universal Plotter")
        self.plot_window.geometry("1000x700")
        self.plot_window.minsize(800, 600)
        self.plot_window.protocol("WM_DELETE_WINDOW", self.plot_window.withdraw)
        self.plot_window.bind("<Map>", lambda e: self._force_plot_update())

        main_split = ttk.PanedWindow(self.plot_window, orient=tk.HORIZONTAL)
        main_split.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        toggles_container = ttk.Frame(main_split, width=200)
        main_split.add(toggles_container, weight=0)

        self.toggles_frame = ttk.LabelFrame(toggles_container, text="Active Variables", padding=10)
        self.toggles_frame.pack(fill=tk.X, side=tk.TOP)

        plot_frame = ttk.Frame(main_split)
        main_split.add(plot_frame, weight=1)
        
        self.main_fig = Figure(figsize=(8, 5), dpi=100)
        self.main_ax = self.main_fig.add_subplot(111)
        self.main_ax.set_title("Live Data Stream", fontsize=12)
        self.main_ax.grid(True, linestyle='--', alpha=0.6)
        
        self.main_canvas = FigureCanvasTkAgg(self.main_fig, master=plot_frame)
        self.main_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.main_toolbar = NavigationToolbar2Tk(self.main_canvas, plot_frame)
        self.main_toolbar.update()

        self.plot_window.withdraw()

    def show_plotter_window(self):
        self.plot_window.deiconify()
        self.plot_window.lift()
        self.needs_plot_update = True

    def _force_plot_update(self):
        self.needs_plot_update = True

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def clear_all_data(self):
        self.text.delete(1.0, tk.END)
        self.x_counter = 0
        self.full_log_buffer.clear()
        self.plot_data.clear()
        self.plot_lines.clear()
        self.toggles.clear()

        for widget in self.toggles_frame.winfo_children(): widget.destroy()

        self.main_ax.cla()
        self.main_ax.set_title("Live Data Stream", fontsize=12)
        self.main_ax.grid(True, linestyle='--', alpha=0.6)
        self.main_canvas.draw_idle()
        self.needs_plot_update = True

    def start_logging(self):
        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            self.ser = serial.Serial(port, baud, timeout=1)
            
            self.start_btn.config(state="disabled")
            self.stop_btn.config(state="normal")
            self.status_var.set(f"Status: Connected to {port}")

            msg = f"===== Connected at {baud} baud =====\n"
            self.data_queue.put(('text_only', msg))
            self.full_log_buffer.append(msg)

            self.running = True
            threading.Thread(target=self.read_serial, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Connection Error", f"Could not connect to port.\n\n{str(e)}")

    def stop_logging(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_var.set("Status: Disconnected | Ready")
        self.data_queue.put(('text_only', "\n===== Disconnected =====\n"))
        self.full_log_buffer.append("\n===== Disconnected =====\n")

    def save_data(self):
        if not self.full_log_buffer:
            messagebox.showinfo("Empty Log", "There is no data to save yet.")
            return
            
        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")],
            title="Save Log As..."
        )
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.writelines(self.full_log_buffer)
                messagebox.showinfo("Saved", f"Log successfully saved to:\n{filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Could not save file: {e}")

    def analyze_data(self):
        if not self.plot_data:
            messagebox.showinfo("No Data", "No variable data available to analyze.")
            return

        analyze_win = tk.Toplevel(self.root)
        analyze_win.title("Data Analysis")
        analyze_win.geometry("350x300")
        analyze_win.grab_set() # Focus lock
        
        ttk.Label(analyze_win, text="Select Variable to Analyze:", font=self.ui_font).pack(pady=(15,5))
        
        var_combo = ttk.Combobox(analyze_win, values=list(self.plot_data.keys()), state="readonly", font=self.ui_font)
        var_combo.pack(pady=5)
        var_combo.current(0)
            
        result_str = tk.StringVar(value="\nSelect a variable and click Analyze.\n")
        
        def run_analysis():
            key = var_combo.get()
            if not key or key not in self.plot_data:
                return
            data = list(self.plot_data[key]['y'])
            if not data:
                result_str.set("No data points collected for this variable.")
                return
            
            n = len(data)
            minimum = min(data)
            maximum = max(data)
            mean = statistics.mean(data)
            stdev = statistics.stdev(data) if n > 1 else 0.0
            
            res = (f"Variable: {key}\n"
                   f"Data Points: {n}\n\n"
                   f"Min: {minimum:.3f}\n"
                   f"Max: {maximum:.3f}\n"
                   f"Mean: {mean:.3f}\n"
                   f"Std Dev: {stdev:.3f}")
            result_str.set(res)

        ttk.Button(analyze_win, text="Calculate Statistics", command=run_analysis).pack(pady=10)
        ttk.Label(analyze_win, textvariable=result_str, justify=tk.LEFT, font=("Menlo", 11)).pack(pady=10)

    def parse_line_for_plot(self, line):
        extracted_data = {}
        clean_line = line.strip()
        matches = re.findall(r'([a-zA-Z0-9_]+)\s*[=:]\s*([-+]?\d*\.?\d+)', clean_line)
        for key, val_str in matches:
            try:
                extracted_data[key] = float(val_str)
            except ValueError:
                pass
        return extracted_data

    def register_new_variable(self, key):
        if key not in self.plot_data:
            self.plot_data[key] = {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)}
            color = self.colors[self.color_idx % len(self.colors)]
            self.color_idx += 1
            
            line, = self.main_ax.plot([], [], label=key, color=color, linewidth=1.5)
            self.plot_lines[key] = line
            
            self.toggles[key] = tk.BooleanVar(value=True)
            cb = ttk.Checkbutton(
                self.toggles_frame, text=key, variable=self.toggles[key], command=self._on_toggle_changed
            )
            cb.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=2)
            
            self.main_ax.legend(loc='upper left')

            if not self.plotter_auto_opened:
                self.show_plotter_window()
                self.plotter_auto_opened = True

    def _on_toggle_changed(self):
        self.needs_plot_update = True

    def read_serial(self):
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line_bytes = self.ser.readline()
                    line_str = line_bytes.decode("utf-8", errors="replace")
                    
                    if line_str:
                        # Save raw data to background buffer for File Saving
                        self.full_log_buffer.append(line_str)
                        
                        parsed_vars = self.parse_line_for_plot(line_str)
                        
                        display_str = line_str
                        if self.timestamp_var.get():
                            timestamp = datetime.now().strftime("[%H:%M:%S.%f]")[:-3] + "] "
                            display_str = timestamp + display_str

                        self.data_queue.put(('data', display_str, parsed_vars))
            except Exception as e:
                if self.running:
                    self.data_queue.put(('text_only', f"\n[ERROR]: {e}\n"))
                    self.root.after(0, self.stop_logging)
                break

    def process_queue(self):
        text_chunk = []
        updates_made = False

        for _ in range(1000): 
            if self.data_queue.empty():
                break
                
            item = self.data_queue.get_nowait()
            msg_type = item[0]
            
            if msg_type == 'text_only':
                text_chunk.append(item[1])
            elif msg_type == 'data':
                text_chunk.append(item[1])
                parsed_vars = item[2]
                
                if parsed_vars:
                    self.x_counter += 1
                    for key, val in parsed_vars.items():
                        if key not in self.plot_data:
                            self.register_new_variable(key)
                        
                        self.plot_data[key]['x'].append(self.x_counter)
                        self.plot_data[key]['y'].append(val)
                        
                    updates_made = True

        if text_chunk:
            self.text.insert(tk.END, "".join(text_chunk))
            line_count = int(self.text.index('end-1c').split('.')[0])
            if line_count > 2000:
                self.text.delete('1.0', f"{line_count - 2000}.0")
            if self.autoscroll_var.get():
                self.text.see(tk.END)

        if updates_made:
            self.needs_plot_update = True
            
        self.root.after(50, self.process_queue)

    def update_plot(self):
        if self.needs_plot_update:
            self.needs_plot_update = False
            
            if self.plot_window.winfo_ismapped():
                has_visible = False
                
                for key, line in self.plot_lines.items():
                    if self.toggles[key].get():
                        line.set_data(list(self.plot_data[key]['x']), list(self.plot_data[key]['y']))
                        line.set_visible(True)
                        if len(self.plot_data[key]['x']) > 0:
                            has_visible = True
                    else:
                        line.set_visible(False)
                        
                # Relim(visible_only=True) perfectly shrinks/expands the scale
                # when large scales are toggled off!
                self.main_ax.relim(visible_only=True)
                if has_visible:
                    self.main_ax.autoscale_view(True, True, True)
                self.main_canvas.draw_idle()

        self.root.after(50, self.update_plot)


if __name__ == "__main__":
    root = tk.Tk()
    app = SerialLab(root)
    root.mainloop()
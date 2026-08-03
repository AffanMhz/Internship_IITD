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

# Matplotlib imports for plotting
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

class SerialLogger:
    def __init__(self, root):
        self.root = root
        self.root.title("UWB Serial Logger & Real-Time Plotter")
        self.root.geometry("1300x800")
        self.root.minsize(1000, 600)

        # Try to use native macOS styling if available
        style = ttk.Style()
        if "aqua" in style.theme_names():
            style.theme_use("aqua")

        # Variables for Serial
        self.ser = None
        self.running = False
        self.log_file = None
        self.data_queue = queue.Queue()
        
        # Variables for Plotting
        self.max_points = 500 # Adjust this to keep more/less history in the plot
        self.x_counter = 0
        self.plot_data = {}         # Format: {'key': {'x': deque, 'y': deque}}
        self.plot_lines = {}        # Format: {'key': Line2D_object}
        self.plot_destinations = {} # Format: {'key': 'main' or 'timer'}
        self.toggles = {}           # Format: {'key': tk.BooleanVar}
        
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
                       '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
        self.color_idx = 0
        self.needs_plot_update = False
        self.timer_auto_opened = False # Track if we've auto-opened the second window yet

        # UI Font defaults
        self.ui_font = ("Helvetica Neue", 12)
        self.mono_font = ("Menlo", 11)

        self._build_ui()
        self._build_timer_window()
        
        # Start the Background GUI update loops
        self.root.after(50, self.process_queue)
        self.root.after(50, self.update_plot) # 20 FPS update rate for smooth, lag-free plotting

    def _build_ui(self):
        """Builds the main layout combining the controls, console, and primary plot."""
        # --- Top Header Frame (Connection Controls) ---
        header_frame = ttk.Frame(self.root, padding="10 10 10 5")
        header_frame.pack(fill=tk.X)

        ttk.Label(header_frame, text="Port:", font=self.ui_font).pack(side=tk.LEFT, padx=(0, 5))
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(header_frame, textvariable=self.port_var, width=25, font=self.ui_font)
        self.port_combo.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(header_frame, text="Baud:", font=self.ui_font).pack(side=tk.LEFT, padx=(0, 5))
        
        self.baud_var = tk.StringVar(value="115200")
        baud_rates = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]
        self.baud_combo = ttk.Combobox(header_frame, textvariable=self.baud_var, values=baud_rates, width=10, font=self.ui_font)
        self.baud_combo.pack(side=tk.LEFT, padx=(0, 15))

        refresh_btn = ttk.Button(header_frame, text="↻ Refresh", command=self.refresh_ports)
        refresh_btn.pack(side=tk.LEFT)

        # --- Toolbar Frame (Actions) ---
        toolbar_frame = ttk.Frame(self.root, padding="10 5 10 10")
        toolbar_frame.pack(fill=tk.X)

        self.start_btn = ttk.Button(toolbar_frame, text="▶ Start", command=self.start_logging)
        self.start_btn.pack(side=tk.LEFT, padx=(0, 10))

        self.stop_btn = ttk.Button(toolbar_frame, text="■ Stop", command=self.stop_logging, state="disabled")
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 10))

        clear_btn = ttk.Button(toolbar_frame, text="⎚ Clear Data", command=self.clear_all_data)
        clear_btn.pack(side=tk.LEFT, padx=(0, 20))

        self.timestamp_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(toolbar_frame, text="Add Timestamps", variable=self.timestamp_var).pack(side=tk.LEFT, padx=(0, 15))

        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar_frame, text="Auto-Scroll", variable=self.autoscroll_var).pack(side=tk.LEFT, padx=(0, 20))

        # NEW: Button to show the separated timer plot
        show_timer_btn = ttk.Button(toolbar_frame, text="📈 Show Timers Window", command=self.show_timer_window)
        show_timer_btn.pack(side=tk.LEFT)

        # --- Main Split Window (Console on left, Plot on right) ---
        paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # 1. Left Pane: Text Console
        console_frame = ttk.Frame(paned_window)
        paned_window.add(console_frame, weight=1)

        self.text = scrolledtext.ScrolledText(
            console_frame, 
            wrap=tk.WORD, 
            font=self.mono_font,
            bg="#1E1E1E", fg="#D4D4D4", insertbackground="white", 
            highlightthickness=0, padx=10, pady=10
        )
        self.text.pack(fill=tk.BOTH, expand=True)

        # 2. Right Pane: Main Plotter & Toggles
        plot_container = ttk.Frame(paned_window)
        paned_window.add(plot_container, weight=2)

        # Matplotlib Figure setup (MAIN)
        self.main_fig = Figure(figsize=(5, 4), dpi=100)
        self.main_ax = self.main_fig.add_subplot(111)
        self.main_ax.set_title("Distance & Core Telemetry", fontsize=10)
        self.main_ax.grid(True, linestyle='--', alpha=0.6)
        
        self.main_canvas = FigureCanvasTkAgg(self.main_fig, master=plot_container)
        self.main_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.main_toolbar = NavigationToolbar2Tk(self.main_canvas, plot_container)
        self.main_toolbar.update()

        # Frames for dynamic Checkbuttons
        toggles_container = ttk.Frame(plot_container)
        toggles_container.pack(fill=tk.X, side=tk.BOTTOM, pady=(5,0))

        self.main_toggles_frame = ttk.LabelFrame(toggles_container, text="Main Variables (Distance)", padding=5)
        self.main_toggles_frame.pack(fill=tk.X, side=tk.TOP, pady=(0, 5))

        self.timer_toggles_frame = ttk.LabelFrame(toggles_container, text="Timer Variables (Pop-up Window)", padding=5)
        self.timer_toggles_frame.pack(fill=tk.X, side=tk.TOP)

        # --- Status Bar ---
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        self.status_var = tk.StringVar(value="Status: Disconnected | No file active")
        ttk.Label(status_frame, textvariable=self.status_var, font=self.ui_font, foreground="#555555").pack(side=tk.LEFT, padx=10, pady=5)

        self.refresh_ports()

    def _build_timer_window(self):
        """Builds the secondary window specifically for large timer numbers to avoid scale squishing."""
        self.timer_window = tk.Toplevel(self.root)
        self.timer_window.title("Timer Plotter (Large Values)")
        self.timer_window.geometry("800x500")
        
        # When user closes this window, we just hide it instead of destroying it
        self.timer_window.protocol("WM_DELETE_WINDOW", self.timer_window.withdraw)

        self.timer_fig = Figure(figsize=(6, 4), dpi=100)
        self.timer_ax = self.timer_fig.add_subplot(111)
        self.timer_ax.set_title("Timers & Large Variables", fontsize=10)
        self.timer_ax.grid(True, linestyle='--', alpha=0.6)
        
        self.timer_canvas = FigureCanvasTkAgg(self.timer_fig, master=self.timer_window)
        self.timer_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.timer_toolbar = NavigationToolbar2Tk(self.timer_canvas, self.timer_window)
        self.timer_toolbar.update()

        # Hide it immediately upon creation
        self.timer_window.withdraw()

    def show_timer_window(self):
        """Brings the timer window to the front."""
        self.timer_window.deiconify()
        self.timer_window.lift()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def get_next_filename(self):
        index = 1
        while True:
            filename = f"log_{index:03d}.txt"
            if not os.path.exists(filename):
                return filename
            index += 1

    def clear_all_data(self):
        """Clears text console, plot data, and destroys old toggles."""
        self.text.delete(1.0, tk.END)
        self.x_counter = 0
        
        # Clear Data structures
        self.plot_data.clear()
        self.plot_lines.clear()
        self.plot_destinations.clear()
        self.toggles.clear()

        # Destroy old UI Toggles
        for widget in self.main_toggles_frame.winfo_children(): widget.destroy()
        for widget in self.timer_toggles_frame.winfo_children(): widget.destroy()

        # Reset Axes
        self.main_ax.cla()
        self.main_ax.set_title("Distance & Core Telemetry", fontsize=10)
        self.main_ax.grid(True, linestyle='--', alpha=0.6)
        
        self.timer_ax.cla()
        self.timer_ax.set_title("Timers & Large Variables", fontsize=10)
        self.timer_ax.grid(True, linestyle='--', alpha=0.6)

        self.main_canvas.draw_idle()
        self.timer_canvas.draw_idle()
        self.needs_plot_update = True

    def start_logging(self):
        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())

            self.ser = serial.Serial(port, baud, timeout=1)
            
            filename = self.get_next_filename()
            self.log_file = open(filename, "w", encoding="utf-8")

            # Update UI
            self.start_btn.config(state="disabled")
            self.stop_btn.config(state="normal")
            self.status_var.set(f"Status: Connected to {port} | Logging to: {os.path.abspath(filename)}")

            msg = f"===== Connected at {baud} baud | Saving to {filename} =====\n"
            self.data_queue.put(('text_only', msg))

            self.running = True
            threading.Thread(target=self.read_serial, daemon=True).start()

        except Exception as e:
            messagebox.showerror("Connection Error", f"Could not connect to port.\n\n{str(e)}")

    def stop_logging(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.log_file:
            self.log_file.close()

        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_var.set("Status: Disconnected | Ready")
        self.data_queue.put(('text_only', "\n===== Disconnected =====\n"))

    def parse_line_for_plot(self, line):
        """Parses the incoming serial string for numeric data to plot."""
        extracted_data = {}
        clean_line = line.strip()

        # 1. Key-Value Pairs: Find patterns like `Distance: 4.53` or `round1=1234`
        # Matches alphanumeric keys followed by '=' or ':' and a number
        matches = re.findall(r'([a-zA-Z0-9_]+)\s*[=:]\s*([-+]?\d*\.?\d+)', clean_line)
        for key, val_str in matches:
            try:
                extracted_data[key] = float(val_str)
            except ValueError:
                pass

        return extracted_data

    def register_new_variable(self, key):
        """Creates data structures, routes to correct plot, and creates UI toggles."""
        if key not in self.plot_data:
            
            # Determine destination (Timers go to secondary window, Distance to main)
            # You can add more keywords to 'timer_keywords' if needed
            timer_keywords = ['round', 'reply', 'time', 't1', 't2', 't3', 't4']
            is_timer = any(kw in key.lower() for kw in timer_keywords)
            
            dest = 'timer' if is_timer else 'main'
            self.plot_destinations[key] = dest
            
            target_ax = self.timer_ax if is_timer else self.main_ax
            target_frame = self.timer_toggles_frame if is_timer else self.main_toggles_frame

            # Create data containers
            self.plot_data[key] = {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)}
            
            # Pick a color
            color = self.colors[self.color_idx % len(self.colors)]
            self.color_idx += 1
            
            # Create a line on the axes
            line, = target_ax.plot([], [], label=key, color=color, linewidth=1.5)
            self.plot_lines[key] = line
            
            # Create a UI Checkbutton
            self.toggles[key] = tk.BooleanVar(value=True)
            cb = ttk.Checkbutton(
                target_frame, 
                text=key, 
                variable=self.toggles[key],
                command=self._on_toggle_changed
            )
            cb.pack(side=tk.LEFT, padx=10, pady=2)
            
            # Add legend back to the targeted axes
            target_ax.legend(loc='upper left')

            # Auto-open timer window if this is the first timer we see
            if is_timer and not self.timer_auto_opened:
                self.show_timer_window()
                self.timer_auto_opened = True

    def _on_toggle_changed(self):
        """Callback when a user clicks a checkbox."""
        self.needs_plot_update = True

    def read_serial(self):
        """Runs in a background thread to read hardware serial."""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line_bytes = self.ser.readline()
                    line_str = line_bytes.decode("utf-8", errors="replace")
                    
                    if line_str:
                        if self.log_file and not self.log_file.closed:
                            self.log_file.write(line_str)
                            self.log_file.flush()

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
        """Runs on the main GUI thread to safely update the text area and organize plot data."""
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
                text_str = item[1]
                parsed_vars = item[2]
                text_chunk.append(text_str)
                
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
            
            # Anti-Lag Limit
            line_count = int(self.text.index('end-1c').split('.')[0])
            if line_count > 2000:
                self.text.delete('1.0', f"{line_count - 2000}.0")

            if self.autoscroll_var.get():
                self.text.see(tk.END)

        if updates_made:
            self.needs_plot_update = True
            
        self.root.after(50, self.process_queue)

    def update_plot(self):
        """Throttled plotting function running on the Main Thread."""
        if self.running and self.needs_plot_update:
            self.needs_plot_update = False
            
            main_has_visible = False
            timer_has_visible = False
            
            # Update data arrays for Matplotlib
            for key, line in self.plot_lines.items():
                dest = self.plot_destinations[key]
                if self.toggles[key].get():
                    line.set_data(list(self.plot_data[key]['x']), list(self.plot_data[key]['y']))
                    line.set_visible(True)
                    if len(self.plot_data[key]['x']) > 0:
                        if dest == 'main': main_has_visible = True
                        else: timer_has_visible = True
                else:
                    line.set_visible(False)
                    
            # Update Main Axes
            if main_has_visible:
                self.main_ax.relim()
                self.main_ax.autoscale_view(True, True, True)
                self.main_canvas.draw_idle()

            # Update Timer Axes (only if the window is open/visible to save CPU)
            if timer_has_visible and self.timer_window.winfo_ismapped():
                self.timer_ax.relim()
                self.timer_ax.autoscale_view(True, True, True)
                self.timer_canvas.draw_idle()

        # Reschedule next plot frame
        self.root.after(50, self.update_plot)


if __name__ == "__main__":
    root = tk.Tk()
    app = SerialLogger(root)
    root.mainloop()
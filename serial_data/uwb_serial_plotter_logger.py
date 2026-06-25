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
        self.root.title("UWB Serial Logger Console")
        self.root.geometry("900x700")
        self.root.minsize(600, 400)

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
        self.plot_destinations = {} # Format: {'key': 'dist' or 'stats'}
        self.toggles = {}           # Format: {'key': tk.BooleanVar}
        
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
                       '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
        self.color_idx = 0
        self.needs_plot_update = False
        self.plotter_auto_opened = False

        # UI Font defaults
        self.ui_font = ("Helvetica Neue", 12)
        self.mono_font = ("Menlo", 11)

        self._build_main_ui()
        self._build_plotter_window()
        
        # Start the Background GUI update loops
        self.root.after(50, self.process_queue)
        self.root.after(50, self.update_plot) # 20 FPS update rate for smooth plotting

    def _build_main_ui(self):
        """Builds the main layout containing only the controls and the console."""
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

        # Button to show the dedicated Plotter Window
        show_plotter_btn = ttk.Button(toolbar_frame, text="📈 Open Plotter Window", command=self.show_plotter_window)
        show_plotter_btn.pack(side=tk.RIGHT)

        # --- Console Area ---
        console_frame = ttk.Frame(self.root, padding="10 0 10 10")
        console_frame.pack(fill=tk.BOTH, expand=True)

        self.text = scrolledtext.ScrolledText(
            console_frame, 
            wrap=tk.WORD, 
            font=self.mono_font,
            bg="#1E1E1E", fg="#D4D4D4", insertbackground="white", 
            highlightthickness=0, padx=10, pady=10
        )
        self.text.pack(fill=tk.BOTH, expand=True)

        # --- Status Bar ---
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        self.status_var = tk.StringVar(value="Status: Disconnected | No file active")
        ttk.Label(status_frame, textvariable=self.status_var, font=self.ui_font, foreground="#555555").pack(side=tk.LEFT, padx=10, pady=5)

        self.refresh_ports()

    def _build_plotter_window(self):
        """Builds a completely separate window strictly for plotting data, split by scale."""
        self.plot_window = tk.Toplevel(self.root)
        self.plot_window.title("UWB Real-Time Telemetry Plotter")
        self.plot_window.geometry("1100x800")
        self.plot_window.minsize(800, 600)
        
        # Hide instead of destroy on close
        self.plot_window.protocol("WM_DELETE_WINDOW", self.plot_window.withdraw)
        
        # Force a redraw when the window is reopened to prevent blank screens
        self.plot_window.bind("<Map>", lambda e: self._force_plot_update())

        # Main horizontal split: Toggles (Left) | Plots (Right)
        main_split = ttk.PanedWindow(self.plot_window, orient=tk.HORIZONTAL)
        main_split.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 1. Left Pane: Toggles Frame
        toggles_container = ttk.Frame(main_split, width=250)
        main_split.add(toggles_container, weight=0)

        self.dist_toggles_frame = ttk.LabelFrame(toggles_container, text="Distance (Top Graph)", padding=10)
        self.dist_toggles_frame.pack(fill=tk.X, side=tk.TOP, pady=(0, 10))

        self.stats_toggles_frame = ttk.LabelFrame(toggles_container, text="Stats/Timers (Bottom Graph)", padding=10)
        self.stats_toggles_frame.pack(fill=tk.X, side=tk.TOP)

        # 2. Right Pane: Plots Split vertically
        plots_split = ttk.PanedWindow(main_split, orient=tk.VERTICAL)
        main_split.add(plots_split, weight=1)

        # --- Top Plot: Distance (Small Numbers) ---
        dist_frame = ttk.Frame(plots_split)
        plots_split.add(dist_frame, weight=1)
        
        self.dist_fig = Figure(figsize=(5, 3), dpi=100)
        self.dist_ax = self.dist_fig.add_subplot(111)
        self.dist_ax.set_title("Distance (D)", fontsize=10)
        self.dist_ax.grid(True, linestyle='--', alpha=0.6)
        
        self.dist_canvas = FigureCanvasTkAgg(self.dist_fig, master=dist_frame)
        self.dist_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.dist_toolbar = NavigationToolbar2Tk(self.dist_canvas, dist_frame)
        self.dist_toolbar.update()

        # --- Bottom Plot: Stats & Timers (Large Numbers) ---
        stats_frame = ttk.Frame(plots_split)
        plots_split.add(stats_frame, weight=1)
        
        self.stats_fig = Figure(figsize=(5, 3), dpi=100)
        self.stats_ax = self.stats_fig.add_subplot(111)
        self.stats_ax.set_title("System Stats (ok, prej, fto, err, etc.)", fontsize=10)
        self.stats_ax.grid(True, linestyle='--', alpha=0.6)
        
        self.stats_canvas = FigureCanvasTkAgg(self.stats_fig, master=stats_frame)
        self.stats_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.stats_toolbar = NavigationToolbar2Tk(self.stats_canvas, stats_frame)
        self.stats_toolbar.update()

        # Hide it immediately upon creation
        self.plot_window.withdraw()

    def show_plotter_window(self):
        """Brings the plotter window to the front."""
        self.plot_window.deiconify()
        self.plot_window.lift()
        self.needs_plot_update = True

    def _force_plot_update(self):
        """Forces the plot to refresh (useful when reopening the window)."""
        self.needs_plot_update = True

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
        for widget in self.dist_toggles_frame.winfo_children(): widget.destroy()
        for widget in self.stats_toggles_frame.winfo_children(): widget.destroy()

        # Reset Axes
        self.dist_ax.cla()
        self.dist_ax.set_title("Distance (D)", fontsize=10)
        self.dist_ax.grid(True, linestyle='--', alpha=0.6)
        
        self.stats_ax.cla()
        self.stats_ax.set_title("System Stats (ok, prej, fto, err, etc.)", fontsize=10)
        self.stats_ax.grid(True, linestyle='--', alpha=0.6)

        self.dist_canvas.draw_idle()
        self.stats_canvas.draw_idle()
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

        # Matches alphanumeric keys followed by '=' or ':' and a number
        matches = re.findall(r'([a-zA-Z0-9_]+)\s*[=:]\s*([-+]?\d*\.?\d+)', clean_line)
        for key, val_str in matches:
            try:
                extracted_data[key] = float(val_str)
            except ValueError:
                pass

        return extracted_data

    def register_new_variable(self, key):
        """Creates data structures, routes to correct plot (Dist vs Stats), and creates UI toggles."""
        if key not in self.plot_data:
            
            # Smart routing: If the key is 'D' or has 'dist' in it, send it to the top graph.
            # Otherwise, send it to the bottom graph (for ok, prej, err, fto, timers, etc.)
            is_distance = (key.upper() == 'D' or 'dist' in key.lower())
            
            dest = 'dist' if is_distance else 'stats'
            self.plot_destinations[key] = dest
            
            target_ax = self.dist_ax if is_distance else self.stats_ax
            target_frame = self.dist_toggles_frame if is_distance else self.stats_toggles_frame

            # Create data containers
            self.plot_data[key] = {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)}
            
            # Pick a color
            color = self.colors[self.color_idx % len(self.colors)]
            self.color_idx += 1
            
            # Create a line on the correct axes
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
            # Pack tightly in a vertical list
            cb.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=2)
            
            # Add legend back to the targeted axes
            target_ax.legend(loc='upper left')

            # Auto-open plotter window if this is the first variable we see
            if not self.plotter_auto_opened:
                self.show_plotter_window()
                self.plotter_auto_opened = True

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
        
        # Only do the heavy Matplotlib recalculation if there's new data/changes
        if self.needs_plot_update:
            self.needs_plot_update = False
            
            # Only redraw if the window is currently visible to save CPU!
            if self.plot_window.winfo_ismapped():
                dist_has_visible = False
                stats_has_visible = False
                
                # Update data arrays for Matplotlib
                for key, line in self.plot_lines.items():
                    dest = self.plot_destinations[key]
                    if self.toggles[key].get():
                        line.set_data(list(self.plot_data[key]['x']), list(self.plot_data[key]['y']))
                        line.set_visible(True)
                        if len(self.plot_data[key]['x']) > 0:
                            if dest == 'dist': dist_has_visible = True
                            else: stats_has_visible = True
                    else:
                        line.set_visible(False)
                        
                # Update Distance Axes (Top)
                # visible_only=True completely ignores toggled-off lines for autoscaling
                self.dist_ax.relim(visible_only=True)
                if dist_has_visible:
                    self.dist_ax.autoscale_view(True, True, True)
                self.dist_canvas.draw_idle()

                # Update Stats Axes (Bottom)
                self.stats_ax.relim(visible_only=True)
                if stats_has_visible:
                    self.stats_ax.autoscale_view(True, True, True)
                self.stats_canvas.draw_idle()

        # Reschedule next plot frame
        self.root.after(50, self.update_plot)


if __name__ == "__main__":
    root = tk.Tk()
    app = SerialLogger(root)
    root.mainloop()
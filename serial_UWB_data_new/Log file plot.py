import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog

HEADERS = [
    "roll", "pitch", "yaw", "qw", "qx", "qy", "qz", 
    "accx", "accy", "accz", "gyrox", "gyroy", "gyroz", 
    "svm", "state", "z_v", "lever_offx", "lever_offy", 
    "steps", "stride_m", "velocity_mps", "cadence_spm", 
    "cadence_sps", "distance_m", "step_flag", "pwm" # Added 'pwm' here assuming it exists in your newer logs
]

def parse_sensor_log(file_path):
    """
    Parses the raw serial log file, ignoring calibration metadata 
    and phase strings, extracting clean CSV data rows.
    """
    cleaned_data = []
    
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Could not find the file: {file_path}. Please check the path!")

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            
            # Skip empty lines, initialization alerts, header strings, and phase calculation summaries
            if not line or "BNO055" in line or "phase1" in line or line.startswith("roll"):
                continue
                
            # Split comma-separated tokens
            tokens = line.split(',')
            
            # Verify if row matches the expected column structure
            if len(tokens) == len(HEADERS):
                try:
                    # Convert elements to floats
                    numeric_row = [float(val) for val in tokens]
                    cleaned_data.append(numeric_row)
                except ValueError:
                    # Gracefully skip any malformed rows that fail numerical conversion
                    continue
            # If your log DOESN'T output PWM yet but you want the code ready for it, 
            # we can handle the old length vs new length robustly:
            elif len(tokens) == len(HEADERS) - 1:
                 try:
                    numeric_row = [float(val) for val in tokens]
                    numeric_row.append(0.0) # pad PWM with 0 if missing
                    cleaned_data.append(numeric_row)
                 except ValueError:
                    continue
                    
    # Pack into a Pandas DataFrame
    df = pd.DataFrame(cleaned_data, columns=HEADERS)
    return df

def plot_data(df):
    """
    Generates a stacked subplot visualizer showing the accelerometer profiles,
    PWM signals, velocity, and progressive step counters.
    """
    time_axis = df.index
    
    # Setup subplots (3 rows now to accommodate new data)
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    
    # --- Plot 1: Accelerometer Tracks & Signal Vector Magnitude (SVM) ---
    ax1.plot(time_axis, df['accx'], label='Acc X', color='#3498db', alpha=0.8)
    ax1.plot(time_axis, df['accy'], label='Acc Y', color='#2ecc71', alpha=0.8)
    ax1.plot(time_axis, df['accz'], label='Acc Z', color='#e74c3c', alpha=0.8)
    ax1.plot(time_axis, df['svm'], label='SVM (Mag)', color='#9b59b6', linestyle='--', linewidth=1.5)
    
    ax1.set_title("IMU Accelerometer Readings & Signal Magnitude", fontsize=12, fontweight='bold')
    ax1.set_ylabel("Acceleration (m/s²)", fontsize=10)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')
    
    # --- Plot 2: Velocity & PWM Signals ---
    ax2.plot(time_axis, df['velocity_mps'], label='Velocity (m/s)', color='#e67e22', linewidth=2)
    
    ax2_twin = ax2.twinx()
    # Assuming PWM is a percentage or raw bit value; plotting it on a secondary Y-axis
    ax2_twin.plot(time_axis, df['pwm'], label='PWM Signal', color='#34495e', alpha=0.7, linestyle='-')
    
    ax2.set_title("Kinematics & Control Signals", fontsize=12, fontweight='bold')
    ax2.set_ylabel("Velocity (m/s)", color='#e67e22', fontsize=10)
    ax2_twin.set_ylabel("PWM Output", color='#34495e', fontsize=10)
    
    ax2.grid(True, linestyle=':', alpha=0.6)
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    lines_2t, labels_2t = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines_2 + lines_2t, labels_2 + labels_2t, loc='upper left')

    # --- Plot 3: Pedometer (Steps) & Distance Tracks ---
    ax3.plot(time_axis, df['steps'], label='Step Count', color='#f39c12', linewidth=2.5)
    
    ax3_twin = ax3.twinx()
    ax3_twin.plot(time_axis, df['distance_m'], label='Distance (m)', color='#1abc9c', linestyle='-.')
    
    ax3.set_title("Pedometer Tracking Metrics", fontsize=12, fontweight='bold')
    ax3.set_xlabel("Sample Index", fontsize=10)
    ax3.set_ylabel("Total Steps", color='#f39c12', fontsize=10)
    ax3_twin.set_ylabel("Distance Traveled (m)", color='#1abc9c', fontsize=10)
    
    ax3.grid(True, linestyle=':', alpha=0.6)
    
    lines_3, labels_3 = ax3.get_legend_handles_labels()
    lines_3t, labels_3t = ax3_twin.get_legend_handles_labels()
    ax3.legend(lines_3 + lines_3t, labels_3 + labels_3t, loc='upper left')
    
    plt.tight_layout()
    plt.show()

def get_file_path_gui():
    """
    Opens a file dialog for the user to select the log file.
    Returns the selected file path or None if cancelled.
    """
    # Hide the main tkinter window
    root = tk.Tk()
    root.withdraw()
    
    # Open the file dialog
    file_path = filedialog.askopenfilename(
        title="Select Sensor Log File",
        filetypes=(("Text files", "*.txt"), ("CSV files", "*.csv"), ("All files", "*.*"))
    )
    
    return file_path

if __name__ == "__main__":
    
    # Use the GUI file picker
    print("Opening file dialog to select the log file...")
    log_file_path = get_file_path_gui()

    if not log_file_path:
        print("File selection cancelled. Exiting.")
        sys.exit()

    try:
        print(f"\nParsing raw sensor logs from: {log_file_path}...")
        sensor_df = parse_sensor_log(log_file_path)
        
        if sensor_df.empty:
            print("Warning: The parsed DataFrame is empty. No valid data was found.")
        else:
            print(f"Successfully processed {len(sensor_df)} data points.")
            print("Rendering performance plots...")
            plot_data(sensor_df)
            
    except Exception as e:
        print(f"\nAn error occurred: {e}")
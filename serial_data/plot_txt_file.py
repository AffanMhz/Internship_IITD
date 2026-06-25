import re
import tkinter as tk
from tkinter import filedialog
import matplotlib.pyplot as plt
import sys

def select_file():
    """Opens a file dialog to select the log file."""
    # Initialize tkinter and hide the main window
    root = tk.Tk()
    root.withdraw() 
    
    # Open the file picker dialog
    file_path = filedialog.askopenfilename(
        title="Select Log File",
        filetypes=(("Text/Log files", "*.txt *.log"), ("All files", "*.*"))
    )
    return file_path

def parse_log_file(file_path):
    """Parses the log file for distance measurements."""
    distances = []
    labels = []
    counter = 1

    # Regex to catch format 2: "Distance OOB: 9097.020 m"
    oob_pattern = re.compile(r"Distance OOB:\s*(-?[\d.]+)")

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                val = None
                
                # Handle format 1: D:1.24
                if line.startswith('D:'):
                    try:
                        val = float(line.replace('D:', ''))
                    except ValueError:
                        pass
                        
                # Handle format 2: [ANCHOR] Distance OOB: 9097.020 m
                elif 'Distance OOB:' in line:
                    match = oob_pattern.search(line)
                    if match:
                        try:
                            val = float(match.group(1))
                        except ValueError:
                            pass
                
                # If a valid float was found, add it to our lists
                if val is not None:
                    distances.append(val)
                    labels.append(counter)
                    counter += 1
                    
    except Exception as e:
        print(f"Error reading file: {e}")
        return None, None
        
    return labels, distances

def plot_data(labels, distances):
    """Calculates stats and renders the matplotlib graph."""
    if not distances:
        print("No valid distance measurements found in the file.")
        return

    # Calculate Statistics
    min_dist = min(distances)
    max_dist = max(distances)
    avg_dist = sum(distances) / len(distances)

    # Print Dashboard to Console
    print("\n" + "="*40)
    print("DISTANCE LOG ANALYSIS")
    print("="*40)
    print(f"Total Measurements: {len(distances)}")
    print(f"Minimum Distance:   {min_dist:.2f}")
    print(f"Maximum Distance:   {max_dist:.2f}")
    print(f"Average Distance:   {avg_dist:.2f}")
    print("="*40 + "\n")

    # Render Chart
    plt.figure(figsize=(10, 6))
    
    # Plot line and fill under it (similar to the Chart.js style)
    plt.plot(labels, distances, label='Distance', color='#3b82f6', linewidth=2)
    plt.fill_between(labels, distances, color='#3b82f6', alpha=0.1)
    
    plt.title('Interactive Distance Plot')
    plt.xlabel('Measurement Index')
    plt.ylabel('Distance Value')
    
    # Add a subtle grid
    plt.grid(True, linestyle='--', alpha=0.6)
    
    # Dynamically pad the y-axis
    y_range = max_dist - min_dist
    padding = y_range * 0.05 if y_range > 0 else 0.5
    plt.ylim(min_dist - padding, max_dist + padding)
    
    # Show the plot
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    print("Opening file dialog... Please select a log file.")
    
    # 1. Get file from user
    file_path = select_file()
    
    if not file_path:
        print("No file selected. Exiting.")
        sys.exit()
        
    print(f"Processing: {file_path}")
    
    # 2. Extract data
    labels, distances = parse_log_file(file_path)
    
    # 3. Show stats and graph
    if labels is not None and distances is not None:
        plot_data(labels, distances)
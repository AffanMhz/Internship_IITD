import matplotlib.pyplot as plt
import sys

def plot_distances(filepath):
    distances = []
    
    try:
        # Open and read the log file
        with open(filepath, 'r') as file:
            for line in file:
                line = line.strip()
                # We only care about lines that start with 'D:'
                if line.startswith('D:'):
                    try:
                        # Extract the numeric part after 'D:' and convert to float
                        dist_val = float(line[2:])
                        # Filter out the -1 sentinel values we used for out-of-range
                        if dist_val != -1.0:
                            distances.append(dist_val)
                    except ValueError:
                        pass # Ignore malformed lines
                        
    except FileNotFoundError:
        print(f"Error: Could not find the file '{filepath}'.")
        return

    if not distances:
        print("No valid distance data ('D:xx.xx') found in the file.")
        return

    # Create the plot
    plt.figure(figsize=(12, 6))
    plt.plot(distances, marker='.', linestyle='-', color='#1f77b4', markersize=4, alpha=0.8)
    
    # Formatting the plot
    plt.title(f'DW1000 Distance Measurements ({filepath})', fontsize=14)
    plt.xlabel('Measurement Index (Successful Reads)', fontsize=12)
    plt.ylabel('Distance (Meters)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # Adjust layout and display
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # You can change this to match the name of your text file
    input_file = 'log_015.txt'
    
    # Allow passing file via command line argument (e.g., python plot_distances.py log_015.txt)
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
        
    print(f"Reading data from {input_file}...")
    plot_distances(input_file)
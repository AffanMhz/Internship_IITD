import matplotlib.pyplot as plt
from datetime import datetime

def analyze_serial_data(filename):
    times = []
    distances = []
    start_time = None
    
    print(f"Reading and parsing '{filename}'...")
    
    try:
        with open(filename, 'r') as file:
            for line in file:
                # Filter strictly for lines containing the distance data
                if '] D:' in line:
                    try:
                        # Split into timestamp and data (e.g., "[16:56:48.3570] D:1.53")
                        time_part, data_part = line.split('] D:')
                        
                        # Clean up the strings to get the raw values
                        time_str = time_part.split('[')[-1].strip()
                        distance = float(data_part.strip())
                        
                        # Parse the timestamp string into a datetime object
                        time_obj = datetime.strptime(time_str, "%H:%M:%S.%f")
                        
                        # Set the baseline time on the first valid read
                        if start_time is None:
                            start_time = time_obj
                            
                        # Calculate elapsed seconds for the X-axis
                        elapsed_seconds = (time_obj - start_time).total_seconds()
                        
                        times.append(elapsed_seconds)
                        distances.append(distance)
                    except (ValueError, IndexError):
                        # Silently skip any malformed lines
                        continue
    except FileNotFoundError:
        print(f"Error: Could not find the file '{filename}' in the current directory.")
        return

    if not distances:
        print("No valid distance data found. Check your file format.")
        return

    # --- Calculations ---
    total_samples = len(distances)
    avg_distance = sum(distances) / total_samples
    total_time = times[-1] - times[0]
    
    avg_data_rate = total_samples / total_time if total_time > 0 else 0
    
    print("\n--- Analysis Results ---")
    print(f"Total Distance Samples: {total_samples}")
    print(f"Total Duration:         {total_time:.2f} seconds")
    print(f"Average Distance:       {avg_distance:.3f}")
    print(f"Average Data Rate:      {avg_data_rate:.2f} Hz")
    
    # --- Plotting ---
    print("\nGenerating plot...")
    plt.figure(figsize=(12, 5))
    plt.plot(times, distances, label='Distance Data', color='#1f77b4', linewidth=1)
    
    # Add a horizontal line representing the average
    plt.axhline(y=avg_distance, color='red', linestyle='--', label=f'Avg Distance: {avg_distance:.3f}')
    
    # Formatting the chart
    plt.title('Serial Output: Distance vs. Elapsed Time')
    plt.xlabel('Elapsed Time (Seconds)')
    plt.ylabel('Distance')
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    
    # Display the window
    plt.show()

if __name__ == "__main__":
    # Ensure this matches the name of your text file exactly
    analyze_serial_data("log_033.txt")
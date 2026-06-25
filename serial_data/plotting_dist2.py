import re
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.stats import zscore

# ==========================
# CONFIGURATION
# ==========================
LOG_FILE = "data logging _IIT.txt"       # Change filename
TRUE_DISTANCE = None       # Example: 3.4 (meters)
MOVING_AVG_WINDOW = 10
# ==========================

# Extract distances
distances = []

with open(LOG_FILE, "r", errors="ignore") as f:
    for line in f:
        m = re.search(r'Distance.*?([0-9]+\.[0-9]+)', line)
        if m:
            distances.append(float(m.group(1)))

distances = np.array(distances)

if len(distances) == 0:
    print("No distance values found!")
    exit()

# ==========================
# Statistics
# ==========================
mean_val = np.mean(distances)
median_val = np.median(distances)
std_val = np.std(distances)
min_val = np.min(distances)
max_val = np.max(distances)
peak_to_peak = max_val - min_val

print("\n========== STATISTICS ==========")
print(f"Samples           : {len(distances)}")
print(f"Mean              : {mean_val:.3f} m")
print(f"Median            : {median_val:.3f} m")
print(f"Std Dev           : {std_val:.3f} m")
print(f"Min               : {min_val:.3f} m")
print(f"Max               : {max_val:.3f} m")
print(f"Peak-Peak Noise   : {peak_to_peak:.3f} m")

if TRUE_DISTANCE is not None:
    error = distances - TRUE_DISTANCE

    print(f"\nTrue Distance     : {TRUE_DISTANCE:.3f} m")
    print(f"Mean Error        : {np.mean(error):.3f} m")
    print(f"RMS Error         : {np.sqrt(np.mean(error**2)):.3f} m")
    print(f"Max Error         : {np.max(np.abs(error)):.3f} m")

# ==========================
# Moving Average
# ==========================
moving_avg = np.convolve(
    distances,
    np.ones(MOVING_AVG_WINDOW)/MOVING_AVG_WINDOW,
    mode='same'
)

# ==========================
# Outlier Detection
# ==========================
z = np.abs(zscore(distances))
outliers = np.where(z > 3)[0]

print(f"\nOutliers (>3σ): {len(outliers)}")

# ==========================
# Autocorrelation
# ==========================
autocorr = signal.correlate(
    distances - mean_val,
    distances - mean_val,
    mode='full'
)
autocorr = autocorr[len(autocorr)//2:]


# ==========================
# Cumulative Mean
# ==========================
cum_mean = np.cumsum(distances) / np.arange(1, len(distances) + 1)


# ==========================
# FFT
# ==========================
fft_vals = np.abs(np.fft.rfft(distances - mean_val))
freqs = np.fft.rfftfreq(len(distances))


# ==========================
# Plotting
# ==========================

# 1. Primary Plot: Distance vs Sample Number
plt.figure(figsize=(10, 6))
plt.plot(distances, label='Distance', alpha=0.6)
plt.plot(moving_avg, linewidth=3, label=f'Moving Avg (win={MOVING_AVG_WINDOW})')
plt.scatter(outliers, distances[outliers], marker='x', color='red', s=80, label='Outliers')
plt.title("Distance vs Sample Number")
plt.xlabel("Sample Number")
plt.ylabel("Distance (m)")
plt.legend()
plt.grid(True)
plt.tight_layout()

# 2. Secondary Plots: Statistical Analysis
fig, axs = plt.subplots(2, 3, figsize=(16, 10))

# Histogram
axs[0,0].hist(distances, bins=30, color='gray', alpha=0.7)
axs[0,0].set_title("Distance Histogram")
axs[0,0].grid(True)

# Error/Deviation plot
if TRUE_DISTANCE is not None:
    axs[0,1].plot(error, color='orange')
    axs[0,1].axhline(np.mean(error), linestyle='--', color='black')
    axs[0,1].set_title("Error vs Sample")
else:
    axs[0,1].plot(distances - mean_val, color='orange')
    axs[0,1].set_title("Deviation from Mean")
axs[0,1].grid(True)

# Autocorrelation
axs[0,2].plot(autocorr, color='green')
axs[0,2].set_title("Autocorrelation")
axs[0,2].grid(True)

# FFT
axs[1,0].plot(freqs, fft_vals, color='purple')
axs[1,0].set_title("FFT of Noise")
axs[1,0].grid(True)

# Cumulative Mean
axs[1,1].plot(cum_mean, color='brown')
axs[1,1].set_title("Cumulative Mean")
axs[1,1].grid(True)

# Hide the last empty subplot (if 2x3 grid)
axs[1,2].axis('off')

plt.tight_layout()
plt.show()
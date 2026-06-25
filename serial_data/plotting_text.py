import re
import matplotlib.pyplot as plt
import numpy as np

files = [
    "log_001.txt"
]

distance_pattern = re.compile(r"Distance:\s*([0-9.]+)\s*m")

plt.figure(figsize=(14, 7))

stats = []

for file in files:
    distances = []

    with open(file, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            match = distance_pattern.search(line)
            if match:
                distances.append(float(match.group(1)))

    if len(distances) == 0:
        print(f"No distance data found in {file}")
        continue

    distances = np.array(distances)

    stats.append({
        "file": file,
        "samples": len(distances),
        "mean": np.mean(distances),
        "std": np.std(distances),
        "min": np.min(distances),
        "max": np.max(distances),
        "peak_to_peak": np.max(distances) - np.min(distances)
    })

    plt.plot(
        np.arange(len(distances)),
        distances,
        linewidth=1,
        label=file
    )

plt.title("UWB Distance Comparison")
plt.xlabel("Measurement Number")
plt.ylabel("Distance (m)")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.show()

print("\n===== STATISTICS =====\n")

for s in stats:
    print(f"\n{s['file']}")
    print(f"Samples      : {s['samples']}")
    print(f"Mean         : {s['mean']:.3f} m")
    print(f"Std Dev      : {s['std']:.3f} m")
    print(f"Min          : {s['min']:.3f} m")
    print(f"Max          : {s['max']:.3f} m")
    print(f"Peak-Peak    : {s['peak_to_peak']:.3f} m")
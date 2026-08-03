import re
from pathlib import Path
import matplotlib.pyplot as plt

# ==========================================================
# Universal Distance Plotter
# Supports:
#   D:1.23
#   Distance: 1.23 m
#   *** SUCCESS! Distance: 1.23 m ***
#   Distance OOB: -10.5 m
# ==========================================================

# Folder containing log files
FOLDER = Path(".")

# Supported extensions
EXTENSIONS = ("*.txt", "*.log", "*.csv")

# Regex patterns
patterns = [
    re.compile(r"\bD:([-+]?\d*\.?\d+)"),
    re.compile(r"SUCCESS!\s*Distance:\s*([-+]?\d*\.?\d+)"),
    re.compile(r"Distance OOB:\s*([-+]?\d*\.?\d+)"),
    re.compile(r"Distance:\s*([-+]?\d*\.?\d+)"),
]


def extract_distance(line):
    """Extract first matching distance from a line."""
    for p in patterns:
        m = p.search(line)
        if m:
            try:
                return float(m.group(1))
            except:
                pass
    return None


files = []

for ext in EXTENSIONS:
    files.extend(sorted(FOLDER.glob(ext)))

if not files:
    print("No log files found.")
    exit()

for file in files:

    distances = []

    with open(file, "r", errors="ignore") as f:
        for line in f:
            d = extract_distance(line)
            if d is not None:
                distances.append(d)

    if len(distances) == 0:
        print(f"Skipped {file.name} (no distance data)")
        continue

    plt.figure(figsize=(10,5))
    plt.plot(
        range(1, len(distances)+1),
        distances,
        linewidth=1.5
    )

    plt.title(file.stem)
    plt.xlabel("Sample Number")
    plt.ylabel("Distance (m)")
    plt.grid(True)

    plt.tight_layout()

    output_name = f"plot_{file.stem}.png"

    plt.savefig(output_name, dpi=300)
    plt.close()

    print(f"Saved {output_name}")

print("\nFinished.")
import csv
import statistics

distances = [1960, 2920, 4800, 4970, 5750]
files = ['1960mm.csv', '2920mm.csv', '4800mm.csv', '4970mm.csv', '5750mm.csv']

all_data = {}

for i, file in enumerate(files):
    distance = distances[i]
    voltages = {'pin2': [], 'pin3': [], 'pin4': [], 'pin5': []}

    with open(file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            voltages['pin2'].append(float(row['Pin2_V']))
            voltages['pin3'].append(float(row['Pin3_V']))
            voltages['pin4'].append(float(row['Pin4_V']))
            voltages['pin5'].append(float(row['Pin5_V']))

    # Calculate average voltages for each pin
    avg_voltages = {
        'pin2': statistics.mean(voltages['pin2']),
        'pin3': statistics.mean(voltages['pin3']),
        'pin4': statistics.mean(voltages['pin4']),
        'pin5': statistics.mean(voltages['pin5'])
    }

    all_data[distance] = avg_voltages
    print(f"Distance: {distance}mm")
    print(f"  Pin2: {avg_voltages['pin2']:.3f}V")
    print(f"  Pin3: {avg_voltages['pin3']:.3f}V")
    print(f"  Pin4: {avg_voltages['pin4']:.3f}V")
    print(f"  Pin5: {avg_voltages['pin5']:.3f}V")
    print()

# Extract data for analysis
pin2_voltages = [all_data[d]['pin2'] for d in distances]
pin3_voltages = [all_data[d]['pin3'] for d in distances]
pin4_voltages = [all_data[d]['pin4'] for d in distances]
pin5_voltages = [all_data[d]['pin5'] for d in distances]

print("=== ANALYSIS RESULTS ===")
print()

# 1. Simple linear correlation analysis
def simple_correlation(x, y):
    n = len(x)
    sum_x = sum(x)
    sum_y = sum(y)
    sum_xy = sum(x[i] * y[i] for i in range(n))
    sum_x2 = sum(x[i] * x[i] for i in range(n))
    sum_y2 = sum(y[i] * y[i] for i in range(n))

    # Linear regression: y = ax + b
    a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
    b = (sum_y - a * sum_x) / n

    return a, b

print("1. Linear relationships (Distance vs Voltage):")
pins_data = [pin2_voltages, pin3_voltages, pin4_voltages, pin5_voltages]
pin_names = ['Pin2', 'Pin3', 'Pin4', 'Pin5']

for pin_data, pin_name in zip(pins_data, pin_names):
    a, b = simple_correlation(distances, pin_data)
    print(f"{pin_name}: Voltage = {a:.6f} * Distance + {b:.3f}")

print()

# 2. Voltage ratios and differences
print("2. Voltage patterns:")
for distance in distances:
    data = all_data[distance]
    print(f"Distance: {distance}mm")
    print(f"  Pin3/Pin2: {data['pin3']/data['pin2']:.4f}")
    print(f"  Pin4/Pin2: {data['pin4']/data['pin2']:.4f}")
    print(f"  Pin5/Pin2: {data['pin5']/data['pin2']:.4f}")
    print(f"  Total: {sum(data.values()):.3f}")
    print(f"  Average: {sum(data.values())/4:.3f}")

print()

# 3. Look for binary patterns (digital encoding)
print("3. Potential digital patterns (thresholds around 3.6V):")
threshold = 3.6
for distance in distances:
    data = all_data[distance]
    binary = []
    for pin in ['pin2', 'pin3', 'pin4', 'pin5']:
        binary.append('1' if data[pin] > threshold else '0')
    print(f"Distance {distance}mm: {''.join(binary)} (decimal: {int(''.join(binary), 2)})")

print()
print("=== SIMPLE REVERSE FORMULA ===")
print("Based on the patterns, here are some approaches:")
print("1. Use voltage sum/average as distance indicator")
print("2. Use digital binary pattern if consistent")
print("3. Use individual pin voltage thresholds")
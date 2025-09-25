# Check calibration accuracy
distances = [1960, 2920, 4800, 4970, 5750]
avg_voltages = [3.617, 3.602, 3.675, 3.647, 3.691]

# Linear regression: y = mx + b where y=distance, x=avg_voltage
# Using least squares method
n = len(distances)
sum_x = sum(avg_voltages)
sum_y = sum(distances)
sum_xy = sum(distances[i] * avg_voltages[i] for i in range(n))
sum_x2 = sum(avg_voltages[i] * avg_voltages[i] for i in range(n))

m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
b = (sum_y - m * sum_x) / n

print(f"Calibration formula: Distance = {m:.1f} * avg_voltage + {b:.1f}")
print()

# Test accuracy
print("Distance  | Actual | Predicted | Error")
print("----------|--------|-----------|-------")
for i in range(n):
    predicted = m * avg_voltages[i] + b
    error = abs(distances[i] - predicted)
    print(f"{distances[i]:8}mm | {distances[i]:6}mm | {predicted:8.0f}mm | {error:.0f}mm")

print(f"\nFormula for Arduino: distance = {m:.1f} * avgVoltage + {b:.1f}")
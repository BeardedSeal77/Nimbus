/*
 * Makita LD050P Distance Decoder - Reverse Engineered
 * Reads FPC pins and calculates distance based on voltage patterns
 */

int pins[4] = {2, 3, 4, 5};

void setup() {
  Serial.begin(115200);

  for(int i = 0; i < 4; i++) {
    pinMode(pins[i], INPUT_PULLUP);
  }

  Serial.println("Makita LD050P Distance Decoder Ready");
  Serial.println("Distance(mm), Pin2_V, Pin3_V, Pin4_V, Pin5_V, Avg_V");
}

float calculateDistance(float avgVoltage) {
  // Linear regression formula based on analysis:
  // Distance = (avgVoltage - 3.5486) / 0.000021
  // Simplified and calibrated:
  return (avgVoltage - 3.549) * 47600;
}

float getDistance() {
  float voltages[4];
  float sum = 0;

  // Read all 4 pins multiple times for stability
  for(int sample = 0; sample < 10; sample++) {
    for(int i = 0; i < 4; i++) {
      int analogVal = analogRead(pins[i]);
      voltages[i] = analogVal * (5.0 / 1023.0);
      sum += voltages[i];
    }
    delay(10);
  }

  float avgVoltage = sum / 40.0; // 10 samples * 4 pins
  return calculateDistance(avgVoltage);
}

void loop() {
  float voltages[4];
  float sum = 0;

  // Read current voltages
  for(int i = 0; i < 4; i++) {
    int analogVal = analogRead(pins[i]);
    voltages[i] = analogVal * (5.0 / 1023.0);
    sum += voltages[i];
  }

  float avgVoltage = sum / 4.0;
  float distance = calculateDistance(avgVoltage);

  // Output results
  Serial.print(distance, 0);
  Serial.print(", ");
  Serial.print(voltages[0], 3);
  Serial.print(", ");
  Serial.print(voltages[1], 3);
  Serial.print(", ");
  Serial.print(voltages[2], 3);
  Serial.print(", ");
  Serial.print(voltages[3], 3);
  Serial.print(", ");
  Serial.println(avgVoltage, 3);

  delay(500);
}
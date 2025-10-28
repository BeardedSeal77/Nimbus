/*
 * Makita LD050P Diagnostic Reader
 * Use this to recalibrate at known distances
 */

int pins[4] = {2, 3, 4, 5};

void setup() {
  Serial.begin(115200);

  for(int i = 0; i < 4; i++) {
    pinMode(pins[i], INPUT_PULLUP);
  }

  Serial.println("=== MAKITA LD050P DIAGNOSTIC MODE ===");
  Serial.println("Set laser to known distance, then note the average voltage");
  Serial.println("Iteration, Pin2_V, Pin3_V, Pin4_V, Pin5_V, Avg_V, Display_Distance");
  delay(2000);
}

float getStableVoltage(int pin) {
  float sum = 0;
  for(int i = 0; i < 50; i++) {
    sum += analogRead(pin) * (5.0 / 1023.0);
    delay(2);
  }
  return sum / 50.0;
}

void loop() {
  static int iteration = 1;
  float voltages[4];
  float sum = 0;

  // Get very stable readings
  for(int i = 0; i < 4; i++) {
    voltages[i] = getStableVoltage(pins[i]);
    sum += voltages[i];
  }

  float avgVoltage = sum / 4.0;

  // Output data for manual calibration
  Serial.print(iteration);
  Serial.print(", ");
  Serial.print(voltages[0], 3);
  Serial.print(", ");
  Serial.print(voltages[1], 3);
  Serial.print(", ");
  Serial.print(voltages[2], 3);
  Serial.print(", ");
  Serial.print(voltages[3], 3);
  Serial.print(", ");
  Serial.print(avgVoltage, 3);
  Serial.println(", [ENTER_DISTANCE_HERE]");

  iteration++;
  delay(1000);
}
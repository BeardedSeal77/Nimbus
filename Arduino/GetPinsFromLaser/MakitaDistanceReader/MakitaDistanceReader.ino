/*
 * Makita LD050P Distance Reader - Accurate Lookup Table Method
 * Reads FPC pins C1 and interpolates distance based on calibrated data
 */

int pins[4] = {2, 3, 4, 5};

// Calibration data from measurements
struct CalibrationPoint {
  float avgVoltage;
  int distanceMM;
};

CalibrationPoint calibration[] = {
  {3.602, 1960},   // Shortest measured
  {3.617, 2920},   //
  {3.647, 4800},   //
  {3.675, 4970},   //
  {3.691, 5750}    // Longest measured
};

const int numCalPoints = sizeof(calibration) / sizeof(CalibrationPoint);

void setup() {
  Serial.begin(115200);

  for(int i = 0; i < 4; i++) {
    pinMode(pins[i], INPUT);  // Remove pullup resistors
  }

  Serial.println("Makita LD050P Distance Reader");
  Serial.println("Distance(mm), Confidence, Pin2_V, Pin3_V, Pin4_V, Pin5_V, Avg_V");
  delay(1000);
}

float getStableVoltage(int pin) {
  float sum = 0;
  for(int i = 0; i < 20; i++) {
    sum += analogRead(pin) * (5.0 / 1023.0);
    delay(5);
  }
  return sum / 20.0;
}

int interpolateDistance(float avgVoltage) {
  // Handle out of range cases
  if(avgVoltage <= calibration[0].avgVoltage) {
    return calibration[0].distanceMM;
  }
  if(avgVoltage >= calibration[numCalPoints-1].avgVoltage) {
    return calibration[numCalPoints-1].distanceMM;
  }

  // Find bracketing points and interpolate
  for(int i = 0; i < numCalPoints-1; i++) {
    if(avgVoltage >= calibration[i].avgVoltage &&
       avgVoltage <= calibration[i+1].avgVoltage) {

      float ratio = (avgVoltage - calibration[i].avgVoltage) /
                    (calibration[i+1].avgVoltage - calibration[i].avgVoltage);

      return calibration[i].distanceMM +
             ratio * (calibration[i+1].distanceMM - calibration[i].distanceMM);
    }
  }

  return -1; // Error case
}

String getConfidence(float avgVoltage) {
  if(avgVoltage < 3.59 || avgVoltage > 3.70) {
    return "LOW";
  } else if(avgVoltage < 3.60 || avgVoltage > 3.69) {
    return "MEDIUM";
  } else {
    return "HIGH";
  }
}

void loop() {
  float voltages[4];
  float sum = 0;

  // Get stable readings
  for(int i = 0; i < 4; i++) {
    voltages[i] = getStableVoltage(pins[i]);
    sum += voltages[i];
  }

  float avgVoltage = sum / 4.0;
  int distance = interpolateDistance(avgVoltage);
  String confidence = getConfidence(avgVoltage);

  // Output: Distance, Confidence, Pin voltages, Average
  Serial.print(distance);
  Serial.print(", ");
  Serial.print(confidence);
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

  delay(250);
}
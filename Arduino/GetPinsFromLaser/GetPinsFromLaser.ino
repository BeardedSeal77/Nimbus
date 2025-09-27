/*
 * Makita Laser Rangefinder FPC Signal Analyzer - CSV Output
 * Clean CSV output for data analysis across multiple distances
 */

int pins[4] = {2, 3, 4, 5};
int iterationCount = 0;
const int MAX_ITERATIONS = 20;

void setup() {
  Serial.begin(115200);
  
  // Set all pins as inputs with pullup resistors
  for(int i = 0; i < 4; i++) {
    pinMode(pins[i], INPUT_PULLUP);
  }
  
  // CSV Header
  Serial.println("Iteration,Pin2_V,Pin3_V,Pin4_V,Pin5_V");
  
  delay(1000);
}

void loop() {
  if(iterationCount >= MAX_ITERATIONS) {
    Serial.println("# COMPLETE - 20 iterations finished");
    while(1); // Stop here forever
  }
  
  // Read all 4 pins and convert to voltages
  float voltages[4];
  for(int i = 0; i < 4; i++) {
    int analogVal = analogRead(pins[i]);
    voltages[i] = analogVal * (5.0 / 1023.0);
  }
  
  // Output clean CSV line
  Serial.print(iterationCount + 1);
  Serial.print(",");
  Serial.print(voltages[0], 3); // 3 decimal places
  Serial.print(",");
  Serial.print(voltages[1], 3);
  Serial.print(",");
  Serial.print(voltages[2], 3);
  Serial.print(",");
  Serial.println(voltages[3], 3);
  
  iterationCount++;
  delay(250); // Sample every 250ms for stable readings
}
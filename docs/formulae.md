#include <Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Servo myServo;
Adafruit_INA219 ina219_fixed;
Adafruit_INA219 ina219_rotating;

const int LDR1_PIN = A0;
const int LDR2_PIN = A1;

void setup() {
    Serial.begin(9600);
    myServo.attach(9);
    ina219_fixed.begin(0x40);
    ina219_rotating.begin(0x41);
}

void loop() {
    int ldr1Value = analogRead(LDR1_PIN);
    int ldr2Value = analogRead(LDR2_PIN);
    
    float voltage_fixed = ina219_fixed.getBusVoltage_V();
    float current_fixed = ina219_fixed.getCurrent_mA();
    float power_fixed = voltage_fixed * (current_fixed / 1000.0); // Convert mA to A

    float voltage_rotating = ina219_rotating.getBusVoltage_V();
    float current_rotating = ina219_rotating.getCurrent_mA();
    float power_rotating = voltage_rotating * (current_rotating / 1000.0); // Convert mA to A

    // Calculate offset and smooth readings
    float offset = (ldr1Value - ldr2Value) / 2.0; // Simple offset calculation
    float smoothDiff = 0.0; // Placeholder for EMA calculation

    // Control servo based on LDR readings
    if (ldr1Value > ldr2Value) {
        myServo.write(0); // Rotate to LDR1
    } else {
        myServo.write(180); // Rotate to LDR2
    }

    // Send data to Python dashboard
    Serial.print(ldr1Value);
    Serial.print(",");
    Serial.print(ldr2Value);
    Serial.print(",");
    Serial.print(smoothDiff);
    Serial.print(",");
    Serial.print(myServo.read());
    Serial.print(",");
    Serial.print(voltage_fixed);
    Serial.print(",");
    Serial.print(current_fixed);
    Serial.print(",");
    Serial.print(power_fixed);
    Serial.print(",");
    Serial.print(voltage_rotating);
    Serial.print(",");
    Serial.print(current_rotating);
    Serial.print(",");
    Serial.println(power_rotating);

    delay(1000); // Delay for stability
}
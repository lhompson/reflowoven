// DIY Reflow Oven temperature control code.
// Written by Lee Thompson

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// For SPI, DO -> pin 12, CLK -> 13, CS -> 10
const int CS = 10;
SPISettings mySettting(10000, LSBFIRST, SPI_MODE0);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int pulselength;

int servopin = 9;
int vccpin = 8;  // Pin to power the MAX6675 thermocouple.
int timer = 0;
int pos = 0;
int tempReading;
int index = 0;

int tempGoal;
int reading;
int error = 0;
int previouserror = error;
int pidsum;
int proportional;
int dt = 3;

// PID variables. Do not recommend changing these.
float P = 4;

// Temperature profile that the oven should follow.
// For each goal, specify a dwell time that matches index.
int goals[] = {200, 250, 25};
int dwells[] = {60, 20, 200};

void setup() {  
  Serial.begin(9600);
  pinMode(CS, OUTPUT);
  pinMode(vccpin, OUTPUT);
  SPI.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency
}

int getTemperature()  {
  digitalWrite(CS, LOW);
  tempReading = SPI.transfer16(0);
  digitalWrite(CS, HIGH);
  // The bitwise AND pulls the temperature from the data.
  // The -25 at the end is a calibration specific to my sensor. May need to change.
  return abs(map(tempReading & 0x7FF8, 0, 16383, 0, 1024) - 25);
}

void setServo(int pos) {
    if (pos >= 0) {
      pulselength = map(pos, 0, 180, 150, 450); // 515);
      pwm.setPWM(0, 0, pulselength);
      pwm.setPWM(1, 0, 125);
    }
    else {
      pulselength = map(pos, -180, 0, 450, 125);
      pwm.setPWM(1, 0, pulselength);
      pwm.setPWM(0, 0, 150);
    }
}

void loop() {
  // Power on the MAX6675.
  digitalWrite(vccpin, HIGH);

  while (true) {
    getTemperature();  // Have seen weird first reading transients without this.
    tempGoal = goals[index];
    error = tempGoal - getTemperature();
    
    while (abs(error) > 0) { 
      // Get the current temperature.
      reading = getTemperature();
      Serial.print("Temperature,");
      Serial.print(reading);
      Serial.print(",");
      Serial.print("Goal,");
      Serial.print(tempGoal);

      // Calculate the error.
      previouserror = error;
      error = tempGoal - reading;
    
      // PID
      proportional = P*error;
      pidsum = proportional;
      pos = pos + pidsum;
      if (pos < -100) {
        pos = -100;
      }
  
      if (pos > 180) {
        pos = 180;
      }
      Serial.println();
    
      // Set the servo motor appropriately.
      setServo(pos);
    
      // 1Hz update rate.
      delay(1000);
    }

    // Once the goal has been reached, dwell for the specified time period.
    timer = 0;
    while (timer <= dwells[index]) { 
      // Get the current temperature.
      reading = getTemperature();
      Serial.print("Temperature,");
      Serial.print(reading);
      Serial.print(",");
      Serial.print("Goal,");
      Serial.print(tempGoal);
    
      // Calculate the error.
      previouserror = error;
      error = tempGoal - reading;
    
      // PID
      proportional = P*error;
      pidsum = proportional;
      pos = pos + pidsum;
      if (pos < -100) {
        pos = -100;
      }
  
      if (pos > 180) {
        pos = 180;
      }
      Serial.println();
    
      // Set the servo motor appropriately.
      setServo(pos);
    
      // 1Hz update rate.
      delay(1000);
      timer = timer +1;
    }
    index = index + 1;
  }
}

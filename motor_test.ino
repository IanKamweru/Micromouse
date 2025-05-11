// #include "ArduinoMotorShieldR3.h"
// #include "AxisEncoderShield3.h"

// // Create motor shield object
// ArduinoMotorShieldR3 motorShield;

// #define ENCODER_CHANNEL 2  // Second slot on the encoder shield

// void setup() {
//   Serial.begin(115200);
//   //motorShield.init();
//   initEncoderShield();  // Initialize the encoder shield

//   Serial.println("Testing Motor on Channel B");
// }

// void loop() {
//   printEncoderCounts();
//   // Half speed forward
//   //Serial.println("Motor B 50% Forward");
//   //motorShield.setM2Speed(0);
//   //delay(1000);    
  
//   //motorShield.setM2Speed(150);
//   //delay(260); // minimum pwm to start moving is 80
//   //printEncoderCounts();
//   //motorShield.setM2Speed(0);
//   delay(1000);  

//   //while(1);
// }

// // Function to print the current encoder counts
// void printEncoderCounts() {
//   long counts = getEncoderValue(ENCODER_CHANNEL);
//   Serial.print("Encoder Slot ");
//   Serial.print(ENCODER_CHANNEL);
//   Serial.print(" = ");
//   Serial.println(counts);
// }

#include "ArduinoMotorShieldR3.h"
#include "AxisEncoderShield3.h"

// constants
#define TS 5000UL                      // sample interval (5 ms in us)
#define RECORD_TIME 10000000UL        // total duration (10 s in us)
#define MAX_SAMPLES 2000              // 10s / 5ms = 2000 samples max
#define CPR 1204.0                    // encoder counts per revolution
#define PWM_VALUE 0                // step input to apply to motor

ArduinoMotorShieldR3 md; 

void setup() {
  Serial.begin(115200);                         // open serial connection
  md.init();                                    // Initialize motor shield
  initEncoderShield();                          // Initialize encoder shield
  delay(200);                                   // allow time for setup
}

void loop() {
  // initialize arrays to store data
  unsigned long times[MAX_SAMPLES];              // store timestamp for each sample (us)
  long positions[MAX_SAMPLES];                   // encoder positions
  float speeds[MAX_SAMPLES];                     // computed motor speeds (rad/s)

  int timeIdx = 0;                               // index for current sample
  bool running = false;                          // flag to start timing control

  long prevEncoder = getEncoderValue(2);         // initial encoder position
  unsigned long progStart = micros();
  unsigned long prevLoopStart = progStart;
  unsigned long curTime = progStart;

  // motor off for 1s, then step to PWM_VALUE
  md.setM2Speed(0);
  delay(1000);                 
  md.setM2Speed(PWM_VALUE);

  // timed sampling loop
  while (curTime < (progStart + RECORD_TIME)) {
    curTime = micros();                           // update current time

    // if enough time has passed since the last sample, take a new sample
    if (!running || (curTime - prevLoopStart) >= TS) {
      unsigned long elapsedTime = curTime - prevLoopStart;  // actual elapsed time in us
      prevLoopStart = curTime;

      long encoderVal = getEncoderValue(2);   // read encoder channel 1
      long delta = encoderVal - prevEncoder;  // change in position since last sample

      float velocity = 0.0;
      if (running) {
        velocity = (float(delta) * 2.0 * 3.1416f) / (CPR * (float(elapsedTime) / 1e6f)); // motor speed in rad/s
      }
      running = true;

      // store current sample data
      times[timeIdx] = curTime;
      positions[timeIdx] = encoderVal;
      speeds[timeIdx] = velocity;

      // update previous encoder value for next sample
      prevEncoder = encoderVal;
      timeIdx++;
    }
  }

  // stop motor
  md.setM2Speed(0);

  // print data over serial
  Serial.println(); 
  Serial.println("Time_us,Position,Speed_rad_per_s");
  for (int i = 0; i < timeIdx; i++) {
    Serial.print(times[i]);
    Serial.print(",");
    Serial.print(positions[i]);
    Serial.print(",");
    Serial.println(speeds[i], 6);
  }

  while (true); // halt program
}


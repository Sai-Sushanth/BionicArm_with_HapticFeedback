// EMG Envelop - BioAmp EXG Pill
// https://github.com/upsidedownlabs/BioAmp-EXG-Pill

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2021 Upside Down Labs - contact@upsidedownlabs.tech

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.    

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_SSD1306.h>
//#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH 0
#define MAX_PULSE_WIDTH 1000
#define FREQUENCY 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include <Servo.h>
#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN A0
#define INPUT_PIN1 A1
#define BUFFER_SIZE 128
int vibroA = 4;
int vibroB = 5;
int vibroC = 6;
int vibroD = 7;

int circular_buffer[BUFFER_SIZE];
int data_index, sum;
int16_t a0, a1, a2, a3;
float Vadc0, Vadc1, Vadc2, Vadc3;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;


void setup() {
  servo2.attach(3);
  servo3.attach(5);
   servo4.attach(6);
  servo5.attach(9);
   servo6.attach(10);
  
  // Serial connection begin
  Serial.begin(BAUD_RATE);
  ads.begin();  
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}
//
void haptic(int controlIn, int motorOut)
{
int pulse_wide, pulse_width, fsr_Val;

// Read values from potentiometer
fsr_Val = controlIn;

// Convert to pulse width
pulse_wide = map(fsr_Val, 0, 65536, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
 Serial.println("pulse_wide=");
 Serial.println(pulse_wide);
pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
pulse_width = map(fsr_Val, 0, 255, 0, 100);
//Serial.print("pulse_width=");
//Serial.print(pulse_width);
//Serial.println("%");
//pulse_width -= 544;
//Serial.println(pulse_wide);
//Control Motor
pwm.setPWM(motorOut, 0, pulse_width);

}
//


void loop() {

  a0 = ads.readADC_SingleEnded(0);
  a1 = ads.readADC_SingleEnded(1);
  a2 = ads.readADC_SingleEnded(2);
  a3 = ads.readADC_SingleEnded(3);
  // Calculate elapsed time
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  // Run timer
  static long timer = 0;
  timer -= interval;

  // Sample and get envelop
  if(timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    int sensor_value = analogRead(INPUT_PIN);
    int signal = EMGFilter_s1(sensor_value);
    int envelop = getEnvelop_s1(abs(signal));
    //Serial.print(signal);
    //Serial.print(",");
    Serial.println(envelop);
    //int angle=map(sensor_value, 0, 1024, 0, 180);
  
 if(envelop > 100){
     servo2.write(0);
     hand180();
      
  }
  else if(envelop < 30){
   servo2.write(180);
    hand0();
    }
  }

   // Calculate elapsed time
  static unsigned long past1 = 0;
  unsigned long present1 = micros();
  unsigned long interval1 = present1 - past1;
  past1 = present1;

  // Run timer
  static long timer1 = 0;
  timer1 -= interval1;

  // Sample and get envelop
  if(timer1 < 0) {
    timer1 += 1000000 / SAMPLE_RATE;
    int sensor_value1 = analogRead(INPUT_PIN1);
    int signal1 = EMGFilter_s1(sensor_value1);
    int envelop1 = getEnvelop_s1(abs(signal1));
    //Serial.print(signal);
    //Serial.print(",");
    Serial.println(envelop1);
    //int angle=map(sensor_value, 0, 1024, 0, 180);
  
 if(envelop1 > 100){
   servo3.write(0);
   hand180();
    
  }
  else if(envelop1 < 30){
    
     servo3.write(180);
     hand0();
   
    }
  }
  //Control vibro A
haptic(a0, vibroA);

//Control vibro B
haptic(a1, vibroB);

//Control vibro C
haptic(a2, vibroC);

//Control vibro D
haptic(a3, vibroD);
}

//hands

void hand180(){
  servo4.write(180);
    servo5.write(180);
  
  }
  
  void hand0(){
    
     servo4.write(0);
    servo5.write(0);
    
    }

// Envelop detection algorithm
int getEnvelop_s1(int abs_emg){
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum/BUFFER_SIZE) * 2;
}

int getEnvelop_s2(int abs_emg){
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum/BUFFER_SIZE) * 2;
}

int getEnvelop_s3(int abs_emg){
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum/BUFFER_SIZE) * 2;
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference: 
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EMGFilter_s1(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
// sensor2
float EMGFilter_s2(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
// senor3
float EMGFilter_s3(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13
// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 180 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360 // maximum distance to be measured (unit: mm)
#define _DUTY_MIN 0 // servo full clockwise position (0 degree)
#define _DUTY_MAX 180 // servo full counterclockwise position (180 degree)
#define _EMA_ALPHA 0.6
#define _IMPOSSIBLE_VALUE -999

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float dist_ema;
float emaPrev = _IMPOSSIBLE_VALUE - 1;
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);
  myservo.attach(PIN_SERVO); 
  
// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  
// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
  digitalWrite(PIN_LED, HIGH);
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  dist_ema = EvaluateEMA(dist_raw);

  Serial.print("Min: 100, raw: ");
  Serial.print(dist_raw);
  Serial.print(", ema: ");
  Serial.print(dist_ema);
  Serial.print(", servo: ");
  Serial.print(myservo.read());
  Serial.println(", Max: 360");

  if(dist_ema < _DIST_MIN) {
     myservo.write(_DUTY_MIN);
     digitalWrite(PIN_LED, HIGH);
  }
  else if(dist_ema < _DIST_MAX){
     float t = (dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN);
     float amount = Hee(_DUTY_MIN, _DUTY_MAX, t);
     myservo.write(amount);
     digitalWrite(PIN_LED, LOW);
  }
  else {
    myservo.write(_DUTY_MAX);
    digitalWrite(PIN_LED, HIGH);
  }
}

float Hee(float x, float y, float t) {
  return x * (1 - t) + y * t; 
}
float EvaluateEMA(float value) {
  float result;
  if (emaPrev > _IMPOSSIBLE_VALUE) {
    result = round(_EMA_ALPHA * value) + round((1 - _EMA_ALPHA) * emaPrev);
  }
  else {
    result = value;
  }
  emaPrev = result;
  return result;
}
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  //if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.

  //if (reading == 0.0) reading = dist_prev;
  //else dist_prev = reading;
  
  return reading;
}

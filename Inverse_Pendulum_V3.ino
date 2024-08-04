#include <CytronMotorDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_BNO055.h>

//initialise motor and sensor
CytronMD motor1(PWM_DIR, 3, 4);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
const byte sensorPin = 2;  // IR sensor connected to pin 2
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long interruptInterval = 0;
const unsigned long debounceDelay = 1000;  // Debounce time in microseconds
const unsigned long minValidInterval = 5000; // Minimum valid interval to filter out noise

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorISR, RISING);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

}

void loop() {
  //Angular velocity calculation
  noInterrupts();
  unsigned long interval = interruptInterval;
  interrupts();
  float angV = 0;
  if (interval >= minValidInterval) {
    angV = (2 * PI) / (interval / 1000000.0); // in radians per second
    Serial.print("Angular Velocity: ");
    Serial.print(angV);
    Serial.print(" rad/s ");
  }
  
   /* Get a new sensor event */
  int16_t s = 0; 
  sensors_event_t event; 
  bno.getEvent(&event);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float z = euler.z();
  if(abs(z) <15.){s = -80*z;}
  else{s = 0;}
  // if(abs(s)>90)
  // {
  //   if(z>0){
  //       s=-90;
  //   }
  //   if(z<0){
  //       s=90;
  //   }
  // }
  if(angV>250 || abs(s)>255)
  {
    if(s>0){s = 50;}
    else{s = -50;}
  }
  motor1.setSpeed(s);
  Serial.print("M: ");
  Serial.print(s);
  Serial.print(" Z: ");
  Serial.print(z);
  Serial.println();
  delay(50);
}
void sensorISR() {
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime > debounceDelay) {
    unsigned long interval = currentTime - lastInterruptTime;
    if (interval >= minValidInterval) {
      interruptInterval = interval;
    }
    lastInterruptTime = currentTime;
  }
}

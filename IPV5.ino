#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;
//
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
//
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  DEBUG_SERIAL.println("Orientation Sensor Raw Data Test"); DEBUG_SERIAL.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    DEBUG_SERIAL.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  //
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM);
  dxl.torqueOn(DXL_ID);
}
float curr_y, prev_y = 0;
int kp = -440;
int flag = 1;
void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  DEBUG_SERIAL.print(" Y: ");
  DEBUG_SERIAL.println(euler.y());
  DEBUG_SERIAL.print("\t\t");
  delay(10);
  float s = 0.0;
  curr_y = euler.y();
  if(flag = 0){
    if(abs(prev_y) > abs(curr_y)){
      s = (-573 * euler.y())-130;
      if(abs(euler.y()<0.35)){
        s = 0;
        flag = 1;
      }
    }
  }
  if(abs(euler.y())>= 1.5){
      flag = 0;
      s = kp*euler.y();
    }
  if(s>885){s=885;}
  if(s<-885){s=-885;}
  s = s/8.85;
  dxl.setGoalPWM(DXL_ID, s, UNIT_PERCENT);
  delay(10);
  DEBUG_SERIAL.print("M: ");
  DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID, UNIT_PERCENT));
  prev_y = curr_y;
  
  // put your main code here, to run repeatedly:

}

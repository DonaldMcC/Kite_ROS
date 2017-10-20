/*This now seems to work - provided plugged in everything correct way - so slcd in pins 7 and 8 and sliders on the analogue port nearest the I2C port on the side A4 and A5 
 still to test battery life with lcd and currently logging every half a second but this may need to be reduced ie more logging 
 NOw being scaled back to remove the datalogger and see if we can get the 3 resistors and perhaps the lcd display - but may well take that out again soon too

 Newkitev2 - will add ros librarie and see if we can publish data to a ROS node
 */


#include <Wire.h>
#include <math.h>
#include <ros.h>

// include the library code:
#include <SerialLCD.h>
#include <SoftwareSerial.h> //this is a must

// initialize the library
SerialLCD slcd(8,9);//this is a must, assign soft serial pins


//initiate ROS
ros::Nodehandle nh;


ros::Publisher chatter("chatter", %str_msg)

void callback ( const std_msgs::String& msg{
str_msg.data=msg.data;
chatter.publish( &str_msg );
}



#define ACCELE_SCALE 2  // accelerometer full-scale, should be 2, 4, or 8

/* LSM303 Address definitions */
#define LSM303_MAG  0x1E  // assuming SA0 grounded
#define LSM303_ACC  0x18  // assuming SA0 grounded

#define X 0
#define Y 1
#define Z 2

/* LSM303 Register definitions */
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define HP_FILTER_RESET_A 0x25
#define REFERENCE_A 0x26
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define INT1_CFG_A 0x30
#define INT1_SOURCE_A 0x31
#define INT1_THS_A 0x32
#define INT1_DURATION_A 0x33
#define CRA_REG_M 0x00
#define CRB_REG_M 0x01//refer to the Table 58 of the datasheet of LSM303DLM
#define MAG_SCALE_1_3 0x20//full-scale is +/-1.3Gauss
#define MAG_SCALE_1_9 0x40//+/-1.9Gauss
#define MAG_SCALE_2_5 0x60//+/-2.5Gauss
#define MAG_SCALE_4_0 0x80//+/-4.0Gauss
#define MAG_SCALE_4_7 0xa0//+/-4.7Gauss
#define MAG_SCALE_5_6 0xc0//+/-5.6Gauss
#define MAG_SCALE_8_1 0xe0//+/-8.1Gauss
#define MR_REG_M 0x02
#define OUT_X_H_M 0x03
#define OUT_X_L_M 0x04
#define OUT_Y_H_M 0x07
#define OUT_Y_L_M 0x08
#define OUT_Z_H_M 0x05
#define OUT_Z_L_M 0x06
#define SR_REG_M 0x09
#define IRA_REG_M 0x0A
#define IRB_REG_M 0x0B
#define IRC_REG_M 0x0C

/* Global variables */
int accel[3];  // we'll store the raw acceleration values here
int mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 10;

void setup()
{
  // so looks like slcd clashes with the SD card
  slcd.begin(); 
  Wire.begin();
  
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only

 //initialise ros
 nh.initNode();
 nh.advertise(chatter);
  }
  
  initLSM303(ACCELE_SCALE);  // Initialize the LSM303, using a SCALE full-scale range

}


void loop()
{

getLSM303_accel(accel);  // get the acceleration values and store them in the accel array
  while(!(LSM303_read(SR_REG_M) & 0x01))
    ;  // wait for the magnetometer readings to be ready
  getLSM303_mag(mag);  // get the magnetometer values, store them in mag
  //printValues(mag, accel);  // print the raw accel and mag values, good debugging
  Serial.println("Acceleration of X,Y,Z is");
  for (int i=0; i<3; i++)
  {
    realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;  // calculate real acceleration values, in units of g
	Serial.print(realAccel[i]);
	Serial.println("g");
  }
  /* print both the level, and tilt-compensated headings below to compare */
  /*  Serial.println("The clockwise angle between the magnetic north and x-axis: ");
  Serial.print(getHeading(mag), 3); // this only works if the sensor is level
  */
  
  slcd.clear();
  slcd.setCursor(0, 0);
  int yval = mag[Y]+800;
  slcd.print("Y"); 
  slcd.print(yval,DEC);
  slcd.setCursor(6, 0);
  slcd.print("H"); 
  int head = getTiltHeading(mag, realAccel);
  slcd.print(head,DEC);
 
  slcd.setCursor(0, 1);
  slcd.print("A"); 
  slcd.print(analogRead(0),DEC);
  
  slcd.setCursor(5, 1);
  slcd.print("B"); 
  slcd.print(analogRead(2),DEC);
    
  slcd.setCursor(10, 1);
  slcd.print("C"); 
  slcd.print(analogRead(3),DEC);
  
  Serial.print(getTiltHeading(mag, realAccel), 3);  // see how awesome tilt compensation is?!
  /*Serial.println(" degrees");*/

  nh.spinOnce();
  delay(500);

}


void initLSM303(int fs)
{
  LSM303_write(0x27, CTRL_REG1_A);  // 0x27 = normal power mode, all accel axes on
  if ((fs==8)||(fs==4))
    LSM303_write((0x00 | (fs-fs/2-1)<<4), CTRL_REG4_A);  // set full-scale
  else
    LSM303_write(0x00, CTRL_REG4_A);
  LSM303_write(0x14, CRA_REG_M);  // 0x14 = mag 30Hz output rate
  LSM303_write(MAG_SCALE_1_3, CRB_REG_M); //magnetic scale = +/-1.3Gauss
  LSM303_write(0x00, MR_REG_M);  // 0x00 = continouous conversion mode
}


float getHeading(int * magValue)
{
  // see section 1.2 in app note AN3192
  float heading = 180*atan2(magValue[Y], magValue[X])/PI;  // assume pitch, roll are 0
  
  if (heading <0)
    heading += 360;
  
  return heading;
}


float getTiltHeading(int * magValue, float * accelValue)
{
 // see appendix A in app note AN3192 
  float pitch = asin(-accelValue[X]);
  float roll = asin(accelValue[Y]/cos(pitch));
  
  float xh = magValue[X] * cos(pitch) + magValue[Z] * sin(pitch);
  float yh = magValue[X] * sin(roll) * sin(pitch) + magValue[Y] * cos(roll) - magValue[Z] * sin(roll) * cos(pitch);
  float zh = -magValue[X] * cos(roll) * sin(pitch) + magValue[Y] * sin(roll) + magValue[Z] * cos(roll) * cos(pitch);
  
  
  float heading = 180 * atan2(yh, xh)/PI;
  
  if (yh >= 0)    return heading;  
  else    return (360 + heading);
}

void getLSM303_mag(int * rawValues)
{
  Wire.beginTransmission(LSM303_MAG);
  Wire.write(OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM303_MAG, 6);
  for (int i=0; i<3; i++)
    rawValues[i] = (Wire.read() << 8) | Wire.read();
  int temp;
  temp = rawValues[Y];
  rawValues[Y] = rawValues[Z];
  rawValues[Z] = temp;  
}

void getLSM303_accel(int * rawValues)
{
  rawValues[Z] = ((int)LSM303_read(OUT_X_L_A) << 8) | (LSM303_read(OUT_X_H_A));
  rawValues[X] = ((int)LSM303_read(OUT_Y_L_A) << 8) | (LSM303_read(OUT_Y_H_A));
  rawValues[Y] = ((int)LSM303_read(OUT_Z_L_A) << 8) | (LSM303_read(OUT_Z_H_A));
  // had to swap those to right the data with the proper axis
}

byte LSM303_read(byte address)
{
  byte temp;
  
  if (address >= 0x20)
    Wire.beginTransmission(LSM303_ACC);
  else
    Wire.beginTransmission(LSM303_MAG);
    
  Wire.write(address);
  
  if (address >= 0x20)
    Wire.requestFrom(LSM303_ACC, 1);
  else
    Wire.requestFrom(LSM303_MAG, 1);
  while(!Wire.available())
    ;
  temp = Wire.read();
  Wire.endTransmission();
  
  return temp;
}

void LSM303_write(byte data, byte address)
{
  if (address >= 0x20)
    Wire.beginTransmission(LSM303_ACC);
  else
    Wire.beginTransmission(LSM303_MAG);
    
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}




//. Motor driver shield- 2012 Copyright (c) Seeed Technology Inc.
// 
//  Original Author: Jimbo.we
//  Contribution: LG
//  Now modified to include ROS messaging and include a potentiometer reading as well
//  
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

std_msgs::Int16 msg;
// Declaring String variable
std_msgs::Int16 int_msg; 

ros::Publisher kiteangle("kiteangle", &msg);

// Defining callback as unsigned and no measurement on motors we only really need to send
// left right or stop and logically 0 should be stop - possibly there will eventually be a range
// of speeds so we will go with first digit being direction and final two being speed 
// with 100-199 being left and 200-299 being right
void callback (const std_msgs::Int16&int_msg)
{ 
  if (int_msg.data == 0)
    stop();
  else if (int_msg.data < 200)
      left();
    else
      right();

} 


// Defining Subscriber 
ros::Subscriber<std_msgs::Int16> 
sub("motormsg", callback); 


int pinI1=8;//define I1 interface
int pinI2=11;//define I2 interface 
int speedpinA=9;//enable motor A
int pinI3=12;//define I3 interface 
int pinI4=13;//define I4 interface 
int speedpinB=10;//enable motor B
int spead =255;//define the spead of motor as fast as pooss

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup()
{
  nh.initNode();
  nh.advertise(kiteangle);
  nh.subscribe(sub);
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);
}


void forward()
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
}

void backward()//
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI1,LOW);
}

void left()//
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI1,LOW);
}

void right()//
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
     digitalWrite(pinI1,HIGH);
}

void stop()//
{
     digitalWrite(speedpinA,LOW);// Unenable the pin, to stop the motor. this should be done to avid damaging the motor. 
     digitalWrite(speedpinB,LOW);
     delay(1000);
 
}

void loop()
{
  //left();
  //delay(100);
  //stop();
  //right();
  //delay(100);
  //stop();
 // delay(2000);
  //forward();
  //delay(100);
  //stop();
  //backward();
  //delay(100); 
  //stop(); 
  sensorValue = analogRead(sensorPin);
  msg.data = sensorValue;
  kiteangle.publish(&msg);
  nh.spinOnce();
  delay(100);
}

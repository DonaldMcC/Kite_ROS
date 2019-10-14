/*
  SerialLCD.h - Serial LCD driver Library
  2010 Copyright (c) Seeed Technology Inc.  All right reserved.
 
  Original Author: Jimbo.We
  Contribution: Visweswara R 
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "WProgram.h"
#include <NewSoftSerial.h>
#include "SerialLCD.h"

SerialLCD::SerialLCD(uint8_t rx, uint8_t tx):NewSoftSerial(rx,tx)
{

}

/********** High level commands, for the user! **********/

// Initialize the Serial LCD Driver. SerialLCD Module initiates the communication.
void SerialLCD::begin()
{
    NewSoftSerial::begin(9600);
    while(1)
    {
        if ( NewSoftSerial::available() > 0 &&  NewSoftSerial::read()==SLCD_INIT )
        {
            NewSoftSerial::print(SLCD_INIT_ACK,BYTE);
            break;
        }
    }
    while(1)
    {
        if (NewSoftSerial::available() > 0 &&NewSoftSerial::read()==SLCD_INIT_DONE)
            break;
    }
}

// Clear the display
void SerialLCD::clear()
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);   
    NewSoftSerial::print(SLCD_CLEAR_DISPLAY,BYTE);   
}

// Return to home(top-left corner of LCD)
void SerialLCD::home()
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_RETURN_HOME,BYTE);    
}

// Set Cursor to (Column,Row) Position
void SerialLCD::setCursor(uint8_t column, uint8_t row)
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE); 
    NewSoftSerial::print(SLCD_CURSOR_HEADER,BYTE); //cursor header command
    NewSoftSerial::print(column,BYTE);
    NewSoftSerial::print(row,BYTE);
}

// Switch the display off without clearing RAM
void SerialLCD::noDisplay() 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_DISPLAY_OFF,BYTE);    
}

// Switch the display on
void SerialLCD::display() 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_DISPLAY_ON,BYTE);    
}

// Switch the underline cursor off
void SerialLCD::noCursor() 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_CURSOR_OFF,BYTE);     
}

// Switch the underline cursor on
void SerialLCD::cursor() 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_CURSOR_ON,BYTE);     
}

// Switch off the blinking cursor
void SerialLCD::noBlink() 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_BLINK_OFF,BYTE);     
}

// Switch on the blinking cursor
void SerialLCD::blink() 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_BLINK_ON,BYTE);     
}

// Scroll the display left without changing the RAM
void SerialLCD::scrollDisplayLeft(void) 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_SCROLL_LEFT,BYTE);
}

// Scroll the display right without changing the RAM
void SerialLCD::scrollDisplayRight(void) 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_SCROLL_RIGHT,BYTE);
}

// Set the text flow "Left to Right"
void SerialLCD::leftToRight(void) 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_LEFT_TO_RIGHT,BYTE);
}

// Set the text flow "Right to Left"
void SerialLCD::rightToLeft(void) 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_RIGHT_TO_LEFT,BYTE);
}

// This will 'right justify' text from the cursor
void SerialLCD::autoscroll(void) 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_AUTO_SCROLL,BYTE);
}

// This will 'left justify' text from the cursor
void SerialLCD::noAutoscroll(void) 
{
    NewSoftSerial::print(SLCD_CONTROL_HEADER,BYTE);
    NewSoftSerial::print(SLCD_NO_AUTO_SCROLL,BYTE);
}

// Print Commands

void SerialLCD::print(uint8_t b)
{
    NewSoftSerial::print(SLCD_CHAR_HEADER,BYTE);
    NewSoftSerial::print(b);
}
void SerialLCD::print(const char b[])
{
    NewSoftSerial::print(SLCD_CHAR_HEADER,BYTE);
    NewSoftSerial::print(b);
}

void SerialLCD::print(unsigned long n, uint8_t base)
{
    unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
    unsigned long i = 0;

    if (base == 0) print(n);

    else if(base!=0)
    {
        if (n == 0) {
            print('0');
            return;
        }

        while (n > 0) {
            buf[i++] = n % base;
            n /= base;
        }

        for (; i > 0; i--)
            print((char) (buf[i - 1] < 10 ?
                          '0' + buf[i - 1] :
                          'A' + buf[i - 1] - 10));
    }
}

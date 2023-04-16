// GR-KURUMI Sketch Template Version: V2.01
// Copyright (C) 2017 I.Nagai(Okayama Univ.) All rights reserved.

// 2017.02.22-24 ver.1.0
// 2017.02.28 ver.1.1 baudrate 800k bps -> 460k bps
// 2017.06.20 ver.2.0 Sensor : ADXL355

#include <Arduino.h>
#include <Wire.h>

#include <string.h>

//ADXL355 (accelerometer)
#define AM_I2C_ADR         0x1D //High->0x53, Low->0x1D
#define AM_DEVID_AD        0x00
#define AM_STATUS          0x04
#define AM_FIFO_ENTRIES    0x05
#define AM_XDATA3          0x08
#define AM_XDATA2          0x09
#define AM_XDATA1          0x0A
#define AM_YDATA3          0x0B
#define AM_YDATA2          0x0C
#define AM_YDATA1          0x0D
#define AM_ZDATA3          0x0E
#define AM_ZDATA2          0x0F
#define AM_ZDATA1          0x10
#define AM_FIFO_DATA       0x11
#define AM_FILTER          0x28
#define AM_RANGE           0x2C
#define AM_POWER_CTL       0x2D
#define AM_RESET           0x2F

// pin 22,23,24 are assigned to RGB LEDs
#define LED_R 22        // LOW active
#define LED_G 23        // LOW active
#define LED_B 24        // LOW active

unsigned long t0, t, delta_t;

//Send 1 byte to I2C
void I2C_Send1(unsigned char adr, unsigned char b0)
{
        Wire.beginTransmission(adr);
        Wire.write(b0);
        Wire.endTransmission();    
}

//Send 2 bytes to I2C
void I2C_Send2(unsigned char adr, unsigned char b0, unsigned char b1)
{
        Wire.beginTransmission(adr);
        Wire.write(b0);
        Wire.write(b1);
        Wire.endTransmission();    
}

//Receive n bytes from I2C
void I2C_Recv(int adr, unsigned char *b, int n)
{
    Wire.requestFrom(adr,n);
    while(Wire.available()<n);
    for(int i=0;i<n;i++){
        b[i]= Wire.read();
    }
}

//Receive n bytes from I2C
void I2C_Recv_Multi(int adr, int sub, unsigned char *b, int n)
{
        Wire.beginTransmission(adr);
        Wire.write(sub);
        Wire.endTransmission(false);
        Wire.requestFrom(adr,n,false);
    while(Wire.available()<n);
    for(int i=0; i<n; i++){
        b[i]= Wire.read();
    }
        Wire.endTransmission(true);
}

//Receive n bytes from I2C (ADXL355)
void I2C_Recv_Multi2(int adr, int sub, unsigned char *b, int n)
{
        Wire.beginTransmission(adr);
        Wire.write(sub);
        Wire.endTransmission();
        Wire.requestFrom(adr,n);
    while(Wire.available()<n);
    for(int i=0; i<n; i++){
        b[i]= Wire.read();
    }
        Wire.endTransmission();
}

void setup(void)
{
        // initialize the LED pin
        pinMode(LED_R, OUTPUT);
        pinMode(LED_G, OUTPUT);
        pinMode(LED_B, OUTPUT);
        // LEDs off
        digitalWrite(LED_R, 1);
        digitalWrite(LED_G, 1);
        digitalWrite(LED_B, 1);

        delay(50);

        Serial.begin(460000);   //baudrate [bps]
        //Serial.begin(38400);  //baudrate [bps]
        //Serial.write("Program start\r\n");

        Wire.begin();   //Initialize I2C

        //unsigned char b[16];
        //char s[256];
        //I2C_Recv_Multi2(AM_I2C_ADR,AM_DEVID_AD,b,1);
        //sprintf(s,"DEVID_AD=0x%X\r\n",b[0]);
        //Serial.write(s);

        //Initialize ADXL355
        I2C_Send2(AM_I2C_ADR, AM_RESET, 0x52);
        delay(50);
        I2C_Send2(AM_I2C_ADR, AM_POWER_CTL, 0x00);      //00h: measure mode
        //I2C_Send2(AM_I2C_ADR, AM_FILTER,    0x0A);    //0Ah: no-highpass, ODR=3.906Hz
        //I2C_Send2(AM_I2C_ADR, AM_FILTER,    0x07);    //07h: no-highpass, ODR=31.25Hz
        I2C_Send2(AM_I2C_ADR, AM_FILTER,    0x05);      //05h: no-highpass, ODR=125Hz
        I2C_Send2(AM_I2C_ADR, AM_RANGE,     0x01);      //01h: kousoku-mode, +/-2g

    t0= micros();
}

int s_flag;
int t_flag;
int x[3];
int fifo_index;

// the loop routine runs over and over again forever:
void loop(void)
{
        int c;
        unsigned char b[16], sub, val;
        char s[256];
        
        //Process received command
        if(Serial.available()){
                c= Serial.read();
                switch(c){
                        case 's':       //Start output
                                s_flag= 1;
                                break;
                        case 'c':       //Stop output
                                s_flag= 0;
                                break;
                        case 'w':       //write register
                                while(Serial.available()<2);
                                sub= Serial.read();
                                val= Serial.read();
                                I2C_Send2(AM_I2C_ADR, sub, val);
                                break;
                        case 't':       //Toggle text output
                                t_flag= 1 - t_flag;
                                break;
                }
        }

        t= micros();

        //Reading sensor data
        I2C_Recv_Multi2(AM_I2C_ADR,AM_FIFO_ENTRIES,b,1);
        if(b[0]){
                I2C_Recv_Multi2(AM_I2C_ADR,AM_FIFO_DATA,b,3);
                if(b[2]&0x01) fifo_index= 0; else fifo_index=(fifo_index+1)%3;
                x[fifo_index]= (((int)b[0])<<8) | ((int)b[1]);  //only upper 16-bit
                if(fifo_index==2){
                        if(t_flag){
                                delta_t= t-t0;
                                t0= t;
                                sprintf(s,"%ld:",delta_t);
                                Serial.write(s);
                                sprintf(s,"X%7d Y%7d Z%7d\r\n",x[0],x[1],x[2]);
                                Serial.write(s);
                        }
                        if(s_flag){
                                Serial.write((unsigned char *)x,6);
                        }
                }
        }

}
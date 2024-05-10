#include <Simpletimer.h>


// ESP32 and TOF10120 Laser RangeFinder Distance Sensor
//download libraries
//https://www.electroniclinic.com/arduino-libraries-download-and-projects-they-are-used-in-project-codes/
 
#include<stdlib.h>

#include <Wire.h>
 
 
Simpletimer timer1; // for TOF10120 Sensor
Simpletimer timer2; // use to check if the distance value remains the same for 4 seconds
int secs = 0; 
float setSensorValue = 0; 
// for TOF10120 Laser Rangefinder Sensor
 
unsigned char ok_flag;
unsigned char fail_flag;
 
unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag=0;
 
int x_mm; // distance in millimeters
float f_value; 
 

 
void setup(){
 
  Serial.begin(9600);
  Wire.begin();
 
 
  timer1.setInterval(200L, TOF10120);
  timer2.setInterval(1000L, SensorTime);
 
}
void loop(){
 
  timer1.run(); // Initiates SimpleTimer
  timer2.run();
}
 
 
 
void TOF10120()
{
  
    for(int tof = 0; tof <=  9; tof++)
    {
      x_mm = x_mm + ReadDistance();
     
    }
       f_value = x_mm / 10;
       //Serial.print(f_value);
       //Serial.println(" mm");
       x_mm = 0;
 
     if ( (f_value < setSensorValue) || ( f_value > setSensorValue))
     {
      if ( secs == 4 )
      {
        setSensorValue = f_value; 
      }
 
     }
 
     if ( setSensorValue == f_value )
     {
      secs = 0;
     }
    
 Serial.print("Distance in mm: "); 
 Serial.println(setSensorValue);
}
 
 
int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}
 
 
 
void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82); // transmit to device #82 (0x52), you can also find this address using the i2c_scanner code, which is available on electroniclinic.com
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));      // sets distance data address (addr)
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(82, cnt);    // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();  // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
  }
}
 
int ReadDistance(){
    SensorRead(0x00,i2c_rx_buf,2);
    lenth_val=i2c_rx_buf[0];
    lenth_val=lenth_val<<8;
    lenth_val|=i2c_rx_buf[1];
    delay(100); 
    return lenth_val;
}
 
void SensorTime()
{
  secs = secs + 1; 
  if ( secs == 5 )
  {
    secs = 0; 
  }
 // Serial.println(secs);
  
}
#include <Wire.h>

unsigned char ok_flag;
unsigned char fail_flag;

unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag = 0;


void setup() {
  Wire.begin(); 
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  int x = ReadDistance();
  Serial.print(x);
  Serial.println(" mm");
}

int serial_putc(char c, struct __file *) {
  Serial.write(c); // Write the character directly to Serial
  return c;
}

void SensorRead(unsigned char addr, unsigned char* datbuf, unsigned char cnt) {
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82); // transmit to device #82 (0x52)
  Wire.write(byte(addr));     // sets distance data address (addr)
  Wire.endTransmission();     // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(82, cnt);  // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();      // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read();      // receive low byte as lower 8 bits
  }
}

int ReadDistance(){
  SensorRead(0x00, i2c_rx_buf, 2);
  lenth_val = i2c_rx_buf[0];
  lenth_val = lenth_val << 8;
  lenth_val |= i2c_rx_buf[1];
  delay(300); 
  return lenth_val;
}

#include <PS4BT.h>

#include <usbhub.h>
#include <Wire.h>  
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

byte x=127, y=127, w=127;
byte psbt[3];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  Wire.begin(8);                // join I2C bus with address #8
  Wire.onRequest(requestEvent); // register event
}

void loop() {
  // put your main code here, to run repeatedly:
  Usb.Task();

  if (PS4.connected()) {
    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117 ){
      x=PS4.getAnalogHat(LeftHatX);
      y=PS4.getAnalogHat(LeftHatY);
    }
    else{
      x=127;
      y=127;
    }

    w = map(PS4.getAnalogButton(L2)-PS4.getAnalogButton(R2),-255,255,0,255);
    if(w<135&&w>120){
      w=127;
    }
  }

  else{
    x=127;
    y=127;
    w=127;
    }
  Serial.print("X :");   // print the character
  Serial.print(x);
  Serial.print("     Y : ");
  Serial.print(y);
  Serial.print("     W : ");
  Serial.println(w);
 
  psbt[0] = x;
  psbt[1] = y;
  psbt[2] = w;
 
}

void requestEvent() {
    Wire.write(psbt,3);
}

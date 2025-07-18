#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <Arduino.h>
RF24 radio(2, 4); // CE, CSN

const byte address[6] = "NODE1";

struct message
{
  uint16_t pot1;
  bool button1;
};

message Data;
Servo ESC;
int write1;

void setup()
{
  Serial.begin(115200);
  if (!radio.begin())
  {
    Serial.println("NRF24L01 not responding");
    while (1)
      ;
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(0, address);
  radio.startListening();
  ESC.attach(17, 1000, 2000);
}

void loop()
{
  if (radio.available())
  {
    radio.read(&Data, sizeof(Data));
    Serial.print("Received: ");

    write1 = map(Data.pot1, 0, 1023, 0, 180);
    Serial.println(write1);

    ESC.write(write1);
  }
}

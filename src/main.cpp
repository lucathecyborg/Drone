#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>

RF24 radio(2, 4); // CE, CSN

const byte address[6] = "NODE1";

struct joystick
{
  uint16_t x;
  uint16_t y;
  bool button;
};

struct message
{
  uint16_t pot1;
  joystick joystickL;
  joystick joystickR;
  bool button1;
};

message Data;

// PWM config for motors
#define TOPL_PIN 14
#define TOPR_PIN 27
#define BOTTOML_PIN 26
#define BOTTOMR_PIN 25

#define TOPL_CHANNEL 0
#define TOPR_CHANNEL 1
#define BOTTOML_CHANNEL 2
#define BOTTOMR_CHANNEL 3

#define PWM_FREQ 50       // 50Hz for ESC
#define PWM_RESOLUTION 16 // 16-bit resolution

int base_motor_speed;
uint16_t power = 69;

int topL_speed;
int topR_speed;
int bottomL_speed;
int bottomR_speed;



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
  radio.enableAckPayload();
  radio.startListening();

  // Setup PWM for topL ESC
  ledcSetup(TOPL_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(TOPL_PIN, TOPL_CHANNEL);
  ledcSetup(TOPR_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(TOPR_PIN, TOPR_CHANNEL);

  // Send 1000 µs to arm ESC
  int armDuty = map(1000, 1000, 2000, 3276, 6553);
  ledcWrite(TOPL_CHANNEL, armDuty);
  ledcWrite(TOPR_CHANNEL, armDuty);
  Serial.println("Sending arming signal to ESC...");
  delay(3000); // Wait 3s for ESC to arm
  Serial.println("ESC should be armed now.");

  // Wait for NRF to receive
  while (!radio.available())
  {
    // Optionally send battery info
    // sendBattery();
  }
}

unsigned long lastPrint = 0;

void loop()
{
  if (radio.available())
  {
    radio.read(&Data, sizeof(Data));
    unsigned long now = millis();

    base_motor_speed = map(Data.pot1, 0, 1023, 0, 150);

    int x_center = 510;
    int y_center = 515;

    int x_adjust = map(Data.joystickL.x, x_center - 512, x_center + 511, -30, 30);
    int y_adjust = map(Data.joystickL.y, y_center - 512, y_center + 511, -30, 30);

    int max_motor_speed = (x_adjust != 0 || y_adjust != 0) ? 180 : 150;

    if (base_motor_speed > 30)
    {
      topL_speed = constrain(base_motor_speed + y_adjust - x_adjust, 0, max_motor_speed);
      topR_speed = constrain(base_motor_speed + y_adjust + x_adjust, 0, max_motor_speed);
      bottomL_speed = constrain(base_motor_speed - y_adjust - x_adjust, 0, max_motor_speed);
      bottomR_speed = constrain(base_motor_speed - y_adjust + x_adjust, 0, max_motor_speed);
    }
    else
    {
      topL_speed = base_motor_speed;
      topR_speed = base_motor_speed;
      bottomL_speed = base_motor_speed;
      bottomR_speed = base_motor_speed;
    }

    // Only topL motor controlled here via ledcWrite
    int pwmDuty = map(topL_speed, 0, 180, 3276, 6553);
    ledcWrite(TOPL_CHANNEL, pwmDuty);
    pwmDuty = map(topR_speed, 0, 180, 3276, 6553);
    ledcWrite(TOPR_CHANNEL, pwmDuty);

    if (now - lastPrint > 200)
    {
      lastPrint = now;
      Serial.println("LT: " + String(topL_speed) + " RT: " + String(topR_speed) +
                     " LB: " + String(bottomL_speed) + " RB: " + String(bottomR_speed));
    }
    radio.writeAckPayload(1, &power, sizeof(power));
  }

}

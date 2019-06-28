#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_SHIELD

/* Include 1Sheeld library. */
#include <OneSheeld.h>
#include <Dagu4Motor.h>

Dagu4Motor m1 (9,8,A0,7,6);

void setup() 
{
  /* Start communication. */
  OneSheeld.begin();
  m1.begin();
  //m1.setSpeed(150);
  //m1.setMotorDirection(true);
  m1.stopMotors();
}

void loop() 
{
  /* Always check the status of gamepad buttons. */
  if (GamePad.isUpPressed())
  {
    m1.setMotorDirection(true);
    m1.setSpeed(150);

  }
 if(GamePad.isDownPressed())
  {
    m1.setMotorDirection(false);
    m1.setSpeed(150);

  }
  if(GamePad.isLeftPressed())
  {

  }
  if(GamePad.isRightPressed())
  {

  }
  if (GamePad.isRedPressed())
  {
    m1.stopMotors();
  }
}


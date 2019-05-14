/*

Gamepad Shield Example

This example shows an application on 1Sheeld's gamepad shield.

By using this example, you can light up some LEDs placed on
different Arduino pins using the gamepad in our app.

OPTIONAL:
To reduce the library compiled size and limit its memory usage, you
can specify which shields you want to include in your sketch by
defining CUSTOM_SETTINGS and the shields respective INCLUDE_ define. 

*/

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


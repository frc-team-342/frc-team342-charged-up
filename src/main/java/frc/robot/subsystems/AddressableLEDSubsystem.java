// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDConstants.*;

public class AddressableLEDSubsystem extends SubsystemBase {
  /** Creates a new AddressableLEDSubsystem. */

  public enum ColorType {
    YELLOW,
    PURPLE;
  }

  private final AddressableLED LED;
  private final AddressableLEDBuffer LEDBuffer;

  public AddressableLEDSubsystem() {
    LED = new AddressableLED(PWM_PORT);
    LEDBuffer = new AddressableLEDBuffer(LENGTH);
    LED.setLength(LEDBuffer.getLength());
    LED.setData(LEDBuffer);
    LED.start();
  }

  /**
   * This method sets all the LED groups (Human Player & Driver) to off
   */
  public void LEDOff() {
    //Sets each LED to off
    for(int i = 0; i < LEDBuffer.getLength(); i++)
    {
      LEDBuffer.setHSV(i, 0, 0, 0);
    }
    LED.setData(LEDBuffer);
  }

  /**
   * This method sets the Human Player LED group to the Yellow Color or Purple Color
   */
  public void driverColorMethod(ColorType colortype) {
    //If the colortype requested is yellow, then it will set the Human Player group color to yellow
    //If the colortype requested is purple, then it will set the Human Player group color to purple
    if(ColorType.YELLOW == colortype)
    {
      //Sets each LED to Yellow
      for(int i = 0; i < DRIVER_START_RANGE; i++)
      {
        LEDBuffer.setHSV(i, YELLOW_H, YELLOW_S, YELLOW_V);
      }
      LED.setData(LEDBuffer);
    }
    else if(ColorType.PURPLE == colortype)
    {
      //Sets each LED to Purple
      for(int i = 0; i < DRIVER_START_RANGE; i++)
      {
        LEDBuffer.setHSV(i, PURPLE_H, PURPLE_S, PURPLE_V);
      }
      LED.setData(LEDBuffer);
    }
  }

  /**
   * This method sets the Driver LED group to a specifed color
   */
  public void humanColorMethod(ColorType colorType) {
    if(ColorType.YELLOW == colorType)
    {
      //Sets each LED to Yellow
      for(int i = DRIVER_START_RANGE; i < LEDBuffer.getLength(); i++)
      {
        LEDBuffer.setHSV(i, YELLOW_H, YELLOW_S, YELLOW_V);
      }
      LED.setData(LEDBuffer);
    }
    else if(ColorType.PURPLE == colorType)
    {
      //Sets each LED to Purple
      for(int i = DRIVER_START_RANGE; i < LEDBuffer.getLength(); i++)
      {
        LEDBuffer.setHSV(i, PURPLE_H, PURPLE_S, PURPLE_V);
      }
      LED.setData(LEDBuffer);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
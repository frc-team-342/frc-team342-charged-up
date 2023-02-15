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
    for(int i = 0; i < LEDBuffer.getLength(); i++)
    {
      LEDBuffer.setHSV(i, 0, 0, 0);
    }
    LED.setData(LEDBuffer);
  }

  /**
   * This method sets the Human Player LED group to the Yellow Color or Purple Color
   */
  public void HumanColorMethod(ColorType colortype) {
    if(ColorType.YELLOW == colortype)
    {
      for(int i = 0; i < 256; i++)
      {
        LEDBuffer.setHSV(i, YELLOW_H, YELLOW_S, YELLOW_V);
      }
      LED.setData(LEDBuffer);
    }
    else if(ColorType.PURPLE == colortype)
    {
      for(int i = 256; i < LEDBuffer.getLength(); i++)
      {
        LEDBuffer.setHSV(i, PURPLE_H, PURPLE_S, PURPLE_V);
      }
      LED.setData(LEDBuffer);
    }
  }

  /**
   * This method sets the Driver LED group to the Yellow Color
   */
  public void DriverColorMethod(ColorType colorType)
  {
    if(ColorType.YELLOW == colorType)
    {
      for(int i = 0; i < 256; i++)
      {
        LEDBuffer.setHSV(i, YELLOW_H, YELLOW_S, YELLOW_V);
      }
      LED.setData(LEDBuffer);
    }
    else if(ColorType.PURPLE == colorType)
    {
      for(int i = 256; i < LEDBuffer.getLength(); i++)
      {
        LEDBuffer.setHSV(i, YELLOW_H, YELLOW_S, YELLOW_V);
      }
      LED.setData(LEDBuffer);
    }
  }

  /**
   * 
   * @return Run: HumanYellowColorMethod() & End: LEDOff()
   */
  public CommandBase HumanYellowColor()
  {
    return runEnd(() -> HumanColorMethod(ColorType.YELLOW), this::LEDOff);
  }

  /**
   * 
   * @return Run: DriverYellowColorMethod() & End: LEDOff()
   */
  public CommandBase DriverYellowColor()
  {
    return runEnd(() -> DriverColorMethod(ColorType.YELLOW), this::LEDOff);
  }

  /**
   * 
   * @return Run: HumanYellowPurpleMethod() & End: LEDOff()
   */
  public CommandBase HumanPurplecolor()
  {
    return runEnd(() -> HumanColorMethod(ColorType.PURPLE), this::LEDOff);
  }

  /**
   * 
   * @return Run: DriverYellowPurpleMethod() & End: LEDOff()
   */
  public CommandBase DriverPurplecolor()
  {
    return runEnd(() -> DriverColorMethod(ColorType.PURPLE), this::LEDOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
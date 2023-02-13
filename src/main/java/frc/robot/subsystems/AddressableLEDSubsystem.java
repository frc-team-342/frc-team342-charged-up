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

  AddressableLED LED;
  AddressableLEDBuffer LEDBuffer;

  public AddressableLEDSubsystem()
  {
    LED = new AddressableLED(0);
    LEDBuffer = new AddressableLEDBuffer(LENGTH);
    LED.setLength(LEDBuffer.getLength());
    LED.setData(LEDBuffer);
    LED.start();
  }

  public void LEDOff()
  {
    for(int i = 0; i < LEDBuffer.getLength(); i++)
      {
        LEDBuffer.setHSV(i, 0, 0, 0);
      }
    LED.setData(LEDBuffer);
  }

  public CommandBase HumanYellowColor()
  {
    return runEnd(
    () ->
    {
      for(int i = 256; i < LEDBuffer.getLength(); i++)
      {
        LEDBuffer.setHSV(i, YELLOW_H, YELLOW_S, YELLOW_V);
      }
      LED.setData(LEDBuffer);
    },

    () ->
    {
      LEDOff();
    }
    );
  }

  public CommandBase DriverYellowColor()
  {
    return runEnd(
    () ->
    {
      for(int i = 0; i < 256; i++)
      {
        LEDBuffer.setHSV(i, YELLOW_H, YELLOW_S, YELLOW_V);
      }
      LED.setData(LEDBuffer);
    },

    () ->
    {
      LEDOff();
    }
    );
  }

  public CommandBase HumanPurplecolor()
  {
    return runEnd(
    () ->
    {
      for(int i = 256; i < LEDBuffer.getLength(); i++)
      {
        LEDBuffer.setHSV(i, PURPLE_H, PURPLE_S, PURPLE_V);
      }
      LED.setData(LEDBuffer);
    },

    () ->
    {
      LEDOff();
    }
    );
  }

  public CommandBase DriverPurplecolor()
  {
    return runEnd(
    () ->
    {
      for(int i = 0; i < 256; i++)
      {
        LEDBuffer.setHSV(i, PURPLE_H, PURPLE_S, PURPLE_V);
      }
      LED.setData(LEDBuffer);
    },

    () ->
    {
      LEDOff();
    }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
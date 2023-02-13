// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AddressableLEDSubsystem extends SubsystemBase {
  /** Creates a new AddressableLEDSubsystem. */

  AddressableLED LED;
  AddressableLEDBuffer LEDBuffer;

  public AddressableLEDSubsystem()
  {
    LED = new AddressableLED(0);
    LEDBuffer = new AddressableLEDBuffer(Constants.length);
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

  public void HumanYellowColor()
  {
    for(int i = 256; i < LEDBuffer.getLength(); i++)
    {
      LEDBuffer.setHSV(i, Constants.YellowH, Constants.YellowS, Constants.YellowV);
    }
    LED.setData(LEDBuffer);
  }

  public void HumanPurplecolor()
  {
    for(int i = 256; i < LEDBuffer.getLength(); i++)
    {
      LEDBuffer.setHSV(i, Constants.PurpleH, Constants.PurpleS, Constants.PurpleV);
    }
    LED.setData(LEDBuffer);
  }

  public void DriverYellowColor()
  {
    for(int i = 0; i < 256; i++)
    {
      LEDBuffer.setHSV(i, Constants.YellowH, Constants.YellowS, Constants.YellowV);
    }
    LED.setData(LEDBuffer);
  }

  public void DriverPurplecolor()
  {
    for(int i = 0; i < 256; i++)
    {
      LEDBuffer.setHSV(i, Constants.PurpleH, Constants.PurpleS, Constants.PurpleV);
    }
    LED.setData(LEDBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
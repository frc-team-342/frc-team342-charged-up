// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GripperConstants.*;



public class GripperSystem extends SubsystemBase {
  /** Creates a new GripperSystem. */
  private final ColorSensorV3 colorSensor;
  public GripperSystem() {
    colorSensor = new ColorSensorV3(i2cPort);

  }


  @Override
  public void initSendable(SendableBuilder builder){

    double IR = colorSensor.getIR();
    int proximity = colorSensor.getProximity();
  
    builder.setSmartDashboardType("GripperSystem");

    builder.addDoubleProperty("Red", () -> colorSensor.getColor().red, null);

    builder.addDoubleProperty("Green", () -> colorSensor.getColor().green, null);
    builder.addDoubleProperty("Blue", () -> colorSensor.getColor().blue, null);
    builder.addDoubleProperty("IR", () -> colorSensor.getIR(), null);
    builder.addDoubleProperty("Proximity", () -> colorSensor.getProximity(), null);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

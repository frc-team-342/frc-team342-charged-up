// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GripperConstants.*;



public class GripperSystem extends SubsystemBase {

  //controls the speed of the spinning wheels
  private CANSparkMax rollerMotor;
  private final ColorSensorV3 colorSensor;

  /** Creates a new GripperSystem. */
  public GripperSystem() {
    rollerMotor = new CANSparkMax(ROLLER_MOTOR, MotorType.kBrushless);
    colorSensor = new ColorSensorV3(I2C_PORT);
  }

  private void spin(double speed){
    rollerMotor.set(speed);
  }

  /**
   * Spins the gripper roller to intake
   * sets speed to 0 to stop
   **/
  public CommandBase intake(){
    return runEnd(
      //run
      () -> {
        spin(ROLLER_SPEED);
      },
      //end
      () -> {
        spin(0);
      }

    );
  }

  /**
   * spins the gripper roller at a negative speed to outtake
   * sets speed to 0 to stop
   **/
  public CommandBase outtake(){
    return runEnd(
      //run
      () -> {
        spin(-(ROLLER_SPEED));
      },
      //end
      () -> {
        spin(0);
      }
    );
  }

  

  @Override
  public void initSendable(SendableBuilder builder){
  
    builder.setSmartDashboardType("GripperSystem");

    builder.addDoubleProperty("Red", () -> colorSensor.getColor().red, null);

    builder.addDoubleProperty("Green", () -> colorSensor.getColor().green, null);
    builder.addDoubleProperty("Blue", () -> colorSensor.getColor().blue, null);
    builder.addDoubleProperty("IR", () -> colorSensor.getIR(), null);
    builder.addDoubleProperty("Proximity", () -> colorSensor.getProximity(), null);
    builder.addDoubleProperty("CurrentDrawReadings", () -> rollerMotor.getOutputCurrent(), null);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

public class DriveSystem extends SubsystemBase {
  private final CANSparkMax frontLeft;
  private final CANSparkMax frontRight;
  private final CANSparkMax backLeft;
  private final CANSparkMax backRight;
  private final MotorControllerGroup left;
  private final MotorControllerGroup right;
  private final DifferentialDrive drive;


  /** Creates a new DriveSystem. */
  public DriveSystem() {
    frontLeft = new CANSparkMax(FRONT_LEFT_MOTOR, MotorType.kBrushless);
    frontRight = new CANSparkMax(FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    backLeft = new CANSparkMax(BACK_LEFT_MOTOR, MotorType.kBrushless);
    backRight = new CANSparkMax(BACK_RIGHT_MOTOR, MotorType.kBrushless);

    left = new MotorControllerGroup(backLeft, frontLeft);
    right = new MotorControllerGroup(frontRight, backRight);

    drive = new DifferentialDrive(left, right);
  }

  /**
   * drives the robot with tank drive
   * @param leftSpeed values -1.0 through 1.0
   * @param rightSpeed values -1.0 through 1.0
   */
  public void drive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * 
   * @param xbox the xbox controller being used to drive the robot
   * @return command that drives with joystick
   */
  public CommandBase driveWithJoystick(XboxController xbox) {
    return runEnd(
      // Runs drive repeatedly until command is stopped
      () -> {
        double left = MathUtil.applyDeadband(xbox.getLeftY(), 0.15);
        double right = MathUtil.applyDeadband(xbox.getRightY(), 0.15);
        
        drive(left, right);
      },
      // Stops robot after command is stopped
      () -> {
        drive(0, 0);
      }
    );
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.Constants.DriveConstants.*;

public class DriveSystem extends SubsystemBase {
  private final CANSparkMax frontLeft;
  private final CANSparkMax frontRight;
  private final CANSparkMax backLeft;
  private final CANSparkMax backRight;

  private final SparkMaxPIDController leftController;
  private final SparkMaxPIDController rightController;

  private final AHRS gyro;

  /** Creates a new DriveSystem. */
  public DriveSystem() {
    // motors
    frontLeft = new CANSparkMax(FRONT_LEFT_MOTOR, MotorType.kBrushless);
    frontRight = new CANSparkMax(FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    backLeft = new CANSparkMax(BACK_LEFT_MOTOR, MotorType.kBrushless);
    backRight = new CANSparkMax(BACK_RIGHT_MOTOR, MotorType.kBrushless);

    // pid controllers
    leftController = frontLeft.getPIDController();
    rightController = frontRight.getPIDController();
    
    // back motors follow voltages from front motor
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    // gyro 
    gyro = new AHRS();

    // simulation initiation
    if (Robot.isSimulation()) {
      // add sparks to physics simulator
      REVPhysicsSim.getInstance().addSparkMax(frontLeft, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(frontRight, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(backLeft, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(backRight, DCMotor.getNEO(1));
    }
  }

  /**
   * drives the robot with tank drive
   * 
   * @param leftSpeed values -1.0 through 1.0, scaled by max speed
   * @param rightSpeed values -1.0 through 1.0, scaled by max speed
   */
  public void driveVelocity(double leftSpeed, double rightSpeed) {
    // left side
    double leftVelocity = leftSpeed * MAX_SPEED;
    leftController.setReference(leftVelocity, ControlType.kVelocity);

    // right side
    double rightVelocity = rightSpeed * MAX_SPEED;
    rightController.setReference(rightVelocity, ControlType.kVelocity);
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
        
        driveVelocity(left, right);
      },
      // Stops robot after command is stopped
      () -> {
        driveVelocity(0, 0);
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // simulation periodic
    if (Robot.isSimulation()) {
      // run rev lib physics sim
      REVPhysicsSim.getInstance().run();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Drive");

    // motor velocities
    builder.addDoubleProperty("Left velocity", frontLeft.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("Right Velocity", frontRight.getEncoder()::getVelocity, null);

    // drivetrain velocity + direction
    // TODO
    builder.addDoubleProperty("Gyro angle", gyro::getAngle, null);
  }
}

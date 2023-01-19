// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final AHRS gyro;

  private final PIDController rotateController;

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

    // encoders
    leftEncoder = frontLeft.getEncoder();
    rightEncoder = frontRight.getEncoder();

    // makes encoders return meters
    leftEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE);
    rightEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE);

    leftEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE);
    rightEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE);
    
    // back motors follow voltages from front motor
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    // gyro 
    gyro = new AHRS();

    // pid
    rotateController = new PIDController(0, 0, 0);
    rotateController.setTolerance(0, 0);

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
  public void drivePercent(double leftSpeed, double rightSpeed) {
    // left side
    double leftVelocity = leftSpeed * MAX_SPEED;
    leftController.setReference(leftVelocity, ControlType.kDutyCycle);

    // right side
    double rightVelocity = rightSpeed * MAX_SPEED;
    rightController.setReference(rightVelocity, ControlType.kDutyCycle);
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
        
        drivePercent(left, right);
      },
      // Stops robot after command is stopped
      () -> {
        drivePercent(0, 0);
      }
    );
  }

  /**
   * 
   * @param distance meters
   * @return command that drives 
   */
  public CommandBase driveDistance(double distance) {
    // meters
    double startPos = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    double endPos = startPos + distance;

    return runEnd(
      // runs repeatedly during command
      () -> {
        leftController.setReference(endPos, ControlType.kPosition);
        rightController.setReference(endPos, ControlType.kPosition);
      }, 
      // runs once at end of command
      () -> {
        // stops motors
        leftController.setReference(0, ControlType.kVelocity);
        rightController.setReference(0, ControlType.kVelocity);

        frontLeft.stopMotor();
        frontRight.stopMotor();
      }
    ).until(
      () -> {
        // meters
        double currPos = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;

        // TODO: check if at distacne
        return false; 
      }
    );
  }

  /**
   * 
   * @param velocity meters/second
   * @return command that drives at given velocity without an end condition
   */
  public CommandBase driveVelocity(double velocity) {
    return runEnd(
      // runs repeatedly until end of command
      () -> {
        // convert from m/s to rpm
        double velocityRpm = velocity * 60 / WHEEL_CIRCUMFERENCE;
      
        // pid controllers take input as rpm
        leftController.setReference(velocityRpm, ControlType.kVelocity);
        rightController.setReference(velocityRpm, ControlType.kVelocity);
      }, 
      // runs once at command end
      () -> {
        // stop motors
        leftController.setReference(0, ControlType.kVelocity);
        rightController.setReference(0, ControlType.kVelocity);

        frontLeft.stopMotor();
        frontRight.stopMotor();
      }
    );
  }

  /**
   * 
   * @param angle robot-relative angle
   * @return
   */
  public CommandBase rotateToAngle(Rotation2d angle) {
    // radians
    double startAngle = Math.toRadians(gyro.getAngle());
    double endAngle = startAngle + angle.getRadians();

    return runEnd(
      // runs repeatedly until end of command
      () -> {
        // radians
        double currAngle = Math.toRadians(gyro.getAngle());
        double nextAngle = rotateController.calculate(currAngle, endAngle);

        // TODO: drivetrain motion to angular velocities
      },
      // runs once at end of command 
      () -> {
        rotateController.reset();

        // stop motors
        leftController.setReference(0, ControlType.kVelocity);
        rightController.setReference(0, ControlType.kVelocity);

        frontLeft.stopMotor();
        frontRight.stopMotor();
      }
    ).until(
      // at setpoint
      rotateController::atSetpoint
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
    builder.addDoubleProperty("Left velocity (RPM)", frontLeft.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("Right Velocity (RPM)", frontRight.getEncoder()::getVelocity, null);

    // drivetrain velocity + direction
    // TODO
    builder.addDoubleProperty("Gyro angle", gyro::getAngle, null);
  }
}

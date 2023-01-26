// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

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

    leftController.setP(0.0);
    leftController.setD(0.0);
    leftController.setFF(0.0);

    rightController.setP(0.0);
    rightController.setD(0.0);
    rightController.setFF(0.0);

    // encoders
    leftEncoder = frontLeft.getEncoder();
    rightEncoder = frontRight.getEncoder();

    // makes encoders return meters
    leftEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE / GEAR_RATIO);
    rightEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE / GEAR_RATIO);

    leftEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE * (1.0/60.0) / GEAR_RATIO);
    rightEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE * (1.0/60.0) / GEAR_RATIO);
    
    // back motors follow voltages from front motor
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    // gyro 
    gyro = new AHRS();

    // pid
    rotateController = new PIDController(0, 0, 0); // TODO: tune pid controller
    rotateController.setTolerance(0, 0);
    
    // kinematics + odometry
    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(gyro.getAngle()), 
      leftEncoder.getPosition(), 
      rightEncoder.getPosition()
    );

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
   * @param velocityIn meters/second
   * @param distance meters
   * @return command that drives 
   */
  public CommandBase driveDistance(double velocityIn, double distance) {
    double velocity = MathUtil.clamp(velocityIn, -MAX_SPEED, MAX_SPEED);

    // TODO: refactor to use odometry
    Pose2d start = odometry.getPoseMeters();
    Transform2d transform = new Transform2d(
      // distance from current position facing current direction
      new Translation2d(distance, start.getRotation()), 
      // don't rotate while driving
      new Rotation2d(0.0)
    );
    Pose2d end = start.plus(transform);

    // start heading is recorded to make sure it stays straight
    Rotation2d startAngle = start.getRotation();

    return runEnd(
      // runs repeatedly during command
      () -> {
        // get current heading
        Rotation2d currentAngle = Rotation2d.fromDegrees(gyro.getAngle());
        Rotation2d error = currentAngle.minus(startAngle); // radians

        // use rotation controller to drive error to zero to drive straight
        double rotation = rotateController.calculate(error.getRadians(), 0);

        // drive at velocity from parameter
        leftController.setReference(velocity + (-1 * rotation), ControlType.kVelocity); 
        rightController.setReference(velocity + rotation, ControlType.kVelocity);
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
        // get current distance from original position
        Pose2d curr = odometry.getPoseMeters();
        Transform2d distTraveled = curr.minus(start);
        
        // check that current distance is close to intended distance
        double dist = Math.hypot(distTraveled.getX(), distTraveled.getY());
        return (distance - 0.3) < dist && (distance + 0.3) > dist; // TODO: replace tolerance with constant 
      }
    );
  }

  /**
   * @param velocityIn meters/second
   * @return command that drives at given velocity without an end condition
   */
  public CommandBase driveVelocity(double velocityIn) {
    double velocity = MathUtil.clamp(velocityIn, -MAX_SPEED, MAX_SPEED);

    return runEnd(
      // runs repeatedly until end of command
      () -> {      
        // units don't need to be adjusted because of encoder conversion factor
        leftController.setReference(velocity, ControlType.kVelocity);
        rightController.setReference(velocity, ControlType.kVelocity);
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

        // rad/s ????
        double nextVel = rotateController.calculate(currAngle, endAngle);

        // convert radial velocity to drivetrain speeds
        ChassisSpeeds rotationVel = new ChassisSpeeds(0, 0, nextVel);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(rotationVel);

        // clamp wheel speeds to max velocity
        double left = MathUtil.clamp(wheelSpeeds.leftMetersPerSecond, -MAX_SPEED, MAX_SPEED);
        double right = MathUtil.clamp(wheelSpeeds.rightMetersPerSecond, -MAX_SPEED, MAX_SPEED);

        // apply drivetrain speeds to drive pid controllers
        leftController.setReference(left, ControlType.kVelocity);
        rightController.setReference(right, ControlType.kVelocity);
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
      // returns true if robot is at end angle
      rotateController::atSetpoint
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // update robot position
    odometry.update(
      Rotation2d.fromDegrees(gyro.getAngle()), 
      leftEncoder.getPosition(), 
      rightEncoder.getPosition()
    );

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

    // odometry positions
    builder.addDoubleProperty("Odometry X position (m)", () -> odometry.getPoseMeters().getX(), null);
    builder.addDoubleProperty("Odometry Y position (m)", () -> odometry.getPoseMeters().getY(), null);
    builder.addDoubleProperty("Odometry angle (deg)", () -> odometry.getPoseMeters().getRotation().getDegrees(), null);
  }
}

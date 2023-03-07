// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
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
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


import static frc.robot.Constants.DriveConstants.*;

import java.util.List;

public class DriveSystem extends SubsystemBase implements Testable {
  // speeds are statically imported constants
  private enum Mode {

    NORMAL(NORMAL_SPEED),
    SLOW(SLOW_SPEED);

    public final double speedMultiplier;

    private Mode(double speedMultiplier) {
      this.speedMultiplier = speedMultiplier;
    }
  }

  private final CANSparkMax frontLeft;
  private final CANSparkMax frontRight;
  private final CANSparkMax backLeft;
  private final CANSparkMax backRight;

  private final SparkMaxPIDController leftController;
  private final SparkMaxPIDController rightController;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final AHRS gyro;

  private final AHRS navX;

  private final PIDController rotateController;

  private final PIDController balanceController;

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  private final LinearSystem<N2, N2, N2> model;
  private final DifferentialDrivetrainSim drivetrainSim;

  private final Field2d field;

  private Mode currentMode = Mode.NORMAL;

  /** Creates a new DriveSystem. */
  public DriveSystem() {
    // motors
    frontLeft = new CANSparkMax(FRONT_LEFT_MOTOR, MotorType.kBrushless);
    frontRight = new CANSparkMax(FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    backLeft = new CANSparkMax(BACK_LEFT_MOTOR, MotorType.kBrushless);
    backRight = new CANSparkMax(BACK_RIGHT_MOTOR, MotorType.kBrushless);

    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);

    //navX
    navX = new AHRS();

    // pid controllers
    leftController = frontLeft.getPIDController();
    rightController = frontRight.getPIDController();

    leftController.setP(0.001);
    leftController.setD(0.001);
    leftController.setFF(1.0);

    rightController.setP(0.001);
    rightController.setD(0.001);
    rightController.setFF(1.0);

    // encoders
    leftEncoder = frontLeft.getEncoder();
    rightEncoder = frontRight.getEncoder();

    // makes encoders return meters
    leftEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE / GEAR_RATIO );
    rightEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE / GEAR_RATIO);

    leftEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE * (1.0/60.0) / GEAR_RATIO);
    rightEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE * (1.0/60.0) / GEAR_RATIO);
    
    // Inverts the leader motors
    frontRight.setInverted(true);
    frontLeft.setInverted(false);

    // back motors follow voltages from front motor
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    // gyro 
    gyro = new AHRS();

    // pid
    rotateController = new PIDController(0, 0, 0); // TODO: tune pid controller
    rotateController.setTolerance(Math.PI / 4, 0);

    balanceController = new PIDController(0.85, 0, 0); // TODO: tune/test number 
    balanceController.setTolerance(Math.PI / 4, 0);
    
    // kinematics
    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);    
    
    // state-space system model
    model = LinearSystemId.createDrivetrainVelocitySystem(
      DCMotor.getNEO(4), // motors used in the drivetrain 
      MASS, 
      WHEEL_RADIUS, 
      TRACK_WIDTH / 2, 
      MOMENT_OF_INERTIA, 
      1 / GEAR_RATIO
    );

    // drivetrain simulation
    drivetrainSim = new DifferentialDrivetrainSim(
      model, // drivetrain state-space model
      DCMotor.getNEO(4),
      1 / GEAR_RATIO,
      TRACK_WIDTH,
      WHEEL_RADIUS,
      null
    );

    // robot pose on field visualization
    field = new Field2d();
    SmartDashboard.putData(field);

    // simulation initiation
    if (Robot.isSimulation()) {
      // add sparks to physics simulator
      REVPhysicsSim.getInstance().addSparkMax(frontLeft, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(frontRight, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(backLeft, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(backRight, DCMotor.getNEO(1));
    }

    // odometry instantiated differently in sim or real robot
    if (Robot.isReal()) {
      odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(gyro.getAngle()), 
        leftEncoder.getPosition(), 
        rightEncoder.getPosition()
      );
    } else {
      odometry = new DifferentialDriveOdometry(
        drivetrainSim.getHeading(), 
        drivetrainSim.getLeftPositionMeters(), 
        drivetrainSim.getRightPositionMeters()
      );
    }
  }

  /** Changes the speed multiplier between the normal mode to slow mode */
  public void toggleSlowMode() {
    if (currentMode != Mode.SLOW) {
      currentMode = Mode.SLOW;
    } else {
      currentMode = Mode.NORMAL;   
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
    double leftVelocity = leftSpeed * currentMode.speedMultiplier;
    leftController.setReference(leftVelocity, ControlType.kDutyCycle);

    // right side
    double rightVelocity = rightSpeed * currentMode.speedMultiplier;
    rightController.setReference(rightVelocity, ControlType.kDutyCycle);
  }

  /**
   * 
   * @param joyLeft the left joystick being used to drive the robot
   * @param joyRight the right joystick being used to drive the robot
   * @return command that drives with joystick
   */
  public CommandBase driveWithJoystick(Joystick joyLeft, Joystick joyRight) {
    return runEnd(
      // Runs drive repeatedly until command is stopped
      () -> {
        double left = MathUtil.applyDeadband(joyLeft.getY(), 0.15);
        double right = MathUtil.applyDeadband(joyRight.getY(), 0.15);
        
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

        setDrivePIDControllers(wheelSpeeds);
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
/**
 * it returns a command that autobalances
 * @return
 */

/* public CommandBase autoBalance() {
    
    

    return runEnd(
      // runs repeatedly until end of command
      () -> {
        // radians
        double currAngle = Math.toRadians(gyro.getRoll());

        double endAngle = 0;

        // rad/s ????
        double nextVel = balanceController.calculate(currAngle, endAngle);

        // convert radial velocity to drivetrain speeds
        ChassisSpeeds balanceVel = new ChassisSpeeds(nextVel, 0, 0);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(balanceVel);

        setDrivePIDControllers(wheelSpeeds);

        
      },
      // runs once at end of command 
      () -> {
        balanceController.reset();
        
        // stop motors
        leftController.setReference(0, ControlType.kVelocity);
        rightController.setReference(0, ControlType.kVelocity);
        
        frontLeft.stopMotor();
        frontRight.stopMotor();
      }
    ).until(
      balanceController::atSetpoint
    );
    
  } 
  */

public CommandBase autoBalance(){
  
  return runEnd(
    
      () -> {

        double maxPercentOutput = 0.3;
        double maxAngle = 20;
        double angle = -MathUtil.clamp(navX.getRoll(), -maxAngle, maxAngle ); // Negative because of robot orientation
        double speed = MathUtil.clamp((angle / maxAngle) * maxPercentOutput, -maxPercentOutput, maxPercentOutput); // Speed is proportional to the angle 
        
        double tolerance = 3;
        //Add a variable called "tolerance" in degrees

        //Change the logig of oyur if statement to say if the angle is inside tolerance, don't move, otherwise move.

        System.out.println("Angle: " + angle);
        System.out.println("Speed: " + speed);

        if (angle < tolerance && angle > -tolerance) {
          drivePercent(0, 0);
        } 
        else {
          drivePercent(speed, speed);
        }

      },

      // when it ends

      () -> {
        driveVelocity(0);
      }

    );
  }




  /**
   * sets the reference velocity of the PID controllers
   * @param wheelSpeeds - the desired referenece velocity for the PID controller  
   */
  private void setDrivePIDControllers(DifferentialDriveWheelSpeeds wheelSpeeds) {
    // clamp wheel speeds to max velocity
    double left = MathUtil.clamp(wheelSpeeds.leftMetersPerSecond, -MAX_SPEED, MAX_SPEED);
    double right = MathUtil.clamp(wheelSpeeds.rightMetersPerSecond, -MAX_SPEED, MAX_SPEED);

    // apply drivetrain speeds to drive pid controllers
    leftController.setReference(left, ControlType.kVelocity);
    rightController.setReference(right, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // odometry and pose visualization update different in simulation and real
    if (Robot.isReal()) {
      // update from real encoder and gyro values
      odometry.update(
        Rotation2d.fromDegrees(-gyro.getAngle()), 
        leftEncoder.getPosition(), 
        rightEncoder.getPosition()
      );

      // update field visualization from odometry in real robot
      field.setRobotPose(odometry.getPoseMeters());
    } else {
      // update from drivetrain sim
      odometry.update(
        drivetrainSim.getHeading(), 
        drivetrainSim.getLeftPositionMeters(), 
        drivetrainSim.getRightPositionMeters()
      );

      // update field visualization from drivetrain sim in simulation
      field.setRobotPose(drivetrainSim.getPose());
    } 
  }

  @Override
  public void simulationPeriodic() {
    // run rev lib physics sim
    REVPhysicsSim.getInstance().run();

    // set sim navx to track data from drivetrain rotation
    int device = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(device, "Yaw"));

    // navx is counterclockwise positive
    double currentAngle = drivetrainSim.getHeading().getDegrees();
    angle.set(Math.IEEEremainder(-currentAngle, 360)); 

    // set inputs to drivesystem simulation
    drivetrainSim.setInputs(
      frontLeft.getAppliedOutput() * RobotController.getInputVoltage(), 
      frontRight.getAppliedOutput() * RobotController.getInputVoltage()
    );

    // drivetrain simulation update with timestamp
    drivetrainSim.update(0.02);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Drive");

    // motor velocities
    builder.addDoubleProperty("Left velocity", frontLeft.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("Right Velocity", frontRight.getEncoder()::getVelocity, null);

    if (Robot.isSimulation()) {
      builder.addDoubleProperty("Simulation left velocity", drivetrainSim::getLeftVelocityMetersPerSecond, null);
      builder.addDoubleProperty("Simulation right velocity", drivetrainSim::getRightVelocityMetersPerSecond, null);
    }

    // drivetrain velocity + direction
    builder.addDoubleProperty("Gyro angle", gyro::getAngle, null);
    builder.addDoubleProperty("Gyro Pitch", gyro::getPitch, null);
    builder.addDoubleProperty("Gyro Roll", gyro::getRoll, null);

    // odometry positions
    builder.addDoubleProperty("Odometry X position (m)", () -> odometry.getPoseMeters().getX(), null);
    builder.addDoubleProperty("Odometry Y position (m)", () -> odometry.getPoseMeters().getY(), null);
    builder.addDoubleProperty("Odometry angle (deg)", () -> odometry.getPoseMeters().getRotation().getDegrees(), null);

    if (Robot.isSimulation()) {
      builder.addDoubleProperty("Voltage", RobotController::getInputVoltage, null);
      builder.addDoubleProperty("Left output", () -> frontLeft.getAppliedOutput(), null);
      builder.addDoubleProperty("Right output", () -> frontRight.getAppliedOutput(), null);
    }
  }

  @Override
  public List<Connection> hardwareConnections() {
    return List.of(
      // default constructor for spark connections
      Connection.fromSparkMax(frontLeft),
      Connection.fromSparkMax(frontRight),
      Connection.fromSparkMax(backLeft),
      Connection.fromSparkMax(backRight),
      // default constructor for navx connection
      Connection.fromNavx(gyro)
    );
  }

  @Override
  public CommandBase testRoutine() {
    // if/when joystick drive uses pid this should be changed to also use pid
    return Commands.sequence(
      // drive forwards
      new RunCommand(
        () -> {
          // both wheels forwards at speed set by mode
          drivePercent(currentMode.speedMultiplier, currentMode.speedMultiplier);
        }, 
        this // this command depends on drivesystem
      ).withTimeout(1.5), // run this for 3 seconds before continuing

      // drive backwards
      new RunCommand(
        () -> {
          // both wheels backwards at speed set by mode
          drivePercent(-currentMode.speedMultiplier, -currentMode.speedMultiplier);
        }, 
        this
      ).withTimeout(1.5),

      // drive clockwise
      new RunCommand(
        () -> {
          // left wheel forwards right wheel backwards
          drivePercent(currentMode.speedMultiplier, -currentMode.speedMultiplier);
        }, 
        this 
      ).withTimeout(1.5), 

      // drive counterclockwise
      new RunCommand(
        () -> {
          // left wheel backwards right wheel forwards
          drivePercent(-currentMode.speedMultiplier, currentMode.speedMultiplier);
        }, 
        this
      ).withTimeout(1.5),

      // enable slow mode
      new InstantCommand(
        () -> { 
          toggleSlowMode();
        },
        this
      ),

      // drive forwards in slow mode
      new RunCommand(
        () -> {
          // both wheels forwards
          drivePercent(currentMode.speedMultiplier, currentMode.speedMultiplier);
        }, 
        this
      ).andThen(
        () -> {
          // stop driving
          drivePercent(0, 0);
        }, 
        this
      ).withTimeout(1.5),

      // disable slow mode
      new InstantCommand(
        () -> { 
          toggleSlowMode(); 
        },
        this
      )
    );
  }
}

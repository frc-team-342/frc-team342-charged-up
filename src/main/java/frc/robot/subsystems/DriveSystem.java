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
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.Robot;
import frc.robot.commands.drive.DriveVelocity;

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
  private final Limelight limelight;

  private final PIDController rotateController;

  private final RamseteController ramsete;
  private final TrajectoryConfig defaultConfig;

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDrivePoseEstimator poseEstimator;

  private final LinearSystem<N2, N2, N2> model;
  private final DifferentialDrivetrainSim drivetrainSim;

  private final Field2d field;

  private Mode currentMode = Mode.NORMAL;

  /** Creates a new DriveSystem. */
  public DriveSystem(Limelight limelight) {
    // motors
    frontLeft = new CANSparkMax(FRONT_LEFT_MOTOR, MotorType.kBrushless);
    frontRight = new CANSparkMax(FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    backLeft = new CANSparkMax(BACK_LEFT_MOTOR, MotorType.kBrushless);
    backRight = new CANSparkMax(BACK_RIGHT_MOTOR, MotorType.kBrushless);

    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);

    // pid controllers
    leftController = frontLeft.getPIDController();
    rightController = frontRight.getPIDController();

    leftController.setP(0.001);
    leftController.setD(0.001);
    leftController.setFF(0.15);

    rightController.setP(0.001);
    rightController.setD(0.001);
    rightController.setFF(0.15);

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

    setBrakeMode(true);

    // back motors follow voltages from front motor
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    frontLeft.setSmartCurrentLimit(60);
    backLeft.setSmartCurrentLimit(60);
    frontRight.setSmartCurrentLimit(60);
    backLeft.setSmartCurrentLimit(60);

    // gyro 
    gyro = new AHRS();

    // limelight
    this.limelight = limelight;

    // pid
    rotateController = new PIDController(0, 0, 0); // TODO: tune pid controller
    rotateController.setTolerance(Math.PI / 4, 0);

    // trajectory stuff
    ramsete = new RamseteController();
    defaultConfig = new TrajectoryConfig(MAX_SPEED, MAX_ACCEL);
    
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
      poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics, 
        Rotation2d.fromDegrees(-gyro.getAngle()), 
        leftEncoder.getPosition(), 
        rightEncoder.getPosition(), 
        new Pose2d()
      );
    } else {
      poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics, 
        drivetrainSim.getHeading(), 
        leftEncoder.getPosition(), 
        rightEncoder.getPosition(), 
        new Pose2d()
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
   * sets the idle mode of the drivetrain motors
   * @param brake brake mode if true, coast mode if false
   */
  public void setBrakeMode(boolean brake) {
    // ternary operator: `(condition) ? (value if true) : (value if false)`
    IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;

    frontLeft.setIdleMode(mode);
    frontRight.setIdleMode(mode);
    backLeft.setIdleMode(mode);
    backRight.setIdleMode(mode);
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

  public void drivePercentForTime(double leftSpeed, double rightSpeed, double time) {

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

  /** stop all drive motors */
  public void stopMotors() {
    frontLeft.stopMotor();
    frontRight.stopMotor();
    backLeft.stopMotor();
    backRight.stopMotor();
  }

  /**
   * get the current yaw of the robot
   * @return
   */
  public Rotation2d getGyroAngle() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * current robot pose from odometry
   * @return relative position, start is (0, 0)
   */
  public Pose2d getOdometryPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  public TrajectoryConfig getDefaultTrajetoryConfig() {
    return defaultConfig;
  }

  /**
   * convert robot-relative speeds to wheel speeds
   * @param speeds x vel, y vel, rotational vel
   * @return left and right wheel speeds
   */
  public DifferentialDriveWheelSpeeds inverseKinematics(ChassisSpeeds speeds) {
    return kinematics.toWheelSpeeds(speeds);
  }

  public CommandBase followTrajectory(Trajectory trajectory) {
    Timer trajectoryTime = new Timer();

    return new FunctionalCommand(
      // runs once at start
      () -> {
        // start timer for trajectory
        trajectoryTime.reset();
        trajectoryTime.start();
      }, 
      // runs repeatedly during command execution
      () -> {
        // sample the trajectory at current timestamp
        Trajectory.State goal = trajectory.sample(trajectoryTime.get());

        // find wheel speeds based on robot speeds at timestamp
        ChassisSpeeds robotSpeeds = ramsete.calculate(getOdometryPosition(), goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = inverseKinematics(robotSpeeds);

        System.out.println("Left: " + wheelSpeeds.leftMetersPerSecond);
        System.out.println("Right: " + wheelSpeeds.rightMetersPerSecond);
        System.out.println("Time: " + trajectoryTime.get());

        // set robot speed based on goal
        this.setVelocity(wheelSpeeds);
      }, 
      // runs once at command end
      (Boolean interrupted) -> {
        // stop updating timestamp
        trajectoryTime.stop();

        // stop drive motors
        this.setVelocity(new DifferentialDriveWheelSpeeds(0, 0));
      },
      // determines when command ends 
      () -> {
        // TODO: check against end pose instead of time
        return trajectory.getTotalTimeSeconds() < trajectoryTime.get(); 
      }, 
      // subsystems this command depends on
      this
    );
  }

  /**
   * automatically balance on the charge station
   */
  public CommandBase autoBalance() {
    return runEnd(
      // runs repeatedly while command active
      () -> {
        // robot is back-heavy
        double forwardP = 0.17;
        double backP = 0.32;

        // maximum drivetrain output
        double maxPercentOutput = 0.33;

        double maxAngle = 20;

        // Negative because of robot orientation
        double angle = -MathUtil.clamp(gyro.getRoll(), -maxAngle, maxAngle); 

        // Speed is proportional to the angle 
        //double speed = MathUtil.clamp((angle / maxAngle) * proportional, -maxPercentOutput, maxPercentOutput); 

        double speed = (angle > 0)
          ? (angle / maxAngle) * forwardP
          : (angle / maxAngle) * backP;

        speed = MathUtil.clamp(speed, -maxPercentOutput, maxPercentOutput);
        
        // degrees
        double tolerance = 3;

        if (angle < tolerance && angle > -tolerance) {
          // hold position if within roll tolerance
          drivePercent(0, 0);
        } else {
          // drive to balance if outside of roll tolerance
          drivePercent(speed, speed);
        }
      },
      // when it ends
      () -> {
        leftController.setReference(0.0, ControlType.kVelocity);
        rightController.setReference(0.0, ControlType.kVelocity);
      }
    );
  }

  /**
   * sets the reference velocity of the PID controllers
   * @param wheelSpeeds - the desired referenece velocity for the PID controller  
   */
  public void setVelocity(DifferentialDriveWheelSpeeds wheelSpeeds) {
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
      poseEstimator.update(
        Rotation2d.fromDegrees(-gyro.getAngle()), 
        leftEncoder.getPosition(), 
        rightEncoder.getPosition()
      );

      // update field visualization from odometry in real robot
      field.setRobotPose(poseEstimator.getEstimatedPosition());
    } else {
      // update from drivetrain sim
      poseEstimator.update(
        drivetrainSim.getHeading(), 
        drivetrainSim.getLeftPositionMeters(), 
        drivetrainSim.getRightPositionMeters()
      );

      // update field visualization from drivetrain sim in simulation
      field.setRobotPose(drivetrainSim.getPose());
    }

    // update pose estimator from vision
    if (limelight.hasTargets()) {
      poseEstimator.addVisionMeasurement(limelight.getPosition(), Timer.getFPGATimestamp());
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

    // used for autobalance
    builder.addDoubleProperty("Roll (deg)", gyro::getRoll, null);

    // motor velocities
    builder.addDoubleProperty("Left velocity", leftEncoder::getVelocity, null);
    builder.addDoubleProperty("Right velocity", rightEncoder::getVelocity, null);

    if (Robot.isSimulation()) {
      builder.addDoubleProperty("Simulation left velocity", drivetrainSim::getLeftVelocityMetersPerSecond, null);
      builder.addDoubleProperty("Simulation right velocity", drivetrainSim::getRightVelocityMetersPerSecond, null);
    }

    // drivetrain velocity + direction
    builder.addDoubleProperty("Gyro angle", gyro::getAngle, null);

    // odometry positions
    builder.addDoubleProperty("Odometry X position (m)", () -> poseEstimator.getEstimatedPosition().getX(), null);
    builder.addDoubleProperty("Odometry Y position (m)", () -> poseEstimator.getEstimatedPosition().getY(), null);
    builder.addDoubleProperty("Odometry angle (deg)", () -> this.getGyroAngle().getDegrees(), null);
    builder.addDoubleProperty("Odometry angle (rad)", () -> this.getGyroAngle().getRadians(), null);

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

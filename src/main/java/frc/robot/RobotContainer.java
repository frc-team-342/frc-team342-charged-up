// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.subsystems.AddressableLEDSubsystem.ColorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final DriveSystem driveSystem;
  private final LiftSystem lSystem;

  private POVButton liftUp;
  private POVButton liftMidL;
  private POVButton liftMidR;
  private POVButton liftDown;

  private JoystickButton liftSpeedButton;
  private final DriveSystem driveSystem;

  private final Limelight limelight;

  private final GripperSystem gripperSystem;

  private final AddressableLEDSubsystem aLEDSub;
  
  /* Controller and button instantiations */
  private final XboxController operator;
  private final JoystickButton rightBumper;
  private final Trigger rightTrigger;
  private final Trigger leftTrigger;
  private final Joystick driverLeft;
  private final Joystick driverRight;

  // hardware connection check stuff
  private final NetworkTable hardware = NetworkTableInstance.getDefault().getTable("Hardware");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    operator = new XboxController(OperatorConstants.OP_CONTROLLER);
    rightBumper = new JoystickButton(operator, OperatorConstants.OP_BUTTON_CONE_INTAKE);
    rightTrigger = new Trigger(() -> { return (operator.getRightTriggerAxis() >= 0.8); });
    leftTrigger = new Trigger(() -> { return (operator.getLeftTriggerAxis() >= 0.8); });
    driverLeft = new Joystick(OperatorConstants.DRIVER_LEFT_PORT);
    driverRight = new Joystick(OperatorConstants.DRIVER_RIGHT_PORT);

    /** Drivesystem instantiations */
    driveSystem = new DriveSystem();
    driveSystem.setDefaultCommand(driveSystem.driveWithJoystick(driverLeft, driverRight));
  
    lSystem = new LiftSystem();
    lSystem.setDefaultCommand(lSystem.liftArms(operator));

    liftUp = new POVButton(operator, 0);
    liftMidL = new POVButton(operator, 90);
    liftMidR = new POVButton(operator, 270);
    liftDown = new POVButton(operator, 180);

    /** Limelight instantiations */
    limelight = new Limelight();

    /** Gripper instantiations */
    gripperSystem = new GripperSystem(limelight);
    gripperSystem.setDefaultCommand(gripperSystem.hold());

    aLEDSub = new AddressableLEDSubsystem();

    /** Dashboard sendables for the subsystems go here */
    SmartDashboard.putData(driveSystem);
    SmartDashboard.putData(gripperSystem);
    SmartDashboard.putData(limelight);
    SmartDashboard.putData(lSystem);
    
    // Configure the trigger bindings
    configureBindings();

    // hardware check
    Shuffleboard.getTab("Hardware").add(getCheckCommand());
    Shuffleboard.getTab("Hardware").add(CommandScheduler.getInstance());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    rightBumper.whileTrue(gripperSystem.coneIntake(aLEDSub));
    rightTrigger.whileTrue(gripperSystem.cubeIntake(aLEDSub));
    leftTrigger.whileTrue(gripperSystem.outtake(aLEDSub));

    liftUp.whileTrue(lSystem.liftArmsToPosition(LiftConstants.TOP_POSITION));
    liftMidL.whileTrue(lSystem.liftArmsToPosition(LiftConstants.MID_POSITION));
    liftMidR.whileTrue(lSystem.liftArmsToPosition(LiftConstants.MID_POSITION));
    liftDown.whileTrue(lSystem.liftArmsToPosition(LiftConstants.LOW_POSITION));

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  private CommandBase getCheckCommand() {
    return Commands.sequence(
      // drive
      new InstantCommand(
        () -> { hardware.getEntry("Drive").setString(driveSystem.checkAllConnections()); },
        driveSystem
      ),

      // lift
      new InstantCommand(
        () -> { hardware.getEntry("Lift").setString(driveSystem.checkAllConnections()); },
        lSystem
      ),

      // gripper
      new InstantCommand(
        () -> { hardware.getEntry("Gripper").setString(gripperSystem.checkAllConnections()); },
        gripperSystem
      ),

      // limelight
      new InstantCommand(
        () -> { hardware.getEntry("Limelight").setString(limelight.checkAllConnections()); }
      )
    ).ignoringDisable(true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.DriveSlow(driveSystem);
  }

  public Command getTestCommand() {
    // return all test routines chained together
    return new SequentialCommandGroup(
      // check that all devices are connected
      getCheckCommand(),
      
      /*
       * drive system test routine:
       * - drive forwards
       * - drive backwards
       * - turn clockwise
       * - turn counterclockwise
       * - drive forwards in slow mode
       */
      driveSystem.testRoutine(),

      /*
       * intake
       */
      gripperSystem.testRoutine(),

      /*
       * led test routine
       * - both sides purple for 2 seconds
       * - both sides yellow for 2 seconds
       * - leds off
       */
      aLEDSub.testRoutine(),

      /*
       * limelight test routine
       * - blink leds for 2 seconds
       * - go back to pipeline default for leds
       */
      limelight.testRoutine()

      // arm test is not included for safety reasons
    );
  }

  public void setBrakeMode(boolean mode){
    lSystem.setBrakeMode(mode);
  }
}

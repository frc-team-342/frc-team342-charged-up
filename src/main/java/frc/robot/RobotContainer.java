// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.auto.LiftArmToPosition;
import frc.robot.commands.drive.DriveDistance;
import frc.robot.commands.gripper.Hold;
import frc.robot.commands.led.Rainbow;
import frc.robot.subsystems.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final LEDSystem led;

  private POVButton liftUp;
  private POVButton liftMidL;
  private POVButton liftMidR;
  private POVButton liftDown;

  private final DriveSystem driveSystem;

  private final Limelight limelight;

  private final GripperSystem gripperSystem;
  
  /* Controller and button instantiations */
  private final XboxController operator;
  private final JoystickButton rightBumper;
  private final Trigger rightTrigger;
  private final Trigger leftTrigger;
  private final JoystickButton xButton;
  private final JoystickButton aButton;
  private final JoystickButton bButton;
  private final JoystickButton yButton;

  private final Joystick driverLeft;
  private final Joystick driverRight;
  private final JoystickButton balanceLeftBtn;
  private final JoystickButton balanceRightBtn;

  private SendableChooser<Command> autoChooser;

  private final InstantCommand togglePipeline;

  // hardware connection check stuff
  private final NetworkTable hardware = NetworkTableInstance.getDefault().getTable("Hardware");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // controllers
    operator = new XboxController(OperatorConstants.OP_CONTROLLER);
    driverLeft = new Joystick(OperatorConstants.DRIVER_LEFT_PORT);
    driverRight = new Joystick(OperatorConstants.DRIVER_RIGHT_PORT);
    
    // autobalance driver buttons
    balanceLeftBtn = new JoystickButton(driverLeft, 2);
    balanceRightBtn = new JoystickButton(driverRight, 2);

    // intake + outtake
    rightBumper = new JoystickButton(operator, OperatorConstants.OP_BUTTON_CONE_INTAKE);
    rightTrigger = new Trigger(() -> { return (operator.getRightTriggerAxis() >= 0.8); });
    leftTrigger = new Trigger(() -> { return (operator.getLeftTriggerAxis() >= 0.8); });

    // xbox controller buttons
    xButton = new JoystickButton(operator, XboxController.Button.kX.value);
    aButton = new JoystickButton(operator, XboxController.Button.kA.value);
    yButton = new JoystickButton(operator, XboxController.Button.kY.value);
    bButton = new JoystickButton(operator, XboxController.Button.kB.value);

    // operator assist lift buttons
    liftUp = new POVButton(operator, 0);
    liftMidL = new POVButton(operator, 90);
    liftMidR = new POVButton(operator, 270);
    liftDown = new POVButton(operator, 180);

    /** Drivesystem instantiations */
    driveSystem = new DriveSystem();
    driveSystem.setDefaultCommand(driveSystem.driveWithJoystick(driverLeft, driverRight));

    led = new LEDSystem();
  
    lSystem = new LiftSystem();
    lSystem.setDefaultCommand(lSystem.liftArms(operator));

    /** Limelight instantiations */
    limelight = new Limelight();

    /** Gripper instantiations */
    gripperSystem = new GripperSystem();
    gripperSystem.setDefaultCommand(new Hold(gripperSystem, led));

    /** Dashboard sendables for the subsystems go here */
    SmartDashboard.putData(driveSystem);
    SmartDashboard.putData(gripperSystem);
    SmartDashboard.putData(limelight);
    SmartDashboard.putData(lSystem);

    togglePipeline = new InstantCommand(limelight::togglePipeline);
    
    // Configure the trigger bindings
    configureBindings();

    // hardware check
    Shuffleboard.getTab("Hardware").add(getCheckCommand());
     Shuffleboard.getTab("Hardware").add(CommandScheduler.getInstance());

    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Score low and balance", Autos.outtakeAndBalance(driveSystem, lSystem, gripperSystem, led));
    autoChooser.addOption("Do nothing", new InstantCommand());

    // blue side
    autoChooser.addOption("2-Side Blue Leave: 1 Piece", Autos.leftSideBlue(driveSystem, lSystem, gripperSystem, led, false));
    autoChooser.addOption("8-Side Blue Leave: 1 Piece", Autos.rightSideBlue(driveSystem, lSystem, gripperSystem, led, false));

    // red side
    autoChooser.addOption("8-Side Red Leave: 1 Piece", Autos.rightSideRed(driveSystem, lSystem, gripperSystem, led, false));
    autoChooser.addOption("2-Side Red Leave: 1 Piece", Autos.leftSideRed(driveSystem, lSystem, gripperSystem, led, false));
    
    // blue side
    autoChooser.addOption("2-Side Blue Leave: 2 Piece", Autos.leftSideBlue(driveSystem, lSystem, gripperSystem, led, true));
    autoChooser.addOption("8-Side Blue Leave: 2 Piece", Autos.rightSideBlue(driveSystem, lSystem, gripperSystem, led, true));

    // red side
    autoChooser.addOption("8-Side Red Leave: 2 Piece", Autos.rightSideRed(driveSystem, lSystem, gripperSystem, led, true));
    autoChooser.addOption("2-Side Red Leave: 2 Piece", Autos.leftSideRed(driveSystem, lSystem, gripperSystem, led, true));

    SmartDashboard.putData(autoChooser);
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
    rightBumper.whileTrue(gripperSystem.intake());
    rightTrigger.whileTrue(gripperSystem.intake());
    leftTrigger.whileTrue(gripperSystem.outtake());

    xButton.whileTrue(led.humanPlayer(LEDConstants.YELLOW));
    xButton.whileTrue(led.humanPlayer(LEDConstants.PURPLE));

    yButton.onTrue(togglePipeline);

    // autobalance driver buttons
    balanceLeftBtn.whileTrue(driveSystem.autoBalance());
    balanceRightBtn.whileTrue(driveSystem.autoBalance());
    
    // operator assist arm lift buttons
    liftUp.whileTrue(new LiftArmToPosition(lSystem, LiftConstants.TOP_POSITION));
    liftMidL.whileTrue(new LiftArmToPosition(lSystem, LiftConstants.MID_POSITION));
    liftMidR.whileTrue(new LiftArmToPosition(lSystem, LiftConstants.MID_POSITION));
    liftDown.whileTrue(new LiftArmToPosition(lSystem, LiftConstants.LOW_POSITION));
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
    return autoChooser.getSelected();
  }

  public Command getDisabledCommand() {
    return new Rainbow(led).ignoringDisable(true);
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
      led.testRoutine(),

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

  public void disableLL3DMode(){
    limelight.setPipeline(0);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.GripperSystem;
import frc.robot.subsystems.LiftSystem;

public class LiftThenLeave extends CommandBase {
  private DriveSystem drive;
  private LiftSystem lift;
  private GripperSystem gripper;
  private AddressableLEDSubsystem aLEDSub;

  private Timer timer;
  private double time;

  /** Creates a new LiftThenLeave. */
  public LiftThenLeave(DriveSystem drive, LiftSystem lift, GripperSystem gripper) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, lift, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lift.liftArmsToPosition(LiftConstants.MID_POSITION);
    gripper.outtake();

    timer.start();
    drive.drivePercent(-0.1, -0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drivePercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}

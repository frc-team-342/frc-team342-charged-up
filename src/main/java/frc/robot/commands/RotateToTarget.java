// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.Limelight;

public class RotateToTarget extends CommandBase {
  private DriveSystem drive;
  private Limelight lime;
  
  /** Creates a new RotateToTarget. */
  public RotateToTarget(DriveSystem d, Limelight ll) {

    drive = d;
    lime = ll;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.rotateToAngle(lime.createRotation2D());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drivePercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

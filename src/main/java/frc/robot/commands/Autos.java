// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public final class Autos {
  /** Example static factory for an autonomous command. */

  public static CommandBase RotateThenDriveAuto(DriveSystem driveSubsystem) {

    return Commands.sequence(
    driveSubsystem.rotateToAngle(new Rotation2d(180)),
    driveSubsystem.driveDistance(Constants.AutoConstants.FAST_SPEED, 3.0)
    );
  }

  public static CommandBase DriveFast(DriveSystem driveSubsystem)
  {
    return driveSubsystem.driveDistance(Constants.AutoConstants.FAST_SPEED, 3.0);
  }

  public static CommandBase DriveSlow(DriveSystem driveSubsystem)
  {
    return driveSubsystem.driveDistance(Constants.AutoConstants.SLOW_SPEED, 3.0);
  }
  
  // needs testing 
  /** robot drives onto charge station and balances */

  public static CommandBase driveUpAndBalance(DriveSystem drivesystem) {
    return Commands.sequence(

      drivesystem.driveDistance(.5, 3),
      drivesystem.autoBalance());
      
  }

  

  /** robot drives onto charge station, balances, drives out of community, then back onto charge station and balances */

  public static CommandBase leftSide(DriveSystem drivesystem) {
    return Commands.sequence(

    drivesystem.driveDistance(.5, 3), 
    drivesystem.autoBalance(), 
    drivesystem.driveDistance(.5, 5),
    drivesystem.driveDistance(-.5,-5),
    new WaitCommand(1.5),
    drivesystem.autoBalance());

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

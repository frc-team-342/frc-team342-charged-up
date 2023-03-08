// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.DriveSystem;

/** Example static factory for an autonomous command. */
public final class Autos {

  public static CommandBase RotateThenDriveAuto(DriveSystem driveSubsystem) {
    return Commands.sequence();
  }

  public static CommandBase DriveFast(DriveSystem driveSubsystem) {
    return Commands.sequence();
  }

  public static CommandBase DriveSlow(DriveSystem driveSubsystem) {
    return Commands.sequence();
  }
  
  /** robot drives onto charge station and balances */
  public static CommandBase driveUpAndBalance(DriveSystem drivesystem) {
    return Commands.sequence();
  }

  /** robot drives onto charge station, balances, drives out of community, then back onto charge station and balances */
  public static CommandBase leftSide(DriveSystem drivesystem) {
    return Commands.sequence();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

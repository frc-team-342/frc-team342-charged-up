// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public final class Autos {
  /** Example static factory for an autonomous command. */

  public static CommandBase RotateThenDriveAuto(DriveSystem driveSubsystem) {
    return Commands.sequence();
  }

  public static CommandBase DriveFast(DriveSystem driveSubsystem) {
    return Commands.sequence();
  }

  public static CommandBase DriveSlow(DriveSystem driveSubsystem) {
    return Commands.sequence();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

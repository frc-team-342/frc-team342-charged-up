// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto() {
    return new InstantCommand();
  }
// needs testing 

// robot drives onto charge station and balances 
  public static CommandBase driveUpAndBalance(DriveSystem drivesystem) {
    return Commands.sequence(

      drivesystem.driveDistance(.5, 3),
      drivesystem.autoBalance());
      
  }

  //robot stays still

  public static CommandBase doNothing(DriveSystem drivesystem) {
    return Commands.sequence(drivesystem.driveDistance(0, 0));
  }

  //robot drives onto charge station, balances, drives out of community, then back onto charge station and balances

  public static CommandBase leftSide(DriveSystem drivesystem) {
    return Commands.sequence(

    drivesystem.driveDistance(.5, 3), 
    drivesystem.autoBalance(), 
    drivesystem.driveDistance(.5, 5),
    drivesystem.driveDistance(-.5,-5),
    drivesystem.autoBalance());

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

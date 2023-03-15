// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.commands.drive.DriveDistance;
import frc.robot.commands.drive.DriveVelocity;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.GripperSystem;
import frc.robot.subsystems.LiftSystem;

/** Example static factory for an autonomous command. */
public final class Autos {
  /** intake, lift arms, outtake, lower arms */
  private static CommandBase liftAndOuttake(LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led) {
    return Commands.sequence(
      // intake preloaded game piece
      gripper.hold().withTimeout(0.5),
      // run until either command finishes
      new ParallelRaceGroup(
        // hold preloaded game piece in gripper
        gripper.hold(),
        // lift arms to high scoring position
        lift.liftArmsToPosition(LiftConstants.TOP_POSITION)
        // cancel command group if not finished in x seconds
        //new WaitCommand(9)
      ),
      // outtake game piece
      gripper.outtake(led).withTimeout(0.8),
      // lower arms
      lift.liftArmsToPosition(LiftConstants.LOW_POSITION)
    );
  }

  public static CommandBase rotateThenDriveAuto(DriveSystem driveSubsystem) {
    return Commands.sequence(
      new RotateToAngle(new Rotation2d(180), driveSubsystem),
      new DriveDistance( 3.0, Constants.AutoConstants.FAST_SPEED, driveSubsystem)
    );
  }

  public static CommandBase liftThenLeave(LiftSystem lSystem, DriveSystem driveSystem) {
    return Commands.sequence(
      lSystem.liftArmsToPosition(LiftConstants.MID_POSITION), 
      new InstantCommand(
        () -> {
          driveSystem.drivePercent(-0.1, -0.1);
        }, driveSystem
      ).withTimeout(3)
    );
  }
  
  /** robot drives onto charge station and balances */
  public static CommandBase backUpAndBalance(DriveSystem drivesystem, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led) {
    return Commands.sequence(
      liftAndOuttake(lift, gripper, led),
      new DriveDistance(-1.5, 1.8, drivesystem).withTimeout(2), 
      drivesystem.autoBalance()
    );
      
  }

  /** robot drives onto charge station, balances, drives out of community, then back onto charge station and balances */
  public static CommandBase leftSideBlue(DriveSystem drivesystem, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led) {
    return Commands.sequence(
      liftAndOuttake(lift, gripper, led),
      new RotateToAngle(Rotation2d.fromDegrees(40), drivesystem).withTimeout(1),
      new DriveDistance(-0.5, 1, drivesystem), 
      new RotateToAngle(Rotation2d.fromDegrees(-40), drivesystem).withTimeout(1),
      new DriveDistance(-1.7, 1.3, drivesystem), 
      new WaitCommand(1.5)
    );
  }

  /** robot drives onto charge station, balances, drives out of community, then back onto charge station and balances */
  public static CommandBase rightSideBlue(DriveSystem drivesystem, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led) {
    return Commands.sequence(
      liftAndOuttake(lift, gripper, led),
      new RotateToAngle(Rotation2d.fromDegrees(-40), drivesystem).withTimeout(1),
      new DriveDistance(-0.5, 1, drivesystem), 
      new RotateToAngle(Rotation2d.fromDegrees(40), drivesystem).withTimeout(1),
      new DriveDistance(-3.05, 1.5, drivesystem), 
      new WaitCommand(1.5)
    );
  }

  public static CommandBase leftSideRed(DriveSystem drive, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led) {
    return Commands.sequence(
      liftAndOuttake(lift, gripper, led),
      new RotateToAngle(Rotation2d.fromDegrees(40), drive).withTimeout(1),
      new DriveDistance(-0.5, 1, drive), 
      new RotateToAngle(Rotation2d.fromDegrees(-40), drive).withTimeout(1),
      new DriveDistance(-3.05, 1.5, drive), 
      new WaitCommand(1.5)
    );
  }

  public static CommandBase rightSideRed(DriveSystem drive, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led) {
    return Commands.sequence(
      liftAndOuttake(lift, gripper, led),
      new RotateToAngle(Rotation2d.fromDegrees(-40), drive).withTimeout(1),
      new DriveDistance(-0.5, 1, drive), 
      new RotateToAngle(Rotation2d.fromDegrees(40), drive).withTimeout(1),
      new DriveDistance(-1.7, 1.3, drive), 
      new WaitCommand(1.5)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

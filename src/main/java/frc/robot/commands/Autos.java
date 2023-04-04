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
        lift.liftArmsToPosition(LiftConstants.TOP_POSITION),
        // cancel command group if not finished in x seconds
        new WaitCommand(4)
      ),
      // outtake game piece
      gripper.outtake(led).withTimeout(0.8),
      // lower arms
      lift.liftArmsToPosition(LiftConstants.LOW_POSITION)
    );
  }

  /** Additional part of auto that turns the robot around and attempts to intake a game piece */
  public static CommandBase rotateThenDriveAuto(DriveSystem driveSubsystem, GripperSystem gripper, AddressableLEDSubsystem led) {
    return Commands.sequence(
      new RotateToAngle(Rotation2d.fromDegrees(182), driveSubsystem).withTimeout(2.5),
      new ParallelCommandGroup(
        new DriveDistance(1.8, Constants.AutoConstants.FAST_SPEED, driveSubsystem).withTimeout(2),
        gripper.coneIntake(led).withTimeout(3)
      )
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

    /** robot scores low then drives onto charge station and balances */
    public static CommandBase outtakeAndBalance(DriveSystem drivesystem, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led) {
      return Commands.sequence(
        gripper.outtake(led).withTimeout(0.5),
        new DriveDistance(-1.5, 1.8, drivesystem).withTimeout(2), 
        drivesystem.autoBalance()
      );
        
    }

  /** robot drives onto charge station, balances, drives out of community, then back onto charge station and balances */
  public static CommandBase leftSideBlue(DriveSystem drivesystem, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led, Boolean additionalAuto) {
    return Commands.sequence(
      liftAndOuttake(lift, gripper, led),
      new RotateToAngle(Rotation2d.fromDegrees(40), drivesystem).withTimeout(1),
      new DriveDistance(-0.5, 1, drivesystem), 
      new RotateToAngle(Rotation2d.fromDegrees(-40), drivesystem).withTimeout(1),
      new DriveDistance(-2.2, 1.3, drivesystem),
  
      // Checks if parameter was set to enable the additional auto. If not, proceed as normal
      additionalAuto ? rotateThenDriveAuto(drivesystem, gripper, led) : new WaitCommand(0.1)
    );
  }

  /** robot drives onto charge station, balances, drives out of community, then back onto charge station and balances */
  public static CommandBase rightSideBlue(DriveSystem drivesystem, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led, Boolean additionalAuto) {
    return Commands.sequence(
      liftAndOuttake(lift, gripper, led),
      new RotateToAngle(Rotation2d.fromDegrees(-40), drivesystem).withTimeout(1),
      new DriveDistance(-0.5, 1, drivesystem), 
      new RotateToAngle(Rotation2d.fromDegrees(40), drivesystem).withTimeout(1),
      new DriveDistance(-3.2, 1.5, drivesystem),

      // Checks if parameter was set to enable the additional auto. If not, proceed as normal
      additionalAuto ? rotateThenDriveAuto(drivesystem, gripper, led) : new WaitCommand(0.1)
    );
  }

  public static CommandBase leftSideRed(DriveSystem drive, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led, Boolean additionalAuto) {
    return Commands.sequence(
      liftAndOuttake(lift, gripper, led),
      new RotateToAngle(Rotation2d.fromDegrees(40), drive).withTimeout(1),
      new DriveDistance(-0.5, 1, drive), 
      new RotateToAngle(Rotation2d.fromDegrees(-40), drive).withTimeout(1),
      new DriveDistance(-3.2, 1.5, drive), 

      // Checks if parameter was set to enable the additional auto. If not, proceed as normal
      additionalAuto ? rotateThenDriveAuto(drive, gripper, led) : new WaitCommand(0.1)
    );
  }

  public static CommandBase rightSideRed(DriveSystem drive, LiftSystem lift, GripperSystem gripper, AddressableLEDSubsystem led, Boolean additionalAuto) {
    return Commands.sequence(
      liftAndOuttake(lift, gripper, led),
      new RotateToAngle(Rotation2d.fromDegrees(-40), drive).withTimeout(1),
      new DriveDistance(-0.5, 1, drive), 
      new RotateToAngle(Rotation2d.fromDegrees(40), drive).withTimeout(1),
      new DriveDistance(-1.7, 1.3, drive),
      
      // Checks if parameter was set to enable the additional auto. If not, proceed as normal
      additionalAuto ? rotateThenDriveAuto(drive, gripper, led) : new WaitCommand(0.1)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

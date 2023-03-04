// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

import static frc.robot.Constants.DriveConstants.*;

public class RotateToAngle extends CommandBase {
  private final DriveSystem drive;
  private final PIDController rotateController;

  private Rotation2d start;
  private Rotation2d end;

  private final Rotation2d angle;

  /** Creates a new RotateToAngle. */
  public RotateToAngle(Rotation2d angle, DriveSystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);
    
    this.angle = angle;

    rotateController = new PIDController(
      ROTATION_P, 
      ROTATION_I, 
      ROTATION_D
    );
    
    // set position tolerance for pid
    rotateController.setTolerance(ROTATION_TOLERANCE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set initial position
    start = drive.getGyroAngle();

    // set end rotation based on start
    end = start.plus(angle);

    // reset rotation pid controller
    rotateController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get current angle
    Rotation2d current = drive.getGyroAngle();

    // rad/s ??
    double rotationVel = rotateController.calculate(current.getRadians(), end.getRadians());

    // convert radial velocity to drive speeds
    ChassisSpeeds radial = new ChassisSpeeds(0, 0, -rotationVel);
    DifferentialDriveWheelSpeeds speeds = drive.inverseKinematics(radial);

    // clamp wheel speeds
    speeds.desaturate(MAX_SPEED);

    // set drivetrain speeds
    drive.setVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop motors
    drive.setVelocity(new DifferentialDriveWheelSpeeds(0, 0));
    drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotateController.atSetpoint();
  }
}

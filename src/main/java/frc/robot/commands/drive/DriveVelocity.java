// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

import static frc.robot.Constants.DriveConstants.*;

public class DriveVelocity extends CommandBase {
  // subsystem
  private final DriveSystem drive;

  /** used to drive straight */
  private final PIDController rotationController;
  
  /** drive velocity */
  private final double velocity;

  /** starting angle as of command init */
  private Rotation2d initial;

  /** Creates a new DriveVelocity. */
  public DriveVelocity(double velocity, DriveSystem drive) {
    this.drive = drive;
    addRequirements(this.drive);

    rotationController = new PIDController(
      ROTATION_P, 
      ROTATION_I, 
      ROTATION_D
    );

    // clamp velocity
    this.velocity = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set initial position at command start
    initial = drive.getGyroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get current angle and find rotational error
    Rotation2d current = drive.getGyroAngle();
    Rotation2d error = initial.minus(current);
    
    // calculate drive effort to stay straight
    double rotation = rotationController.calculate(error.getRadians(), 0);

    // set drive speeds using rotation pid
    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(
      velocity - rotation,
      velocity + rotation
    );
    
    // clamp speeds
    speeds.desaturate(MAX_SPEED);

    this.drive.setVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop drive motors
    drive.setVelocity(new DifferentialDriveWheelSpeeds(0, 0));
    drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

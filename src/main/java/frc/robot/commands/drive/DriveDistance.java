// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

import static frc.robot.Constants.DriveConstants.*;

public class DriveDistance extends CommandBase {
  private final double distance;
  private final double velocity;

  private final DriveSystem drive;
  private final PIDController rotationController;

  private Pose2d start;
  private Pose2d end;

  /** Creates a new DriveDistance. */
  public DriveDistance(double distance, double velocityIn, DriveSystem drive) {
    this.distance = distance;

    // determine wheel velocity based on whether dist is forwards or backwards
    this.velocity = (distance <= 0)
      ? MathUtil.clamp(Math.abs(velocityIn), 0.0, MAX_SPEED)
      : -Math.abs(MathUtil.clamp(velocityIn, -MAX_SPEED, MAX_SPEED));

    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);

    rotationController = new PIDController(
      ROTATION_P, 
      ROTATION_I, 
      ROTATION_D
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set initial position on command start
    start = drive.getOdometryPosition();

    // set final position based on initial position
    // TODO: ???
    end = start.transformBy(
      new Transform2d(
        new Translation2d(distance, start.getRotation()),
        new Rotation2d()
      )
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // find current angle and rotational error
    Rotation2d current = drive.getGyroAngle();
    Rotation2d error = current.minus(start.getRotation());

    // use rotation controller to drive straight
    double rotation = rotationController.calculate(error.getRadians(), 0);

    // wheel speeds using rotation
    var speeds = new DifferentialDriveWheelSpeeds(
      velocity + rotation,
      velocity - rotation
    );

    // clamp wheel speeds
    speeds.desaturate(MAX_SPEED);

    // drive at determined velocity
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
    // get current position
    Pose2d current = drive.getOdometryPosition();
    
    // find difference between current and end positions
    double diff = current.getTranslation().getDistance(start.getTranslation());

    // check if distance traveled is within tolerance
    double absDist = Math.abs(distance);
    return diff > (absDist - DISTANCE_TOLERANCE) && diff < (absDist + DISTANCE_TOLERANCE);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Current Position", () -> {
      // get current position
      Pose2d current = drive.getOdometryPosition();

      // current position formatted as coordinates
      String currPos = String.format("(%f, %f)", current.getX(), current.getY());
      return currPos;
    }, null);
    
    builder.addStringProperty(
      "End Position", 
      () -> {
        // end position formatted as coordinates
        String endPos = String.format("(%f, %f)", end.getX(), end.getY());
        return endPos;
      }, 
      null
    );
  }
}

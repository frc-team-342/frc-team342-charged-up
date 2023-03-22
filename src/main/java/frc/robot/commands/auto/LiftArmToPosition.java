// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.LiftSystem;
import frc.robot.Constants.*;

public class LiftArmToPosition extends CommandBase {

  private LiftSystem liftSystem;
  private SparkMaxPIDController pControllerOne;
  private SparkMaxPIDController pControllerTwo;

  private boolean areWeThereYet;
  private double desPosition;
  private double holdPositionOne;
  private double holdPositionTwo;

  /** Creates a new LiftArmToPosition. */
  public LiftArmToPosition(LiftSystem lSystem, double desiredPosition) {
    
    liftSystem = lSystem;
    desPosition = MathUtil.clamp(desiredPosition, LiftConstants.MAX_POSITION, LiftConstants.MIN_POSITION);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    areWeThereYet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if ((liftSystem.getPosition() < desPosition + LiftConstants.TOLERANCE) && (liftSystem.getPosition() > desPosition - LiftConstants.TOLERANCE)){
      if (!areWeThereYet) {
        areWeThereYet = !areWeThereYet;
        holdPositionOne = liftSystem.getMotorOnePos();
        holdPositionTwo = liftSystem.getMotorTwoPos();
      }
      liftSystem.setMotorPIDPosition(holdPositionOne, holdPositionTwo);
    }
    //If the current position is lower than the desired position, move the arm up
    else if(desPosition - liftSystem.getPosition() < 0){
      liftSystem.setMotorPIDVelocity(LiftConstants.AUTO_SPEED);
    }
    //If higher than desired position, move the arm down
    else {
      liftSystem.setMotorPIDVelocity(-LiftConstants.AUTO_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    areWeThereYet = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (liftSystem.getPosition() < desPosition + LiftConstants.TOLERANCE) && (liftSystem.getPosition() > desPosition - LiftConstants.TOLERANCE) && areWeThereYet;
  }
}

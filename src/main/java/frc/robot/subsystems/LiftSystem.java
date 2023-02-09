// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LiftConstants;

import static frc.robot.Constants.LiftConstants.*;

public class LiftSystem extends SubsystemBase {

  private final CANSparkMax motorOne;
  private final CANSparkMax motorTwo;

  private final RelativeEncoder encoder;

  private final MotorControllerGroup liftGroup;
  private SparkMaxPIDController pControllerOne;
  private SparkMaxPIDController pControllerTwo;
  
    /** Creates a new LiftSystem. */
  public LiftSystem() {

    motorOne = new CANSparkMax(LiftConstants.MOTOR_LEFT, MotorType.kBrushless);

    motorTwo = new CANSparkMax(LiftConstants.MOTOR_RIGHT, MotorType.kBrushless);
    motorTwo.setInverted(true);

    encoder = motorTwo.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

    liftGroup = new MotorControllerGroup(motorOne, motorTwo);

    //Setting default values for PID
    pControllerOne = motorOne.getPIDController();
    pControllerOne.setP(0);
    pControllerOne.setD(0);
    pControllerOne.setFF(0);

    pControllerTwo = motorTwo.getPIDController();
    pControllerTwo.setP(0);
    pControllerTwo.setD(0);
    pControllerTwo.setFF(0);


  }

  public void liftArms(double speed){
    liftGroup.set(speed);
  }

  /**
   * @param position to go to
   * @return Command that uses PID to lift the gripper to the specified position
   */
  public CommandBase liftArmsToPosition(double position){
    
    double clampedPos = MathUtil.clamp(position, MIN_POSITION, MAX_POSITION);

    return runEnd(
    //Runs repeatedly until the end
    () -> {
      pControllerOne.setReference(clampedPos, ControlType.kPosition);
      pControllerTwo.setReference(clampedPos, ControlType.kPosition);
      System.out.println("Inside lift command reached");
    }, 
    //Runs when command ends
    () -> {
      liftArms(0);
    }
    ).until(
      () -> {
        return (getEncoderVal() > clampedPos && getEncoderVal() < clampedPos);
      }
    );
  }

  public double getEncoderVal(){
    return encoder.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Lift Encoder Value", this::getEncoderVal, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

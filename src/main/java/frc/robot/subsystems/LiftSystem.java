// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

import static frc.robot.Constants.LiftConstants.*;

import java.util.List;

public class LiftSystem extends SubsystemBase implements Testable {

  private final CANSparkMax motorOne;
  private final CANSparkMax motorTwo;

  private final MotorControllerGroup liftGroup;

  private final PIDController pControllerOne;
  private final PIDController pControllerTwo;
  
  //private final DutyCycleEncoder dInput;
  private final SparkMaxAbsoluteEncoder absEncoder;
  
    /** Creates a new LiftSystem. */
  public LiftSystem() {

    motorOne = new CANSparkMax(MOTOR_LEFT, MotorType.kBrushless);

    motorTwo = new CANSparkMax(MOTOR_RIGHT, MotorType.kBrushless);
    motorTwo.setInverted(true);

    //dInput = new DutyCycleEncoder(0);
    absEncoder = motorTwo.getAbsoluteEncoder(Type.kDutyCycle);
  
    liftGroup = new MotorControllerGroup(motorOne, motorTwo);

    //Setting default values for PID
    pControllerOne = new PIDController(0.25, 0.0, 0.0);
    pControllerTwo = new PIDController(0.25, 0.0, 0.0);
  }

  public CommandBase liftArms(double speed){

    return runEnd(
      () -> {
        liftGroup.set(speed);
      },

      () -> {
        liftGroup.set(0);
      }
    );
  }

  /**
   * @param position to go to
   * @return Command that uses PID to lift the gripper to the specified position
   */
  public CommandBase liftArmsToPosition(double position){

    double clampedPos = MathUtil.clamp(position, MIN_POSITION, MAX_POSITION);
    SmartDashboard.putNumber("setpoint", clampedPos);

    return runEnd(
    //Runs repeatedly until the end
    () -> {
      
      motorOne.set(pControllerOne.calculate(getPosition(), clampedPos));
      motorTwo.set(pControllerTwo.calculate(getPosition(), clampedPos));

      System.out.println("Inside lift command reached");
    }, 
    //Runs when command ends
    () -> {
      liftArms(0);
    }
    ).until(
      () -> {
        return (getPosition() > clampedPos && getPosition() < clampedPos);
      }
    );
  }


  public double getPosition(){
    return absEncoder.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Encoder", this::getPosition, null);
    builder.addDoubleProperty("motor velocity", motorOne.getEncoder()::getVelocity, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public List<Connection> hardwareConnections(){
    return List.of(
      Connection.fromSparkMax(motorOne),
      Connection.fromSparkMax(motorTwo)
    );
  }

}

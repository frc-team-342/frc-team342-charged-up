// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
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

  private final SparkMaxPIDController pControllerOne;
  private final SparkMaxPIDController pControllerTwo;

  private final DigitalInput limitUp;
  private final DigitalInput limitDown;
  
  private final DutyCycleEncoder armEncoder;
  private final RelativeEncoder motorEncoder;
  
  /** Creates a new LiftSystem. */
  public LiftSystem() {
    motorOne = new CANSparkMax(MOTOR_LEFT, MotorType.kBrushless);

    motorTwo = new CANSparkMax(MOTOR_RIGHT, MotorType.kBrushless);
    motorTwo.setInverted(true);

    armEncoder = new DutyCycleEncoder(0);
    motorEncoder = motorOne.getEncoder();
  
    liftGroup = new MotorControllerGroup(motorOne, motorTwo);

    //Setting default values for PID
    pControllerOne = motorOne.getPIDController();
    pControllerOne.setP(0.001);
    pControllerOne.setD(0.0001);
    pControllerOne.setFF(1);

    pControllerTwo = motorTwo.getPIDController();
    pControllerTwo.setP(0.001);
    pControllerTwo.setD(0.0001);
    pControllerTwo.setFF(1);

    //Limit Switches
    limitUp = new DigitalInput(LIMIT_SWITCH_UP);
    limitDown = new DigitalInput(LIMIT_SWITCH_DOWN);

    motorOne.setSmartCurrentLimit(CURRENT_LIMIT);
    motorTwo.setSmartCurrentLimit(CURRENT_LIMIT);
  }

  /**
   * @param xboxController Operator
   * @return Command that uses obtained values from the limit switch to lift the arm within a specified range.
   */
  public CommandBase liftArms(XboxController xboxController){
    
    return runEnd(
      () -> {
        double setPoint = -xboxController.getLeftY() * MAX_SPEED;

        if(limitUp.get() && (xboxController.getLeftY() > 0)) { 
          // When the upward limit switch is triggered and the operator attempts to move upward, it will not move upward.
          liftGroup.set(0);
        } else if(limitDown.get() && (xboxController.getLeftY() < 0)) { 
          // When the downward limit switch is triggered and the operator attempts to move downward, it will not move downward.
          liftGroup.set(0);
        } else {
          pControllerOne.setReference(setPoint, ControlType.kVelocity);
          pControllerTwo.setReference(setPoint, ControlType.kVelocity);
        }
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
  public CommandBase liftArmsToPosition(double desiredPosition){
    double clampedPos = MathUtil.clamp(desiredPosition, MIN_POSITION, MAX_POSITION);
    return runEnd(
    //Runs repeatedly until the end
    () -> {
      
      SmartDashboard.putNumber("setpoint", clampedPos);

      pControllerOne.setReference(clampedPos, CANSparkMax.ControlType.kPosition);
      pControllerTwo.setReference(clampedPos, CANSparkMax.ControlType.kPosition);

      System.out.println("Inside lift command reached");
    }, 
    //Runs when command ends
    () -> {
      //liftArms(0);
    }
    ).until(
      () -> {
        return (getPosition() > clampedPos && getPosition() < clampedPos);
      }
    );
  }


  public double getPosition(){
    return armEncoder.getAbsolutePosition();
  }

  /**
   * @param mode If false, set the motor's idle mode to coast. If true, set the idle mode to brake.
   */
  public void setBrakeMode(boolean mode){
    motorOne.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
    motorTwo.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Through-bore encoder position", this::getPosition, null);
  
    builder.addDoubleProperty("Motor encoder position", motorEncoder::getPosition, null);
    builder.addDoubleProperty("Motor encoder velocity", motorEncoder::getVelocity, null);

    builder.addBooleanProperty("Up Limit Switch", () -> !limitDown.get(), null);
    builder.addBooleanProperty("Down Limit Switch", () -> !limitUp.get(), null);
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

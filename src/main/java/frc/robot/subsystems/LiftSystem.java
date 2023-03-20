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

  private final SparkMaxPIDController pControllerOne;
  private final SparkMaxPIDController pControllerTwo;

  private final DigitalInput limitUp;
  private final DigitalInput limitDown;
  
  private final DutyCycleEncoder armEncoder;
  private final RelativeEncoder motorOneEncoder;
  private final RelativeEncoder motorTwoEncoder;

<<<<<<< Updated upstream
=======
  private double posSetpointOne = 0;
  private double posSetpointTwo = 0;
  private double velSetpoint = 0;

>>>>>>> Stashed changes
  /** Creates a new LiftSystem. */
  public LiftSystem() {
    motorOne = new CANSparkMax(MOTOR_LEFT, MotorType.kBrushless);

    motorTwo = new CANSparkMax(MOTOR_RIGHT, MotorType.kBrushless);
    motorTwo.setInverted(true);

    armEncoder = new DutyCycleEncoder(ARM_ENCODER_PORT);
    motorOneEncoder = motorOne.getEncoder();
    motorTwoEncoder = motorTwo.getEncoder();
  
    //Setting default values for PID
    pControllerOne = motorOne.getPIDController();
    pControllerOne.setP(P_VALUE, 1);
    pControllerOne.setD(D_VALUE, 1);
    pControllerOne.setFF(FF_VALUE, 1);

    pControllerTwo = motorTwo.getPIDController();
    pControllerTwo.setP(P_VALUE, 1);
    pControllerTwo.setD(D_VALUE, 1);
    pControllerTwo.setFF(FF_VALUE, 1);

    //Limit Switches
    limitUp = new DigitalInput(LIMIT_SWITCH_UP);
    limitDown = new DigitalInput(LIMIT_SWITCH_DOWN);

    motorOne.setSmartCurrentLimit(CURRENT_LIMIT);
    motorTwo.setSmartCurrentLimit(CURRENT_LIMIT);
<<<<<<< Updated upstream
=======

    pControllerOne.setP(0.01275,2);
    pControllerOne.setD(0.00625, 2);
    pControllerOne.setFF(1.0, 2);
    
    pControllerTwo.setP(0.01275, 2);
    pControllerTwo.setD(0.00625, 2);
    pControllerTwo.setFF(1.0, 2);
>>>>>>> Stashed changes
  }

  /**
   * @param xboxController Operator
   * @return Command that uses obtained values from the limit switch to lift the arm within a specified range.
   */
  public CommandBase liftArms(XboxController xboxController){
    
    return runEnd(
      () -> {
        double setPoint = -xboxController.getLeftY() * MAX_SPEED; // Percent output 

        if(!limitUp.get() && (xboxController.getLeftY() > 0)) { 
          // When the upward limit switch is triggered and the operator attempts to move upward, it will not move upward.
          pControllerOne.setReference(0, ControlType.kVelocity, 1);
          pControllerTwo.setReference(0, ControlType.kVelocity, 1);
        } else if(!limitDown.get() && (xboxController.getLeftY() < 0)) { 
          // When the downward limit switch is triggered and the operator attempts to move downward, it will not move downward.
          pControllerOne.setReference(0, ControlType.kVelocity, 1);
          pControllerTwo.setReference(0, ControlType.kVelocity, 1);
        } else {
          pControllerOne.setReference(setPoint, ControlType.kVelocity, 1);
          pControllerTwo.setReference(setPoint, ControlType.kVelocity, 1);
        }
      },

      () -> {
        pControllerOne.setReference(0, ControlType.kVelocity, 1);
        pControllerTwo.setReference(0, ControlType.kVelocity, 1);
      }
    );
  }

  /**
   * @param Absolute position to lift to
   * @return Command that uses PID to lift the gripper to the specified position
   */
  public CommandBase liftArmsToPosition(double desiredPosition){
    double clampedPos = MathUtil.clamp(desiredPosition, MAX_POSITION, MIN_POSITION);

    return runEnd(
    //Runs repeatedly until the end
    () -> {
      //If position is within the range of desired position, stop there
      if ((getPosition() < clampedPos + LiftConstants.TOLERANCE) && (getPosition() > clampedPos - LiftConstants.TOLERANCE)){
        pControllerOne.setReference(0, ControlType.kVelocity, 1);
        pControllerTwo.setReference(0, ControlType.kVelocity, 1);
      }
      //If the current position is lower than the desired position, move the arm up
      else if(clampedPos - getPosition() < 0){
<<<<<<< Updated upstream
        pControllerOne.setReference(0.25, ControlType.kVelocity);
        pControllerTwo.setReference(0.25,ControlType.kVelocity);
      }
      //If higher than desired position, move the arm down
      else {
        pControllerOne.setReference(-0.25, ControlType.kVelocity);
        pControllerTwo.setReference(-0.25,ControlType.kVelocity);
=======
        pControllerOne.setReference(LiftConstants.AUTO_SPEED, ControlType.kVelocity, 1);
        pControllerTwo.setReference(LiftConstants.AUTO_SPEED,ControlType.kVelocity, 1);
      }
      //If higher than desired position, move the arm down
      else {
        pControllerOne.setReference(-LiftConstants.AUTO_SPEED, ControlType.kVelocity, 1);
        pControllerTwo.setReference(-LiftConstants.AUTO_SPEED, ControlType.kVelocity, 1);
>>>>>>> Stashed changes
      }
    }, 
    //Runs when command ends
    () -> {
        pControllerOne.setReference(0, ControlType.kVelocity, 1);
        pControllerTwo.setReference(0, ControlType.kVelocity, 1);
      }
    ).until(
      () -> {
        return (getPosition() < clampedPos + LiftConstants.TOLERANCE) && (getPosition() > clampedPos - LiftConstants.TOLERANCE);
      }
    );
  }


  /**
   * @return absolute position from 0 to 1
   */
  public double getPosition(){
    return armEncoder.getAbsolutePosition();
  }

  public double getMotorOnePos(){
    return motorOneEncoder.getPosition();
  }

  public double getMotorTwoPos(){
    return motorTwoEncoder.getPosition();
  }

  public void setMotorPIDVelocity(double referenceValue){
    velSetpoint = referenceValue;
    pControllerOne.setReference(referenceValue, ControlType.kVelocity, 1);
    pControllerTwo.setReference(referenceValue, ControlType.kVelocity, 1);
  }

  public void setMotorPIDPosition(double referenceValueOne, double referenceValueTwo){
    posSetpointOne = referenceValueOne;
    posSetpointTwo = referenceValueTwo;

    pControllerOne.setReference(referenceValueOne, ControlType.kPosition, 2);
    pControllerTwo.setReference(referenceValueTwo, ControlType.kPosition, 2);
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
  
    builder.addDoubleProperty("Motor one encoder velocity", motorOneEncoder::getVelocity, null);
    builder.addDoubleProperty("Motor two encoder velocity", motorTwoEncoder::getVelocity, null);

    builder.addDoubleProperty("Motor one encoder position", motorOneEncoder::getPosition, null);
    builder.addDoubleProperty("Motor two encoder position", motorTwoEncoder::getPosition, null);

    builder.addBooleanProperty("Up Limit Switch", () -> !limitDown.get(), null);
    builder.addBooleanProperty("Down Limit Switch", () -> !limitUp.get(), null);

    builder.addDoubleProperty("Velocity setpoint", () -> velSetpoint, null);
    builder.addDoubleProperty("Left position setpoint", () -> posSetpointOne, null);
    builder.addDoubleProperty("Right position setpoint", () -> posSetpointTwo, null);
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

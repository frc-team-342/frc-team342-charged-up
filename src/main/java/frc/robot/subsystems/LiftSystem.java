// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
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

  private final PIDController pControllerOne;
  private final PIDController pControllerTwo;

  private final DigitalInput limitUp;
  private final DigitalInput limitDown;
  
  private final DutyCycleEncoder armEncoder;
  
    /** Creates a new LiftSystem. */
  public LiftSystem() {

    motorOne = new CANSparkMax(MOTOR_LEFT, MotorType.kBrushless);

    motorTwo = new CANSparkMax(MOTOR_RIGHT, MotorType.kBrushless);
    motorTwo.setInverted(true);

    armEncoder = new DutyCycleEncoder(0);
  
    liftGroup = new MotorControllerGroup(motorOne, motorTwo);

    //Setting default values for PID
    pControllerOne = new PIDController(0.25, 0.0, 0.0);
    pControllerTwo = new PIDController(0.25, 0.0, 0.0);

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
        if(limitUp.get() && (xboxController.getLeftY() > 0)) { // When the upward limit switch is triggered and the operator attempts to move upward, it will not move upward.
          liftGroup.set(0);
        } else if(limitDown.get() && (xboxController.getLeftY() < 0)) { // When the downward limit switch is triggered and the operator attempts to move downward, it will not move downward.
          liftGroup.set(0);
        } else {
          liftGroup.set(-0.2 * xboxController.getLeftY());
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
    builder.addDoubleProperty("Encoder", this::getPosition, null);
    builder.addDoubleProperty("motor velocity", motorOne.getEncoder()::getVelocity, null);

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

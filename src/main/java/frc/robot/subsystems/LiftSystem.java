// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSystem extends SubsystemBase {

  private final CANSparkMax motorOne;
  private final CANSparkMax motorTwo;
  
  private final SparkMaxAbsoluteEncoder encoder;

  private final MotorControllerGroup liftGroup;
  
    /** Creates a new LiftSystem. */
  public LiftSystem() {

    motorOne = new CANSparkMax(1, MotorType.kBrushless);
    motorTwo = new CANSparkMax(2, MotorType.kBrushless);
   
    encoder = motorOne.getAbsoluteEncoder(Type.kDutyCycle);

    liftGroup = new MotorControllerGroup(motorOne, motorTwo);

  }

  public void liftArms(Double speed){
    liftGroup.set(speed);
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

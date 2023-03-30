// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.AddressableLEDSubsystem.ColorType;

import static frc.robot.Constants.GripperConstants.*;

import frc.robot.Limelight;

public class GripperSystem extends SubsystemBase {

  //controls the speed of the spinning wheels
  //private final ColorSensorV3 colorSensor;
  // controls the speed of the spinning wheels
  private CANSparkMax rollerMotor;
  private Limelight limelight;
  private boolean isHolding;
  private RelativeEncoder encoder;

  private double lastPosition;

  /** Creates a new GripperSystem. */
  public GripperSystem(Limelight limelight) {
    //colorSensor = new ColorSensorV3(GripperConstants.I2C_PORT);
    rollerMotor = new CANSparkMax(ROLLER_MOTOR, MotorType.kBrushless);
    this.limelight = limelight;
    rollerMotor.setSmartCurrentLimit(ROLLER_MOTOR_CURRENT_LIMIT_VALUE);
    rollerMotor.setInverted(true);
    isHolding = true;
    encoder = rollerMotor.getEncoder();

  }

  public void spin(double speed) {
    rollerMotor.set(speed);
  }

  public CommandBase coneIntake() {
    return runEnd(
      // run
      () -> {
        if (rollerMotor.getOutputCurrent() < MAX_CUBE_DRAW)
        {
          spin(ROLLER_SPEED);
        }
        else
        {
          spin(0);
        }
      },

      // end
      () -> {
        spin(0);
        isHolding = true;
      });
  }

  public CommandBase cubeIntake(){
    return runEnd(
      // run
      () -> {
        if (rollerMotor.getOutputCurrent() < MAX_CUBE_DRAW)
        {
          spin(ROLLER_SPEED);
        }
        else
        {
          spin(0);
        }
      },

      // end
      () -> {
        spin(0);
        isHolding = true;
      });
  }

  public CommandBase hold(AddressableLEDSubsystem aLedSubsystem) {
    return runEnd(
      //run
      () -> {
          if(isHolding){
            spin(0.15);
            if(rollerMotor.getEncoder().getPosition() <= (lastPosition + 5))
            {
              aLedSubsystem.DriverColor(ColorType.RED);
            }
          }else{
            spin(0);
          }
        },
        // end
        () -> {
          spin(0);
        });
  }

  /**
   * spins the gripper roller at a negative speed to outtake
   * sets speed to 0 to stop
   **/
  public CommandBase outtake() {
    return runEnd(
        // run
        () -> {

          spin(-(ROLLER_SPEED));

        },
        // end
        () -> {
          spin(0);
          isHolding = false;
        });
  }

  public boolean getIsHolding()
  {
    return isHolding;
  }

  public double getLastPosition()
  {
    return lastPosition;
  }

  public RelativeEncoder getEncoder()
  {
    return encoder;
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.setSmartDashboardType("GripperSystem");
    builder.addDoubleProperty("Current Draw Readings", () -> rollerMotor.getOutputCurrent(), null);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lastPosition = rollerMotor.getEncoder().getPosition();
    System.out.println(lastPosition);
  }
}

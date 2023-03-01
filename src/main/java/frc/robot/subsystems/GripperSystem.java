// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.AddressableLEDSubsystem.ColorType;

import static frc.robot.Constants.GripperConstants.*;
import frc.robot.Limelight;



public class GripperSystem extends SubsystemBase {

  //controls the speed of the spinning wheels
  //private final ColorSensorV3 colorSensor;
  private CANSparkMax rollerMotor;
  private Limelight limelight;
  private final AddressableLEDSubsystem aLedSubsystem;

  /** Creates a new GripperSystem. */
  public GripperSystem(Limelight limelight) {
    //colorSensor = new ColorSensorV3(GripperConstants.I2C_PORT);
    rollerMotor = new CANSparkMax(ROLLER_MOTOR, MotorType.kBrushless);
    this.limelight = limelight;
    aLedSubsystem = new AddressableLEDSubsystem();
  }

  public void spin(double speed){
    rollerMotor.set(speed);
  }

  /**
   * Spins the gripper roller to intake
   * sets speed to 0 to stop
   **/
  public CommandBase intake(){
    return runEnd(
      //run
      () -> {
        System.out.println("Trying to spin");
        spin(ROLLER_SPEED);
      },
      //end
      () -> {
        spin(0);
        
        /** This logic changes the vision mode depending on whatever game piece has been grabbed */
        if(checkForCube()) {
          limelight.setPipeline(1);
          aLedSubsystem.DriverColor(ColorType.PURPLE);
        } else if(checkForGamePiece()) {
          limelight.setPipeline(0);
          aLedSubsystem.DriverColor(ColorType.YELLOW);
        }

      }

    );
  }


  private boolean checkForGamePiece() {
    return false;
    //return colorSensor.getIR() > GripperConstants.GAME_PIECE_IR_MINIMUM;
  }

  private boolean checkForCube() {
    return false;
    //return checkForGamePiece() && colorSensor.getColor().blue > MINIMUM_BLUE_VALUE_FOR_CUBE;
  }

  /**
   * spins the gripper roller at a negative speed to outtake
   * sets speed to 0 to stop
   **/
  public CommandBase outtake(){
    return runEnd(
      //run
      () -> {
      
        spin(-(ROLLER_SPEED));

      },
      //end
      () -> {
        spin(0);
      }
    );
  }

  @Override
  public void initSendable(SendableBuilder builder){
  
    builder.setSmartDashboardType("GripperSystem");
/*
    builder.addDoubleProperty("Red", () -> colorSensor.getColor().red, null);
    builder.addDoubleProperty("Green", () -> colorSensor.getColor().green, null);
    builder.addDoubleProperty("Blue", () -> colorSensor.getColor().blue, null);
    builder.addDoubleProperty("IR", () -> colorSensor.getIR(), null);
    builder.addDoubleProperty("Proximity", () -> colorSensor.getProximity(), null);
    builder.addBooleanProperty("Cube picked up?", () -> checkForCube(), null);
    builder.addBooleanProperty("Game piece picked up?", () -> checkForGamePiece(), null);
    builder.addDoubleProperty("Current Draw Readings", () -> rollerMotor.getOutputCurrent(), null);
*/
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

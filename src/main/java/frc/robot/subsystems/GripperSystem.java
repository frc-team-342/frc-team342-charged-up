// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;
import static frc.robot.Constants.GripperConstants.*;

import frc.robot.Limelight;



public class GripperSystem extends SubsystemBase {

  //controls the speed of the spinning wheels
<<<<<<< HEAD
  //private final ColorSensorV3 colorSensor;
=======
>>>>>>> 042675701f34cb5cc47cd84e517b933d9bb4bfbb
  private CANSparkMax rollerMotor;
  private Limelight limelight;
  private boolean isHolding;

  /** Creates a new GripperSystem. */
  public GripperSystem(Limelight limelight) {
<<<<<<< HEAD
    //colorSensor = new ColorSensorV3(GripperConstants.I2C_PORT);
=======
>>>>>>> 042675701f34cb5cc47cd84e517b933d9bb4bfbb
    rollerMotor = new CANSparkMax(ROLLER_MOTOR, MotorType.kBrushless);
    this.limelight = limelight;
    rollerMotor.setSmartCurrentLimit(20);
    rollerMotor.setInverted(true);
    rollerMotor.setSmartCurrentLimit(ROLLER_MOTOR_CURRENT_LIMIT_VALUE);
    isHolding = true;
    
  }

  public void spin(double speed){
    rollerMotor.set(speed);
  }

  /**
   * Spins the gripper roller to intake
   * sets speed to 0 to stop
   **/
  

  public CommandBase cubeIntake(){
    return runEnd(
      //run 
    () -> {
      if(rollerMotor.getOutputCurrent() < MAX_CUBE_DRAW){
        spin(ROLLER_SPEED);
      }else{
        spin(0);
      }
    },
    //end
    () -> {
      spin(0);
      isHolding = true;
    }
    ); 
  }

  public CommandBase coneIntake(){
    return runEnd(
      //run 
    () -> {
      if(rollerMotor.getOutputCurrent() < DEFAULT_DRAW){
        spin(ROLLER_SPEED);
      }else{
        spin(0);
      }
    },
    //end
    () -> {
      spin(0);
      isHolding = true;
    }
    );
  }

  public CommandBase hold(){
    return runEnd(
      //run
      () -> {
<<<<<<< HEAD
        System.out.println("Trying to spin");
        spin(ROLLER_SPEED);
=======
          if(isHolding){
            spin(0.05);
          }else{
            spin(0);
          }
>>>>>>> 042675701f34cb5cc47cd84e517b933d9bb4bfbb
      },
      //end
      () -> {
        spin(0);
      }
    );
  }

<<<<<<< HEAD

  private boolean checkForGamePiece() {
    return false;
    //return colorSensor.getIR() > GripperConstants.GAME_PIECE_IR_MINIMUM;
  }

  private boolean checkForCube() {
    return false;
    //return checkForGamePiece() && colorSensor.getColor().blue > MINIMUM_BLUE_VALUE_FOR_CUBE;
  }

=======
>>>>>>> 042675701f34cb5cc47cd84e517b933d9bb4bfbb
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
        isHolding = false;
      }
    );
  }

  @Override
  public void initSendable(SendableBuilder builder){
  
    builder.setSmartDashboardType("GripperSystem");
<<<<<<< HEAD
/*
    builder.addDoubleProperty("Red", () -> colorSensor.getColor().red, null);
    builder.addDoubleProperty("Green", () -> colorSensor.getColor().green, null);
    builder.addDoubleProperty("Blue", () -> colorSensor.getColor().blue, null);
    builder.addDoubleProperty("IR", () -> colorSensor.getIR(), null);
    builder.addDoubleProperty("Proximity", () -> colorSensor.getProximity(), null);
    builder.addBooleanProperty("Cube picked up?", () -> checkForCube(), null);
    builder.addBooleanProperty("Game piece picked up?", () -> checkForGamePiece(), null);
=======
>>>>>>> 042675701f34cb5cc47cd84e517b933d9bb4bfbb
    builder.addDoubleProperty("Current Draw Readings", () -> rollerMotor.getOutputCurrent(), null);
*/
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

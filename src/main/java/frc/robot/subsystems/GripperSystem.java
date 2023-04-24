// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import static frc.robot.Constants.GripperConstants.*;

public class GripperSystem extends SubsystemBase implements Testable {

  private CANSparkMax rollerMotor;
  private RelativeEncoder encoder;

  /** current encoder position: rotations of motor shaft */
  private double currPosition;

  /** encoder position during previous robot loop: rotations of motor shaft */
  private double lastPosition;

  /** check whether intake has run since last outtake */
  private boolean isHolding;

  /** Creates a new GripperSystem. */
  public GripperSystem() {
    rollerMotor = new CANSparkMax(ROLLER_MOTOR, MotorType.kBrushless);
    rollerMotor.setSmartCurrentLimit(ROLLER_MOTOR_CURRENT_LIMIT_VALUE);
    rollerMotor.setInverted(true);

    encoder = rollerMotor.getEncoder();

    isHolding = true;
  }

  /**
   * spin the roller motor on the gripper using percent output
   * @param speed [-1.0, 1.0]
   */
  public void spin(double speed) {
    rollerMotor.set(speed);
  }

  /**
   * spin the gripper motors inwards to intake a game piece
   * @return intake command
   */
  public CommandBase intake() {
    return runEnd(
      // runs repeatedly while command is active
      () -> {
        // don't pop the cube
        if (rollerMotor.getOutputCurrent() < DEFAULT_DRAW) {
          spin(ROLLER_SPEED);
        } else {
          spin(0);
        }
      },
      // runs once at end of command
      () -> {
        spin(0);
        
        // possibly holding a game piece after intake runs
        isHolding = true;
      }
    );
  }

  /**
   * spins the gripper motor in reverse to outtake a piece
   * @return outtake command
   */
  public CommandBase outtake() {
    return runEnd(
      // runs repeatedly during command
      () -> {
        // negative speed for reverse
        spin(-ROLLER_SPEED);
      },
      // runs once at end of command
      () -> {
        spin(0);
        
        // cannot be holding a game piece after outtaking
        isHolding = false;
      }
    );
  }

  /** check whether intake has run since last outtake */
  public boolean isHolding() {
    return isHolding;
  }

  /** get encoder position from last periodic loop */
  public double getLastPosition() {
    return lastPosition;
  }

  /** get current encoder position */
  public double getCurrentPosition() {
    return currPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lastPosition = currPosition;
    currPosition = encoder.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("GripperSystem");
    builder.addBooleanProperty("Possibly holding", () -> isHolding, null);

    builder.addDoubleProperty("Current draw (Amps)", () -> rollerMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Current encoder position (rot)", () -> currPosition, null);
    builder.addDoubleProperty("Last encoder position (rot)", () -> lastPosition, null);
    builder.addDoubleProperty("Position delta (rot)", () -> currPosition - lastPosition, null);
  }

  @Override
  public List<Connection> hardwareConnections() {
    return List.of(
      Connection.fromSparkMax(rollerMotor)
    );
  }

  @Override
  public CommandBase testRoutine() {
    return Commands.sequence(
      // run intake
      intake().withTimeout(1.5)
    );
  }
}

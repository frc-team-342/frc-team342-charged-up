// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.GripperConstants.*;
import frc.robot.Limelight;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.GripperSystem;
import frc.robot.subsystems.AddressableLEDSubsystem.ColorType;

public class Intake extends CommandBase {
  /** Creates a new GripperSystemIntakeCommand. */

  AddressableLEDSubsystem aLEDSub;
  GripperSystem gripperSystem;
  Limelight limelight;

  public Intake(AddressableLEDSubsystem aLEDSub, GripperSystem gripperSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.aLEDSub = aLEDSub;
    this.gripperSystem = gripperSystem;
    limelight = new Limelight();

    addRequirements(aLEDSub);
    addRequirements(gripperSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    gripperSystem.spin(ROLLER_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    gripperSystem.spin(0);
        
    /** This logic changes the vision mode depending on whatever game piece has been grabbed */
    if(gripperSystem.checkForCube()) {
      limelight.setPipeline(1);
      aLEDSub.DriverColor(ColorType.PURPLE);
    } else if(gripperSystem.checkForGamePiece()) {
      limelight.setPipeline(0);
      aLEDSub.DriverColor(ColorType.YELLOW);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

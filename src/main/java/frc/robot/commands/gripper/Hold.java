// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.GripperSystem;
import frc.robot.subsystems.AddressableLEDSubsystem.ColorType;

public class Hold extends CommandBase {
  /** Creates a new ConeIntake. */

  GripperSystem gripperSubsystem;
  AddressableLEDSubsystem aLedSubsystem;

  public Hold(GripperSystem gripperSubSystem, AddressableLEDSubsystem aLedSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripperSubsystem = gripperSubSystem;
    this.aLedSubsystem = aLedSubsystem;

    addRequirements(gripperSubSystem);
    addRequirements(aLedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(gripperSubsystem.getIsHolding()){
      gripperSubsystem.spin(0.15);
      if(gripperSubsystem.getEncoder().getPosition() <= (gripperSubsystem.getLastPosition() + 5))
      {
        aLedSubsystem.DriverColor(ColorType.RED);
      }
    }
    else{
      gripperSubsystem.spin(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

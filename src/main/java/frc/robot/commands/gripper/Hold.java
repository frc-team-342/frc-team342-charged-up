// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.GripperSystem;

public class Hold extends CommandBase {
  private GripperSystem gripperSubsystem;
  private LEDSystem aLedSubsystem;

  /** hold game piece, display led if game piece is in gripper */
  public Hold(GripperSystem gripperSubSystem, LEDSystem aLedSubsystem) {
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
  public void execute() {
    // check whether a piece has been intaked since last outtake
    if (gripperSubsystem.isHolding()) {
      // hold game piece in intake
      gripperSubsystem.spin(0.15);

      // change in encoder position in gripper motors
      double delta = gripperSubsystem.getCurrentPosition() - gripperSubsystem.getLastPosition();

      // if there is a game piece in the intake, display the red driver light
      if (delta <= 0.01) {
        aLedSubsystem.setBackPanel(Color.kRed);
      } else {
        aLedSubsystem.setBackPanel(Color.kBlack);
      }
    } else {
      // otherwise just don't spin and turn leds off
      gripperSubsystem.spin(0);
      aLedSubsystem.setBackPanel(Color.kBlack);
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

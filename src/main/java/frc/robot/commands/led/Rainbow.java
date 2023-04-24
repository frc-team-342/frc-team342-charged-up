// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSystem;

public class Rainbow extends CommandBase {
  /** current hue */
  private int current = 0;

  /** led subsystem */
  private LEDSystem led;

  /** Creates a new Rainbow. */
  public Rainbow(LEDSystem led) {
    this.led = led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set leds to current color
    led.setFrontPanel(Color.fromHSV(current, 255, 70));
    led.setBackPanel(Color.fromHSV(current, 255, 70));

    // increment hue
    current++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // turn leds off
    led.setFrontPanel(Color.fromHSV(current, 255, 70));
    led.setBackPanel(Color.fromHSV(current, 255, 70));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

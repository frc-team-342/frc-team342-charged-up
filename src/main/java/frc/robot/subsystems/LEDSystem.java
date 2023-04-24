// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDConstants.*;

public class LEDSystem extends SubsystemBase implements Testable {

  public enum LEDPanel {
    BackPanel(0, FRONT_PANEL_INDEX),
    FrontPanel(FRONT_PANEL_INDEX, LENGTH),
    BothPanels(0, LENGTH);

    public final int lower;
    public final int upper;

    private LEDPanel(int lower, int upper) {
      this.lower = lower;
      this.upper = upper;
    }
  }

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LEDSystem() {
    buffer = new AddressableLEDBuffer(LENGTH);

    led = new AddressableLED(PWM_PORT);
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  /**
   * set the back panel of leds on the robot to a specific color
   * @param color the color to set the leds to
   */
  public void setBackPanel(Color color) {
    // back panel is indexed 0 through FRONT_PANEL_INDEX
    for (int i = 0; i < FRONT_PANEL_INDEX; i++) {
      buffer.setLED(i, color);
    }

    led.setData(buffer);
  }

  /**
   * set the front panel of leds on the robot to a specific color
   * @param color the color to set the leds to
   */
  public void setFrontPanel(Color color) {
    // front panel is indexed FRONT_PANEL_INDEX through end of buffer
    for (int i = FRONT_PANEL_INDEX; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }

    led.setData(buffer);
  }

  /** set either panel to off */
  public void off(LEDPanel panel) {
    for(int i = panel.lower; i < panel.upper; i++) {
      buffer.setLED(i, Color.kBlack);
    }

    led.setData(buffer);
  }

  public CommandBase humanPlayer(Color color) {
    return new FunctionalCommand(
      () -> {}, 
      () -> {
        setFrontPanel(color);
      }, 
      (Boolean interrupted) -> {
        off(LEDPanel.FrontPanel);
      }, 
      () -> { return false; }
    );
  }

  public CommandBase driver(Color color) {
    return new FunctionalCommand(
      () -> {}, 
      () -> {
        setBackPanel(color);
      }, 
      (Boolean interrupted) -> {
        off(LEDPanel.BackPanel);
      }, 
      () -> { return false; }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public CommandBase testRoutine() {
    return Commands.sequence();
  }
}
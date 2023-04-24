// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private Command testCommand;

  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // instantiate branch and commit files from deploy directory
    File branchFile = new File(Filesystem.getDeployDirectory(), "branch.txt");
    File commitFile = new File(Filesystem.getDeployDirectory(), "commit.txt");

    String branch, commit;
    branch = commit = "";

    // try/catch since file operations can fail
    try {
      // attempt to convert file contents to string
      branch = new String(Files.readAllBytes(branchFile.toPath())); // read all text from file into string
      commit = new String(Files.readAllBytes(commitFile.toPath()));
    } catch (IOException e) {
      // Files#readAllBytes can throw an IOException if the file is not present or not readable
      System.err.println(e); // print error to console
      commit = branch = "Not found"; // set dashboard strings to error message
    }

    // send data from files to dashboard
    SmartDashboard.putString("Git Branch", branch);
    SmartDashboard.putString("Git Commit", commit);

    // livewindow is bad and should never be on
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.setBrakeMode(false);
    robotContainer.getDisabledCommand().schedule();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.setBrakeMode(true);
    robotContainer.disableLL3DMode();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // it turns itself back on if you dont do it here too
    LiveWindow.setEnabled(false);

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // schedule automatic function test routine
    testCommand = robotContainer.getTestCommand();
    testCommand.schedule();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

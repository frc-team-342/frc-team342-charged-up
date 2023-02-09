// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static class DriveConstants{
    public static final int FRONT_LEFT_MOTOR = 5;
    public static final int FRONT_RIGHT_MOTOR = 6;
    public static final int BACK_LEFT_MOTOR = 3;
    public static final int BACK_RIGHT_MOTOR = 4;

    public static final double NORMAL_SPEED = 0.8;
    public static final double SLOW_SPEED = 0.4; 

    /** meters */
    public static final double WHEEL_RADIUS = Units.inchesToMeters(3.0);

    /** meters */
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

    /** distance between left and right wheels - meters */
    public static final double TRACK_WIDTH = Units.inchesToMeters(25.245);

    /** driving to driven */
    public static final double GEAR_RATIO = 5.45 / 1;

    /** meters / second */
    public static final double MAX_SPEED = 5.0;

    /** kg m^2 */
    public static final double MOMENT_OF_INERTIA = 3.53325;

    /** kg */
    public static final double MASS = 35.27; // subject to change
  }

  public static class LiftConstants{
    public static final double MAX_SPEED = 0.25;

    // Subject to change when lift is built
    public static final double MAX_POSITION = 85;
    public static final double MIN_POSITION = 0;

    public static final int MOTOR_LEFT = 3;
    public static final int MOTOR_RIGHT = 7;
  }
}

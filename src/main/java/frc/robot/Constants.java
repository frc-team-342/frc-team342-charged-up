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
    public static final int FRONT_LEFT_MOTOR = 1;
    public static final int FRONT_RIGHT_MOTOR = 2;
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

  public static class LimelightConstants{
    
    /** Meters */
    public static final double HEIGHT_TO_LOW = Units.inchesToMeters(6.25);

    /** Meters */
    public static final double HEIGHT_TO_MED = Units.inchesToMeters(19.125);

    /** Meters */
    public static final double HEIGHT_TO_HIGH = Units.inchesToMeters(30.875);

    /** Meters */
    public static final double HEIGHT_TO_HP_STATION = Units.inchesToMeters(15.375);

    /** todo: add actual values for constants below this line */
    /** Degrees */
    public static final double MAX_VERT_OFFSET_FOR_LOW = 16.0;

    /** Degrees */
    public static final double MAX_VERT_OFFSET_FOR_MED = 27.0;

    /** Degrees */
    // public static final double MAX_VERT_OFFSET_FOR_HIGH = 90.0;

    /** Degrees */
    public static final double MAX_VERT_OFFSET_FOR_HP_STATION = 29.0;

    /* Feet */
    public static final double AUTO_ARM_RAISE_MAX_RANGE = 2;

    /* Feet */
    public static final double DISTANCE_BETWEEN_MID_AND_HIGH = 0.0;
  }
}

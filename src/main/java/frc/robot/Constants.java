// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
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
    public static final int OP_CONTROLLER = 0;
    public static final int DRIVER_LEFT_PORT = 1;
    public static final int DRIVER_RIGHT_PORT = 2;

    public static final int OP_BUTTON_HUMAN_PLAYER_YELLOW = 9;
    public static final int OP_BUTTON_HUMAN_PLAYER_PURPLE = 10;
  }

  public static class LEDConstants {
    public static final int PWM_PORT = 0;
    public static final int LENGTH = 512;
    public static final int DRIVER_START_RANGE = 256;

    //HSV Values
    public static final int YELLOW_H = 40;
    public static final int YELLOW_S = 255;
    public static final int YELLOW_V = 70;
    public static final int PURPLE_H = 150;
    public static final int PURPLE_S = 255;
    public static final int PURPLE_V = 70;
  }
  
  public static class GripperConstants {
    public static final I2C.Port I2C_PORT = I2C.Port.kOnboard;
    public static final int ROLLER_MOTOR = 5;
    public static final double ROLLER_SPEED = 0.5;
  }

  public static class DriveConstants {

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

  public static class LiftConstants {
    public static final double MAX_SPEED = 0.15;

    // Subject to change when lift is built
    public static final double MAX_POSITION = 1;
    public static final double MIN_POSITION = 0;

    public static final int MOTOR_LEFT = 6;
    public static final int MOTOR_RIGHT = 7;

    public static final int LIMIT_SWITCH_UP = 2;
    public static final int LIMIT_SWITCH_DOWN = 1;
  }

  /*
   * todo: add actual values for all of the constants, they are currently placeholders
   */
  public static class LimelightConstants {
    public static final double HEIGHT_TO_LOW = 0.0;
    public static final double HEIGHT_TO_MED = 0.0;
    public static final double HEIGHT_TO_HIGH = 0.0;
    public static final double MAX_VERT_OFFSET_FOR_LOW = 30.0;
    public static final double MAX_VERT_OFFSET_FOR_MED = 60.0;
    public static final double MAX_VERT_OFFSET_FOR_HIGH = 90.0;
  }

  public static class AutoConstants {
    public static final double FAST_SPEED = 0.5;
    public static final double SLOW_SPEED = 0.25;
  }
}

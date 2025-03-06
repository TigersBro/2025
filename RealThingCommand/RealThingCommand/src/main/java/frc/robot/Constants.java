// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 8;
    public static final int LEFT_FOLLOWER_ID = 2; 
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    public static final double DRIVE_MOTOR_VOLTAGE_COMP = 12;
    public static final double SLOW_MODE_MOVE = 0.5;
    public static final double SUPER_SLOW_MODE_MOVE = 0.3;
    public static final double SUPER_SLOW_MODE_TURN = 0.3;
    public static final double SLOW_MODE_TURN = 0.6;

    public static final int DRIVE_REVERSE_FRONT_BUTTON_ID = 4;
    public static final int DRIVE_REVERSE_ROTATION_BUTTON_ID = 11;
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_CORAL_OUT = -.4;
    public static final double ROLLER_ALGAE_IN = 0.7;
    public static final double ROLLER_ALGAE_OUT = -0.4;
    public static final double ROLLER_CORAL_STACK = -1;
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 6;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = 0.35;
    public static final double ARM_SPEED_UP = -0.35;
    public static final double ARM_HOLD_DOWN = -0.05;
    public static final double ARM_HOLD_UP = 0.05;
    public static final int ARM_LOWER_LIMIT_MAG_ID = 8;
    public static final int ARM_UPPER_LIMIT_MAG_ID = 9;
    public static final int ARM_MOTOR_STOP = 0;
    public static final String ARM_DOWN_DIRECTION_STRING = "DOWN";
    public static final String ARM_UP_DIRECTION_STRING = "UP";
    public static final int ARM_GEAR_RATIO = 1;
    public static final int ARM_OVERRIDE_BUTTON = 7;
    public static final String ARM_NOT_SET = "NOTSET";
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_rightwinch_MOTOR_ID = 7;
    public static final int CLIMBER_leftwinch_MOTOR_ID = 9;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 60;
    public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
    public static final double CLIMBER_SPEED_DOWN = -0.5;
    public static final double CLIMBER_SPEED_UP = 0.5;
    public static final double CLIMBER_GEAR_RATIO = 71;
    public static final double CLIMBER_CPR = 7/4;
    public static final int CLIMBER_ENCODER_1 = 1;
    public static final int CLIMBER_ENCODER_2 = 2;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
  public static final class ShuffleBoard9638Constants {
    public static final String DEFAULT_TAB = "Test";
    public static final int UPDATE_PERIOD = 3; //Update variables from Shuffleboard every 3 seconds
    }


}

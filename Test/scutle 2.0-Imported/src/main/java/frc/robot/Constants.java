// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain{
        public static final int PWM_L1 = 0;
        public static final int PWM_L2 = 18;
        public static final int PWM_R1 = 1;//2
        public static final int PWM_R2 = 19;
    }
    public static final class Controls{
        public static final int JOYSTICK_USB = 0;
        public static final int PS5_USB = 1;
        public static final int JOYSTICK_FIRE3 = 270;
        public static final int JOYSTICK_FIRE4 = 0;
        public static final int JOYSTICK_FIRE5 = 90;

        public static final int JOYSTICK_UP = 4;
        public static final int JOYSTICK_DN = 2;
        public static final int STOP_COMPRESSOR = 7;
    }
    public static final class Pneumatics{
        public static final int PNEUMATICS_LIFTER_UP = 0;
        public static final int PNEUMATICS_LIFTER_DN = 1;
        public static final int PNEUMATICS_PRESSURE_LIMIT = 120;
    }
    public static final class DriveConstants {
        public static final int LEFT_LEADER_ID = 8;
        public static final int LEFT_FOLLOWER_ID = 2; 
        public static final int RIGHT_LEADER_ID = 3;
        public static final int RIGHT_FOLLOWER_ID = 4;
    
        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
        public static final double DRIVE_MOTOR_VOLTAGE_COMP = 12;
        public static final double SLOW_MODE_MOVE = 0.5;
        public static final double SLOW_MODE_TURN = 0.6;
      }
}

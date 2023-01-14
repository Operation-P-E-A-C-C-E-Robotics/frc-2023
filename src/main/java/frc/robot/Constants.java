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
    public final class Kinematics {
        public static final double PIVOT_HEIGHT = 0; //TODO how high the pivot is above the ground

    }


    public final class DriveTrain {
        public static final int LEFT_MASTER  = 0,
                                LEFT_SLAVE   = 1,
                                RIGHT_MASTER = 2,
                                RIGHT_SLAVE  = 3;


    }

    public final class Wrist{
        public static final double LENGTH = 0.1,
                                  LENGTH_TO_PLACE = 0;
    }

    public final class OperatorInterface {
        public static final int DRIVER_JOYSTICK = 0;
    }

    public final class Auto {
        public static final int PIGEON_IMU  = 0,
                                TRACK_WIDTH = 0; //TODO Configure

        public static final double kS = 0,
                                  kV = 0,
                                  kA = 0,
                                  kP = 0,
                                  kI = 0,
                                  kD = 0,
                                  RAMSETE_B = 0,
                                  RAMSETE_ZETA = 0,
                                  AUTO_VOLTAGE_MAX = 10,
                                  AUTO_MAX_SPEED_METERS_PER_SECOND = 0,
                                  AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0;
    }
}

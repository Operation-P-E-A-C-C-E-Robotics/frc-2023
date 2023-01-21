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
    public static final class Kinematics {
        public static final double PIVOT_HEIGHT = 0; //TODO how high the pivot is above the ground

    }


    public static final class DriveTrain {
        public static final int LEFT_MASTER  = 0,
                                LEFT_SLAVE   = 1,
                                RIGHT_MASTER = 2,
                                RIGHT_SLAVE  = 3;

        public static final double DRIVE_ENCODER_CPR = 2048;
        public static final double GEARBOX_RATIO_HIGH = 10.66;//:1
        public static final double METERS_PER_ROTATION = 0.4844;
    }

    public static final class OperatorInterface {
        public static final int DRIVER_JOYSTICK = 0;
    }

    public static final class EndEffector {
        public static final double LENGTH = 0.1,
                                  LENGTH_TO_PLACE = 0;
    }

    @SuppressWarnings("HungarianNotationConstants")
    public static final class Auto {
        public static final int PIGEON_IMU  = 20;
        public static final double TRACK_WIDTH = 0.6; //TODO Configure

        public static final double kS = 0.10351,
                                  kV = 2.4155,
                                  kA = 0.2751,
                                  kP = 0.24921,
                                  kI = 0,
                                  kD = 0,
                                  RAMSETE_B = 2.0,
                                  RAMSETE_ZETA = 0.3,
                                  AUTO_VOLTAGE_MAX = 7,
                                  AUTO_MAX_SPEED_METERS_PER_SECOND = 1,
                                  AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.8;
    }
}

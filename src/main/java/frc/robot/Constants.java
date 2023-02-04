// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.util.DCMotorSystemBase.SystemConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static final class Pivot{
        public static final int PIVOT_MASTER = 50, //TODO Arbitrary number to not conflict
                                PIVOT_SLAVE  = 51;

        public static final SystemConstants SYSTEM_CONSTANTS = new SystemConstants(
                DCMotor.getFalcon500(2),
                10,
                100,
                2048,
                0.1,
                0.1,
                0.1,
                0.1,
                0.1,
                0.1,
                0.1,
                0.1,
                12,
                0,
                12,
                0.02
        );
    }

    public static final class Turret {
        public static final int MOTOR_PORT = 90; //TODO Arbitrary number to not conflict

        public static final SystemConstants SYSTEM_CONSTANTS = new SystemConstants(
                DCMotor.getFalcon500(1),
                1,
                50,
                2048,
                4,
                10,
                3.0,
                3.0,
                0.01,
                0.01,
                0.05,
                0.05,
                12,
                0,
                12,
                0.02
        );

    }

    public static final class Kinematics {
        public static final double   PIVOT_HEIGHT = 0,
                                    END_EFFECTOR_LENGTH = 0.1,
                                    END_EFFECTOR_LENGTH_TO_PLACE = 0;
    }

    public static final class Wrist {
        public static final int WRIST_MOTOR          = 70, //TODO Arbitrary number to not conflict
                                WRIST_FLIP_FORWARD   = 0,
                                WRIST_FLIP_REVERSE   = 1;
        public static final double WRIST_FLIP_TIME = 0.5;
    }

    public static final class DriveTrain {
        public static final int  LEFT_MASTER  = 0,
                                LEFT_SLAVE   = 1,
                                RIGHT_MASTER = 2,
                                RIGHT_SLAVE  = 3;

        public static final double  DRIVE_ENCODER_CPR = 2048,
                                    GEARBOX_RATIO_HIGH = 10.66,//:1
                                    METERS_PER_ROTATION = 0.4844,
                                    MOMENT_OF_INERTIA = 7.5, //TODO
                                    MASS = 60.0, //TODO
                                    TRACK_WIDTH = 0.6; //TODO Configure

        public static final double LQR_ERROR_TOLERANCE = 0.1,
        LQR_EFFORT = 12.0,
        KALMAN_MODEL_ACCURACY = 3,
        KALMAN_SENSOR_ACCURACY = 0.1;

        public static final double DT = 0.02;

        public static final int PIGEON_IMU  = 20;

        //velocity constants:
        public static final double   kS = 0.10351,
                                    kV = 2.4155,
                                    kA = 0.2751,
                                    kP = 0.24921,
                                    kI = 0,
                                    kD = 0,
                                    RAMSETE_B = 2.0,
                                    RAMSETE_ZETA = 0.3,
                                    AUTO_VOLTAGE_MAX = 7,
                                    AUTO_MAX_SPEED_METERS_PER_SECOND = 1,
                                    AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.1;

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
    }

    public static final class Arm {
        public static final int MASTER_PORT = 95, //TODO Arbitrary number to not conflict
                                ARM_SLAVE  = 97;
        public static final double INERTIA = 0,
                                    GEARING = 0;
    }

    public static final class Constraints{
        public static final double DRIVE_SLEW_RATE_LIMIT_NORMAL = 10, //todo
                                  DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED = 0, //todo
                                  LIFT_EXTENDED_THRESHOLD = 0.4; //todo
    }

    public static final class OperatorInterface {
        public static final int DRIVER_JOYSTICK = 0;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.util.DCMotorSystemBase.SystemConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

//TODO CAN ID's according to the Electrical Service Manual (https://docs.google.com/document/d/1KB8-KpFrxM39kcLAH9h3_pHSRaMYU2LS7VqXjaPE9zI/edit?usp=sharing)
//CAN ID's in the order they are connected in the chain, 1st connection is ID 0, 2nd is ID 1, etc
public final class Constants {
    public static final class DriveTrain {
        //ports
        public static final int LEFT_MASTER  = 2, //DOCS Drive Falcon 0
                                LEFT_SLAVE   = 3, //DOCS Drive Falcon 1
                                RIGHT_MASTER = 0, //DOCS Drive Falcon 3
                                RIGHT_SLAVE  = 1, //DOCS Drive Falcon 2
                                PIGEON_IMU   = 5,
                                SHIFT_HIGH_PORT = 0,
                                SHIFT_LOW_PORT = 1;

        //physical constants
        public static final double  DRIVE_ENCODER_CPR = 2048,
                GEARBOX_RATIO_HIGH = 10.66,//:1
                GEARBOX_RATIO_LOW = 17.88,//:1
                METERS_PER_ROTATION = 0.4844,
                MOMENT_OF_INERTIA = 7.5, //J/m^2 //TODO
                MASS = 68.0, //kg //TODO
                TRACK_WIDTH = 0.6096; //m

        //state space constants
        public static final double  LQR_ERROR_TOLERANCE = 0.1,
                LQR_EFFORT = 12.0,
                KALMAN_MODEL_ACCURACY = 3,
                KALMAN_SENSOR_ACCURACY = 0.1;

        //loop time
        public static final double DT = 0.02;

        //velocity constants:
        public static final double  kS = 0.10351, //Volts
                kV = 2.4155, //Volts per meter per second
                kA = 0.2751, //Volts per meter per second squared
                kP = 0.24921,
                kI = 0,
                kD = 0,
                RAMSETE_B = 2.0,
                RAMSETE_ZETA = 0.3,
                AUTO_VOLTAGE_MAX = 7,
                AUTO_MAX_SPEED_METERS_PER_SECOND = 1, //3
                AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.1;

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
    }

    public static final class Turret {
        public static final int MOTOR_PORT = 4;

        //constraints:
        public static final double  MAX_ANGLE_RAD = Units.degreesToRadians(270), //TODO actual constraints
                MIN_ANGLE_RAD = -MAX_ANGLE_RAD;

        public static final SystemConstants SYSTEM_CONSTANTS = new SystemConstants(
                DCMotor.getFalcon500(1),
                5,
                100 * 5, //100:1 versaplanetary, 5:1 driving gear
                2048,
                5,
                10,
                3.0,
                3.0,
                0.01,
                0.01,
                0.5,
                1,
                12,
                0,
                12,
                0.02
        );

    }

    public static final class Pivot{
        //port numbers
        public static final int PIVOT_MASTER = 9, //DOCS Pivot Falcon 0
                                PIVOT_SLAVE  = 10, //DOCS Pivot Falcon 1
                      BRAKE_SOLENOID_FORWARD = 5,
                     BRAKE_SOLENOID_BACKWARD = 6;
        //constraints
        public static final double  MAX_ANGLE_RAD = Math.PI, //TODO actual constraints
                                    MIN_ANGLE_RAD = -Math.PI,
                                    TIME_FOR_BRAKE_TO_ENGAGE = 0.5; //TODO Meassure time for Break to engage

        //physical constants
        public static final double  LENGTH = 0.5,
                                    MASS = 10;

        public static final SystemConstants SYSTEM_CONSTANTS = new SystemConstants(
                DCMotor.getFalcon500(2),
                5, //6.67
                200,
                2048,
                0.5,
                0.5, //0.735
                0.1,
                0.1,
                0.1,
                0.1,
                0.01,
                0.1,
                12,
                0,
                12,
                0.02
        );
    }

    public static final class Arm {
        //ports
        public static final int MASTER_PORT = 12; //DOCS IGUS Extension Falcon 0 //TODO is this the right motor

        //physical constants
        public static final double  CARRAIGE_MASS = 5, //kg
                MIN_EXTENSION = 0.5, //m
                MAX_EXTENSION = 1.5; //m

        public static final SystemConstants SYSTEM_CONSTANTS = new SystemConstants(
                DCMotor.getFalcon500(1),
                5,
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

    public static final class Wrist {
        //ports
        public static final int WRIST_MOTOR = 13, //DOCS Wrist Falcon 0
                        WRIST_FLIP_FORWARD  = 2, //TODO Get PH port
                        WRIST_FLIP_REVERSE  = 3; //TODO get PH port

        //constants
        public static final double WRIST_FLIP_TIME = 0.5; //seconds TODO time to flip wrist

        //physical constants
        public static final double LENGTH = 0.2, //meters
                MASS = 0.1; //kg

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
        public static final double  PIVOT_HEIGHT = 0,
                                    END_EFFECTOR_LENGTH = 0.15,
                                    END_EFFECTOR_LENGTH_TO_PLACE = 0.1;
    }

    public static final class Constraints{
        public static final double DRIVE_SLEW_RATE_LIMIT_NORMAL = 10, //todo
                                  DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED = 0, //todo
                                  LIFT_EXTENDED_THRESHOLD = 0.4; //todo
    }

    public static final class OperatorInterface {
        public static final int DRIVER_JOYSTICK = 0;
    }

    public static final class SupersystemTolerance{
        public final double turret, pivot, wrist, arm;
        public static final SupersystemTolerance DEFAULT = new SupersystemTolerance(0.1, 0.1, 0.1, 0.1);
        public static final SupersystemTolerance PLACE_HIGH = new SupersystemTolerance(0.1, 0.1, 0.1, 0.1);
        public static final SupersystemTolerance PLACE_MID = new SupersystemTolerance(0.1, 0.1, 0.1, 0.1);
        public static final SupersystemTolerance PLACE_LOW = new SupersystemTolerance(0.1, 0.1, 0.1, 0.1);
        public static final SupersystemTolerance INTAKE_GROUND = new SupersystemTolerance(0.1, 0.1, 0.1, 0.1);
        public static final SupersystemTolerance INTAKE_SUBSTATION = new SupersystemTolerance(0.1, 0.1, 0.1, 0.1);
        public SupersystemTolerance(double turret, double pivot, double wrist, double arm){
            this.turret = turret;
            this.pivot = pivot;
            this.wrist = wrist;
            this.arm = arm;
        }
    }

    public static final boolean TUNING_MODE = true;
}

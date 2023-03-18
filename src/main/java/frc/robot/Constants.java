// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.ServoMotor.SystemConstants;
import frc.robot.commands.supersystem.Automations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

//CAN ID's according to the Electrical Service Manual (https://docs.google.com/document/d/1KB8-KpFrxM39kcLAH9h3_pHSRaMYU2LS7VqXjaPE9zI/edit?usp=sharing)
//CAN ID's in the order they are connected in the chain, 1st connection is ID 0, 2nd is ID 1, etc
public final class Constants {

    public static final class Inversions{
        public static final boolean DRIVE_LEFT = false,
                                    DRIVE_RIGHT = true,
                                    TURRET = false,
                                    TURRET_ENCODER = true,
                                    PIVOT = true,
                                    PIVOT_ENCODER = true,
                                    ARM = true,
                                    WRIST = true,
                                    INTAKE_LEFT = false,
                                    INTAKE_RIGHT = true;
    }
    public static final class DriveTrain {
        //ports
        public static final int LEFT_MASTER  = 0, //DOCS Drive Falcon 0
                                LEFT_SLAVE   = 1, //DOCS Drive Falcon 1
                                RIGHT_MASTER = 3, //DOCS Drive Falcon 3
                                RIGHT_SLAVE  = 2, //DOCS Drive Falcon 2
                                PIGEON_IMU   = 5,
                                SHIFT_HIGH_PORT = 1,
                                SHIFT_LOW_PORT = 0;

        //physical constants
        public static final double  DRIVE_ENCODER_CPR = 2048,
                GEARBOX_RATIO_HIGH = 10.66,//:1
                GEARBOX_RATIO_LOW = 17.88,//:1
                WHEEL_CIRCUMFERENCE = 0.4785,
                MOMENT_OF_INERTIA = 7.5, //J/m^2
                MASS = Units.lbsToKilograms(100), //kg
                TRACK_WIDTH = 0.6096; //m

        //state space constants
        public static final double  LQR_ERROR_TOLERANCE = 0.5,
                LQR_EFFORT = 12.0,
                KALMAN_MODEL_ACCURACY = 3,
                KALMAN_SENSOR_ACCURACY = 0.1;

        public static final double wheelDiameter = 0.477522;

        //loop time
        public static final double DT = 0.02;

        //velocity constants:
        public static final double  kS = 0.10351, //Volts
                kV_LINEAR = 2.4155, //Volts per meter per second
                kA_LINEAR = 0.2751, //Volts per meter per second squared
                kV_ANGULAR = 2, //Volts per radian per second
                kA_ANGULAR = 0.2, //Volts per radian per second squared
                kV_LINEAR_LOW = 2.4155, //Volts per meter per second
                kA_LINEAR_LOW = 0.2751, //Volts per meter per second squared
                kV_ANGULAR_LOW = 2, //Volts per radian per second
                kA_ANGULAR_LOW = 0.2, //Volts per radian per second squared
                kP = 0.24921,
                kI = 0,
                kD = 0,
                AUTO_VOLTAGE_MAX = 7,
                AUTO_MAX_SPEED_METERS_PER_SECOND = 1, //3
                AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.1;

        public static final StatorCurrentLimitConfiguration CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
                false,
                50,
                60,
                0.5
        );
        public static final StatorCurrentLimitConfiguration HARD_CURRENT_LIMIT = CURRENT_LIMIT; //new StatorCurrentLimitConfiguration(
        //         true,
        //         35,
        //         40,
        //         0.1
        // );
        public static final StatorCurrentLimitConfiguration SHIFTING_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
                true,
                10,
                10,
                0
        );
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
    }

    public static final class Turret {
        public static final int MOTOR_PORT = 4,
                                ENCODER_PORT = 10;

        //constraints:
        public static final double  MAX_ANGLE_RAD = Units.degreesToRadians(90),
                MIN_ANGLE_RAD = -MAX_ANGLE_RAD;

        public static final StatorCurrentLimitConfiguration CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
                true,
                40,
                50,
                0.1
        );

        public static final SystemConstants SYSTEM_CONSTANTS = new SystemConstants(
                DCMotor.getFalcon500(1),
                2,
                20 * 5, //20:1 versaplanetary, 5:1 driving gear
                4096,
                2,
                3,
                3.0,
                3.0,
                0.01,
                0.01,
                0.03,
                0.05,
                12,
                0,
                12,
                0.02
        );

    }

    public static final class Pivot{
        //port numbers
        public static final int PIVOT_MASTER = 9, //DOCS Pivot Falcon 0
                                PIVOT_SLAVE  = 11, //DOCS Pivot Falcon 1
                                PIVOT_ENCODER = 12, //TODO
                      BRAKE_SOLENOID_FORWARD = 7,
                     BRAKE_SOLENOID_REVERSE = 6;
        //constraints
        public static final double  MAX_ANGLE_RAD = Units.degreesToRadians(120), //TODO actual constraints
                                    MIN_ANGLE_RAD = -MAX_ANGLE_RAD,
                                    TIME_FOR_BRAKE_TO_ENGAGE = 0.1;

        //physical constants
        public static final double  LENGTH = 0.1, //Note: probably going to ignore this length, and use the current arm length (even though it's not perfect)
                                    MASS = Units.lbsToKilograms(40);

        public static final StatorCurrentLimitConfiguration CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
                true,
                50,
                55,
                0.1
        );

        public static final SystemConstants SYSTEM_CONSTANTS = new SystemConstants(
                DCMotor.getFalcon500(
                    2),
                10, //6.67
                3*4*5*6.4,
                4096,
                1.1,
                1.5, //0.735
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
        public static final int MASTER_PORT = 13; //DOCS IGUS Extension Falcon 0

        //physical constants
        public static final double  CARRAIGE_MASS = 5, //kg
                MIN_EXTENSION = 0.41, //m
                MAX_EXTENSION = 1.4; //m

        public static final double PULLY_DIAMETER = 0.044;
        public static final double PULLY_CIRCUMPHERENCE = Math.PI * Math.pow(PULLY_DIAMETER/2, 2);

        public static final double FULLY_EXTENDED_COUNTS = 204000;

        public static final StatorCurrentLimitConfiguration CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
                true,
                50,
                55,
                0.2
        );

        public static final SystemConstants SYSTEM_CONSTANTS = new SystemConstants(
                DCMotor.getFalcon500(1),
                10,
                30 / 0.13823,
                2048,
                1,
                1.5,
                3.0,
                3.0,
                0.01,
                0.01,
                0.0001,
                0.001,
                12,
                0,
                12,
                0.02
        );
    }

    public static final class Wrist {
        //ports
        public static final int WRIST_MOTOR = 14, //DOCS Wrist Falcon 0
                        WRIST_FLIP_SOLENOID = 4;
        //constants
        public static final double WRIST_FLIP_TIME = 0.5; //seconds TODO time to flip wrist

        //physical constants
        public static final double LENGTH = 0.2, //meters
                MASS = 0.1; //kg

        public static final StatorCurrentLimitConfiguration CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
                true,
                30,
                40,
                0.1
        );

        public static final SystemConstants SYSTEM_CONSTANTS = new SystemConstants(
                DCMotor.getFalcon500(1),
                2,
                7*5*4,
                2048,
                1,
                3,
                3.0,
                3.0,
                0.01,
                0.01,
                0.01,
                0.1,
                12,
                0,
                12,
                0.02
        );
    }

    public static final class EndEffector {
        public static final int LEFT_MOTOR_ID = 15,
                             RIGHT_MOTOR_ID = 16,
                             GRIP_PNEUMATICS_PORT = 5,
                             BEAM_BRAKE_PORT = 0;

        public static final double TIME_TO_EJECT = 0.2; //TODO
        public static final double TIME_FOR_CLAW_TO_OPEN = 0.2; //TODO
        public static final Color CUBE_COLOR = new Color(0,0,0), CONE_COLOR = new Color(0,0,0);
    }
    public static final class Kinematics {
        public static final double  PIVOT_HEIGHT = 0.64,
                                    END_EFFECTOR_LENGTH = 0.43,
                                    END_EFFECTOR_LENGTH_TO_PLACE = 0.34;
    }

    public static final class Constraints{
        public static final double DRIVE_SLEW_RATE_LIMIT_NORMAL = 10,
                                  DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED = 1;
    }

    public static final class OperatorInterface {
        public static final int DRIVER_JOYSTICK = 0,
                                OPERATOR_JOYSTICK = 1,
                                BACKUP_JOYSTICK = 2;
    }

    public static final class SupersystemTolerance{
        public final double turret, pivot, wrist, arm;
        public static final SupersystemTolerance DEFAULT = new SupersystemTolerance(0.3, 0.3, 0.3, 0.1);
        public static final SupersystemTolerance PIVOT_BRAKE = new SupersystemTolerance(50, 0.05, 50, 50);
        public static final SupersystemTolerance PLACE_HIGH = new SupersystemTolerance(0.5, 0.5, 0.5, 0.1);
        public static final SupersystemTolerance PLACE_MID = new SupersystemTolerance(0.07, 0.15, 0.1, 0.1);
        public static final SupersystemTolerance PLACE_LOW = new SupersystemTolerance(0.1, 0.1, 0.1, 0.1);
        public static final SupersystemTolerance PRE_PLACE = new SupersystemTolerance(0.1, 0.2, 0.1, 0.3);
        public static final SupersystemTolerance INTAKE_GROUND = new SupersystemTolerance(0.1, 0.1, 0.1, 0.1);
        public static final SupersystemTolerance INTAKE_SUBSTATION = new SupersystemTolerance(0.1, 0.1, 0.1, 0.1);
        public static final SupersystemTolerance TARGET_VISION = new SupersystemTolerance(0.5, 0.5, 0.5, 0.3);

        public static SupersystemTolerance forLevel(Automations.PlaceLevel level){
            return switch (level) {
                case HIGH -> PLACE_HIGH;
                case MID -> PLACE_MID;
                case LOW -> PLACE_LOW;
            };
        }
        public SupersystemTolerance(double turret, double pivot, double wrist, double arm){
            this.turret = turret;
            this.pivot = pivot;
            this.wrist = wrist;
            this.arm = arm;
        }
    }

    public static final boolean TUNING_MODE = true;
    public static final int LOWER_PNEUMATICS_MODULE_CAN_ID = 6,
                            UPPER_PNEUMATICS_MODULE_CAN_ID = 50;

    public static final double PNEUMATICS_MIN_PRESSURE = 80,
                                PNEUMATICS_MAX_PRESSURE = 120;

    public static final double DISABLE_COMPRESSOR_CURRENT_THRESHOLD = 100,
                                DRIVETRAIN_HARD_CURRENT_LIMIT_THRESHOLD = 120;
}

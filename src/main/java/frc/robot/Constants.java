// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
    }

    public static final class Kinematics {
        public static final double   PIVOT_HEIGHT = 0,
                                    END_EFFECTOR_LENGTH = 0.1,
                                    END_EFFECTOR_LENGTH_TO_PLACE = 0;
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

    public static final class Constraints{
        public static final double DRIVE_SLEW_RATE_LIMIT_NORMAL = 10, //todo
                                  DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED = 0, //todo
                                  LIFT_EXTENDED_THRESHOLD = 0.4; //todo
    }

    public static final class OperatorInterface {
        public static final int DRIVER_JOYSTICK = 0;
    }

    public static final class ApriltagStdDevs{
        public static final double[] xDev = {0.03394163197508, 1.055800729975908, 1.053986386384851, 0.218665008653696, 0.24929924575445};
        public static final double[] yDev = {0.055050969821559, 0.36577180879976, 0.368797899856686, 0.359370523837888, 0.439725872370465};
        public static final double[] yawDev = {0.173170027441242, 1.8543212404811, 0.396519970353792, 1.917647725592118, 1.953411408001835};
        public static final double[] distance = {-0.665, -1.41, -2.22, -3.8, -5.27};
    }
}

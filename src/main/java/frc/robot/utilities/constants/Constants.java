package frc.robot.utilities.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    // These Constants will varry depending on what Swerve Setup that you run

    public static final class ModuleConstants {
        public static final double WheelDiameterMeters = Units.inchesToMeters(4);
        public static final double DriveMotorGearRatio = 1 / 5.8462;
        public static final double TurningMotorGearRatio = 1 / 18.0;
        public static final double DriveEncoderRot2Meter = DriveMotorGearRatio * Math.PI * WheelDiameterMeters;
        public static final double TurningEncoderRot2Rad = TurningMotorGearRatio * 2 * Math.PI;
        public static final double DriveEncoderRPM2MeterPerSec = DriveEncoderRot2Meter / 60;
        public static final double TurningEncoderRPM2RadPerSec = TurningEncoderRot2Rad / 60;
        public static final double kP = 0.5;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class SwerveConstants {

        public static final double TrackWidth = Units.inchesToMeters(21); // Distance between centers of right and left wheels on robot
        public static final double WheelBase = Units.inchesToMeters(25.5); // Distance between centers of front and back wheels on robot

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(WheelBase / 2, -TrackWidth / 2),
            new Translation2d(WheelBase / 2, TrackWidth / 2),
            new Translation2d(-WheelBase / 2, -TrackWidth / 2),
            new Translation2d(-WheelBase / 2, TrackWidth / 2)
        );

        public static final int FrontLeftForwardMotorID = 8; 
        public static final int FrontRightForwardMotorID = 6;
        public static final int BackLeftForwardMotorID = 2;
        public static final int BackRightForwardMotorID = 4;

        public static final int FrontLeftThetaMotorID = 7;
        public static final int FrontRightThetaMotorID = 5;
        public static final int BackLeftThetaMotorID = 1;
        public static final int BackRightThetaMotorID = 3;

        public static final boolean FrontLeftThetaEncoderReversed = true;
        public static final boolean BackLeftThetaEncoderReversed = true;
        public static final boolean FrontRightThetaEncoderReversed = true;
        public static final boolean BackRightThetaEncoderReversed = true;

        public static final boolean FrontLeftForwardEncoderReversed = true;
        public static final boolean BackLeftForwardEncoderReversed = true;
        public static final boolean FrontRightForwardEncoderReversed = false;
        public static final boolean BackRightForwardEncoderReversed = false;

        public static final int FrontLeftCANcoderID = 0;
        public static final int FrontRightCANcoderID = 2;
        public static final int BackLeftCANcoderID = 1;
        public static final int BackRightCANcoderID = 3;

        public static final boolean FrontLeftCANcoderReversed = false;
        public static final boolean BackLeftCANcoderReversed = false;
        public static final boolean FrontRightCANcoderReversed = false;
        public static final boolean BackRightCANcoderReversed = false;

        public static final double FrontLeftCANcoderOffsetRadians = -0.254;
        public static final double BackLeftCANcoderOffsetRadians = -1.252;
        public static final double FrontRightCANcoderOffsetRadians = -1.816;
        public static final double BackRightCANcoderOffsetRadians = -4.811;

        public static final double PhysicalMaxSpeedMetersPerSecond = 5;
        public static final double PhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double TeleopMaxSpeedMetersPerSecond = PhysicalMaxSpeedMetersPerSecond / 4;
        public static final double TeleopMaxAngularSpeedRadiansPerSecond = PhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double TeleopMaxAccelerationUnitsPerSecond = 3;
        public static final double TeleopMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class TankConstants {
        public static final int leftMotorCount = 2;
        public static final int rightMotorCount = 2;

        public static final int rightMasterID = 1;
        public static final int rightSlaveID = 2;
        public static final int leftMasterID = 3;
        public static final int leftSlaveID = 4;

        public static final double kDeadband = 0.05;
    }

    public static final class DriverConstants {
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
    }
}

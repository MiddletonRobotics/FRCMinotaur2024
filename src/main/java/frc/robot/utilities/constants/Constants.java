package frc.robot.utilities.constants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

// All of ze constants

public class Constants {
    public static final class ModuleConstants {

        public static final boolean invertGyro = false;

        /* Swerve Voltage Compensation */
        public static final double voltageCompensation = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = (SwerveConstants.WheelDiameter * Math.PI) / SwerveConstants.DriveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / SwerveConstants.AngleGearRatio;

         /* Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(SwerveConstants.FrontLeftCANcoderOffsetDegrees);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightModule {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(SwerveConstants.FrontRightCANcoderOffsetDegrees);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(SwerveConstants.BackLeftCANcoderOffsetDegrees);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(SwerveConstants.BackRightCANcoderOffsetDegrees);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class SwerveConstants {
        public static final double TrackWidth = Units.inchesToMeters(21.05);
        public static final double WheelBase = Units.inchesToMeters(21.05);
        public static final double WheelDiameter = Units.inchesToMeters(4.0);
        public static final double WheelCircumference = WheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double DriveGearRatio = (6.86 / 1.0); // 6.86:1
        public static final double AngleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(WheelBase / 2, TrackWidth / 2),
            new Translation2d(WheelBase / 2, -TrackWidth / 2),
            new Translation2d(-WheelBase / 2, TrackWidth / 2),
            new Translation2d(-WheelBase / 2, -TrackWidth / 2)
        );

        public static final double DriveConversionPositionFactor = (WheelDiameter * Math.PI) / DriveGearRatio;
        public static final double DriveConversionVelocityFactor = DriveConversionPositionFactor / 60.0;
        public static final double AngleConversionFactor = 360.0 / AngleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = true;

        /* Angle Encoder Invert */
        public static final boolean swerveEncoderInverted = false;

        public static final double FrontLeftCANcoderOffsetDegrees = 92.944;
        public static final double BackLeftCANcoderOffsetDegrees = 257.5635;
        public static final double FrontRightCANcoderOffsetDegrees = 181.9725;
        public static final double BackRightCANcoderOffsetDegrees = 235.239;

        public static final double PhysicalMaxSpeedMetersPerSecond = 4.5;
        public static final double AngularMaxVelocity = 11.5;
    }
// Tank motor ids and base variables
    public static final class TankConstants {
        public static final int RightMasterID = 2;
        public static final int RightSlaveID = 4;
        public static final int LeftMasterID = 1;
        public static final int LeftSlaveID = 3;

        public static final int LeftMotorCount = 2;
        public static final int RightMotorCount = 2;

        public static double ForwardReductionSpeed = 0;
        public static double RotationReductionSpeed = 0;
    }

    // Default controller values
    public static final class ControllerRawButtons {
        public static final int LEFT_X_AXIS = 0;
        public static final int LEFT_Y_AXIS = 1;
        public static final int LEFT_T_AXIS = 2;
        public static final int RIGHT_T_AXIS = 3;
        public static final int RIGHT_X_AXIS = 4;
        public static final int RIGHT_Y_AXIS = 5;

        public static final int BTN_A = 1;
        public static final int BTN_B = 2;
        public static final int BTN_X = 3;
        public static final int BTN_Y = 4;
        public static final int BTN_LB = 5;
        public static final int BTN_RB = 6;
        public static final int BTN_BACK = 7;
        public static final int BTN_START = 8;
        public static final int BTN_LEFT_JOYSTICK = 9;
        public static final int BTN_RIGHT_JOYSTICK = 10;

        public static final int DPAD_NOT_PRESSED = -1;

        public static final int DPAD_NORTH = 0;
        public static final int DPAD_NORTHEAST = 45;
        public static final int DPAD_EAST = 90;
        public static final int DPAD_SOUTHEAST = 135;
        public static final int DPAD_SOUTH = 180;
        public static final int DPAD_SOUTHWEST = 225;
        public static final int DPAD_WEST = 270;
        public static final int DPAD_NORTHWEST = 315;
    }

    public static final class DriverConstants {
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;

        public static final double kDeadband = 0.1;
    }
}

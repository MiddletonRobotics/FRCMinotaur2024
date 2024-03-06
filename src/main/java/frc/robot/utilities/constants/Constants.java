package frc.robot.utilities.constants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

// All of the constants that are accessed by other files, to prevent repetition and allows easy changing

public class Constants {
    public static final class ModuleConstants {

        /* Swerve Voltage Compensation */
        public static final double voltageCompensation = 12.0; // For PID tuning, the max voltage that the PID will compensate for this value (for example at 12V your PID will tune for receiving for 12V, or the max battery output)

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20; // Steering the swerve modules requires less power, and doesn't have a lot of movement, therefore we can reduce the amperes we are feeding it
        public static final int driveContinuousCurrentLimit = 80; // Drive motors should be at the maximum reccomended amperes to get the most power and speed from it

        /* PID Values for the Motors. Used to correct the error when trying to move the motors to a desired location */
        public static final double angleKP = 0.01; // Propotional: If there is error, move the motor propotional to the error
        public static final double angleKI = 0.0; // Intergral: If the error is taking too long to correct, move the motor faster
        public static final double angleKD = 0.0; // Derivative: If the motor is getting close to reaching the target, slow it down
        public static final double angleKFF = 0.0;

        public static final double driveKP = 0.1; // Propotional: If there is error, move the motor propotional to the error
        public static final double driveKI = 0.0; // Intergral: If the error is taking too long to correct, move the motor faster
        public static final double driveKD = 0.0; // Derivative: If the motor is getting close to reaching the target, slow it down
        public static final double driveKFF = 0.0; // Force: Additional gain for creating offsts

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = (SwerveConstants.WheelDiameter * Math.PI) / SwerveConstants.DriveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / SwerveConstants.AngleGearRatio; // Angular movement of freedom (360)

         /* Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.742);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightModule {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.994);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.285);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.849);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class SwerveConstants {
        public static final double TrackWidth = Units.inchesToMeters(21.05); // Distance from center from the right wheels to the left wheels (must be converted to meters)
        public static final double WheelBase = Units.inchesToMeters(21.05); // Distance from center from the front wheels to the back wheels (must be converted to meters)
        public static final double WheelDiameter = Units.inchesToMeters(4.0); // Diameter of the wheel attached to the swerve modules (must be converted to meters)
        public static final double WheelCircumference = WheelDiameter * Math.PI; // To get the circumference multiply the diameter by PI

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double DriveGearRatio = (6.86 / 1.0); // Our MK3 modules are configured with the fast gear ratio (6.86:1)
        public static final double AngleGearRatio = (12.8 / 1.0); // Steering ratio on the MK3 modules is 12.8:1

        /* Swerve Kinematics generated by defining the locations of the modules from the center of the robot (if wrong movement by translation will still work, but the rotation will be messed up) */
        public static final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(WheelBase / 2, TrackWidth / 2), // Divide TrackWidth and WheelBase to define where the module is (+, +)
            new Translation2d(WheelBase / 2, -TrackWidth / 2), // Divide TrackWidth and WheelBase to define where the module is (+, -)
            new Translation2d(-WheelBase / 2, TrackWidth / 2), // Divide TrackWidth and WheelBase to define where the module is (-, +)
            new Translation2d(-WheelBase / 2, -TrackWidth / 2) // Divide TrackWidth and WheelBase to define where the module is (-, -)
        );

        public static final double DriveConversionPositionFactor = (WheelDiameter * Math.PI) / DriveGearRatio;
        public static final double DriveConversionVelocityFactor = DriveConversionPositionFactor / 60.0;
        public static final double AngleConversionFactor = 360.0 / AngleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // Maximum speed in meters per second that the Swerve Modules allow you to go
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake; // What the steering motor should do when not applied with any power (should always be brake while running to prevent overshooting target)
        public static final IdleMode driveNeutralMode = IdleMode.kBrake; // What the drive motor should do when not applied with any power (should always be brake while running to prevent overshooting target)

        public static final IdleMode driveIdleMode = IdleMode.kCoast; // What the drive motor should so when the robot hasn't been initialized
        public static final IdleMode angleIdleMode = IdleMode.kCoast; // What the steering motor should so when the robot hasn't been initialized

        /* Motor and Encoder Inversions, they should all br running in CCW+ (either apply positive power when turned counterclockwise or returning postive values when spun counterclockwise) */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = false; 
        public static final boolean swerveEncoderInverted = false;

        /* The Swerve modules have the encoders and magnets all installed in different orientations, so these values will be the average returned by the CANCoders */
        public static final double FrontLeftCANcoderOffsetRotations = 0;
        public static final double BackLeftCANcoderOffsetRotations = 0;
        public static final double FrontRightCANcoderOffsetRotations = 0;
        public static final double BackRightCANcoderOffsetRotations = 0;

        public static final double PhysicalMaxSpeedMetersPerSecond = 2.75;
        public static final double AngularMaxVelocity = 5;
    }

    public static final class IntakeConstants {
        public static final int rollerMotorID = 13;
        public static final int pivotMotorID = 14;
        public static boolean rollerMotorInvert = false;
        public static IdleMode rollerMotorNeutralMode = IdleMode.kBrake;
        public static double voltageCompensation;
        public static boolean pivotMotorInvert = false;
        public static IdleMode pivotMotorNeutralMode = IdleMode.kBrake;
        public static IdleMode rollerMotorIdleMode = IdleMode.kCoast;
        public static IdleMode pivotMotorIdleMode = IdleMode.kCoast;
        public static final int pivotEncoderID = 17;
    }

    public static final class ShooterConstants {
        public static int leftShooterMotorID = 15;
        public static int rightShooterMotorID = 16;
        public static IdleMode rightShooterMotorNeutralMode = IdleMode.kBrake;
        public static IdleMode leftShooterMotorNeutralMode = IdleMode.kBrake;
        public static IdleMode rightShooterMotorIdleMode = IdleMode.kCoast;
        public static IdleMode leftShooterMotorIdleMode = IdleMode.kCoast;
        public static boolean rightShooterMotorInvert = false;
        public static boolean leftShooterMotorInvert = false;
        public static final double voltageCompensation = 12.0;
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

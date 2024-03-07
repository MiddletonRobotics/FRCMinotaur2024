package frc.robot.utilities.constants;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

        /* Drive Motor Conversion Factors */
        public static final double DriveConversionPositionFactor = (WheelDiameter * Math.PI) / DriveGearRatio;
        public static final double DriveConversionVelocityFactor = DriveConversionPositionFactor / 60.0;
        public static final double AngleConversionFactor = 360.0 / AngleGearRatio;

        /* Swerve Profiling Values */
        public static final double PhysicalMaxSpeedMetersPerSecond = 4.4031408; // Maximum speed in meters per second that the Swerve Modules allow you to go
        public static final double AngularMaxVelocity = 11.5; // Maxiumum speed in radians per seconr that the swerve module is able to rotate (6.28 radians per full rotation)

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
    }

    public static final class AutonomousConstants {
        public static final double PhysicalMaxSpeedMetersPerSecond = 2.0;
        public static final double MaxAccelerationMetersPerSecondSquared = 2;
        public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1.3;
        public static final double kPYController = 1.3;
        public static final double kPThetaController = 2;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared);

        public static final PIDConstants TranslationPID = new PIDConstants(0.1, 0.0, 0.0);
        public static final PIDConstants RotationalPID = new PIDConstants(0.01, 0.0, 0.0);
    }

    public static final class IntakeConstants {

        /* Hardware ID from CAN */
        public static final int rollerMotorID = 13; // Motor ID of the motor thats rolls the intake
        public static final int pivotMotorID = 14; // Motor ID of the motor that pivots or rotates the intake to its storage / idle position
        public static final int pivotEncoderID = 17; // Encoder ID of the encoder that has been mounted to the HEX shaft on the pivot plate

        /* Motor and Encoder Inversions */
        public static final boolean rollerMotorInvert = false;
        public static final boolean pivotMotorInvert = false;
        public static final boolean pivotEnocderInverted = false;

       /* Neutral Modes */ 
        public static final IdleMode rollerMotorNeutralMode = IdleMode.kBrake; // What the roller motor should do when not applied with any power (should always be brake while running to prevent overshooting target)
        public static final IdleMode pivotMotorNeutralMode = IdleMode.kBrake; // What the pivot motor should do when not applied with any power (should always be brake while running to prevent overshooting target)

        public static final IdleMode rollerMotorIdleMode = IdleMode.kCoast; // What the roller motor should so when the robot hasn't been initialized
        public static final IdleMode pivotMotorIdleMode = IdleMode.kCoast; // What the pivot motor should so when the robot hasn't been initialized

        /* Intake Voltage Compensation */
        public static final double voltageCompensation = 12; // For PID tuning, the max voltage that the PID will compensate for this value (for example at 12V your PID will tune for receiving for 12V, or the max battery output)
    }

    public static final class ShooterConstants {

        /* Hardware ID from CAN */
        public static final int lowerShooterMotorID = 16; // Motor ID of the motor thats rolls the lower wheels on the shooter
        public static final int upperShooterMotorID = 15; // Motor ID of the motor thats rolls the upper wheels on the shooter

        /* Motor Inversions (to where postive values shooter a game piece out, or CW+) */
        public static final boolean rightShooterMotorInvert = false; 
        public static final boolean leftShooterMotorInvert = false;

        /* Neutral Modes */ 
        public static final IdleMode rightShooterMotorNeutralMode = IdleMode.kBrake; // What the lower shooter motor should do when not applied with any power (should always be brake while running to prevent overshooting target)
        public static final IdleMode leftShooterMotorNeutralMode = IdleMode.kBrake; // What the upper shooter motor should do when not applied with any power (should always be brake while running to prevent overshooting target)

        public static final IdleMode rightShooterMotorIdleMode = IdleMode.kCoast; // What the lower shooter motor should so when the robot hasn't been initialized
        public static final IdleMode leftShooterMotorIdleMode = IdleMode.kCoast; // What the upper shooter motor should so when the robot hasn't been initialized

        /* Spped Profiling */
        public static final double ampScorerSpeed = 0.45;
        public static final double shooterScorerSpeed = 0.8;

        /* Shooter Voltage Compensation */
        public static final double voltageCompensation = 12.0; // For PID tuning, the max voltage that the PID will compensate for this value (for example at 12V your PID will tune for receiving for 12V, or the max battery output)
    }

    public static final class ClimberConstants {

        /* Hardware ID from CAN */
        public static final int LeftClimbMotorID = 1; // Motor ID of the motor thats attched on the left climber of the robot
        public static final int RightClimbMotorID = 2; // Motor ID of the motor thats attached on the right climber of the robot

        /* Climber Current Limiting */
        public static final int rightClimbContinuousCurrentLimit = 60; // Climbers don't need to be running at maximum amperes due to the extension limit / extension length. Runs at 75%.
        public static final int leftClimbContinuousCurrentLimit = 60; // Climbers don't need to be running at maximum amperes due to the extension limit / extension length. Runs at 75%.

        /* Motor Inversions */
        public static final boolean leftClimbInvert = false;
        public static final boolean rightClimbInvert = false;

        /* Neutral Modes */ 
        public static final IdleMode leftClimbNeutralMode = IdleMode.kBrake;
        public static final IdleMode rightClimbNeutralMode = IdleMode.kBrake;

        /* PID Values for the Motors. Used to correct the error when trying to move the motors to a desired location */
        public static final double climbKP = 0.1; // Propotional: If there is error, move the motor propotional to the error
        public static final double climbKI = 0.0; // Intergral: If the error is taking too long to correct, move the motor faster
        public static final double climbKD = 0.0; // Derivative: If the motor is getting close to reaching the target, slow it down
        public static final double climbKFF = 0.0; // Force: Additional gain for creating offsets

        /* Speed Profiling */
        public static final double climbSpeed = 0.5; // Speed to default the climbers at (we have a reduced gearbox)

        /* Climber Voltage Compensation */
        public static final double voltageCompensation = 12.0; // For PID tuning, the max voltage that the PID will compensate for this value (for example at 12V your PID will tune for receiving for 12V, or the max battery output)
    }

    public static final class TankConstants {

        /* Hardware ID from CAN */
        public static final int FrontLeftID = 1; // Motor ID of the motor thats closest to the front in the left gearbox
        public static final int FrontRightID = 2; // Motor ID of the motor thats closest to the front in the right gearbox
        public static final int BackLeftID = 3; // Motor ID of the motor thats closest to the back in the left gearbox
        public static final int BackRightID = 4; // Motor ID of the motor thats closest to the back in the right gearbox

        /* Motor count on gearbox */
        public static final int LeftMotorCount = 2;
        public static final int RightMotorCount = 2;

        /* Speed Profiling */
        public static double ForwardReductionSpeed = 0; // Tuning for the forward speed of our drivetrain, if needed
        public static double RotationReductionSpeed = 0; // Tuning for the rotational speed of our drivetrain, if needed
    }

    public static final class ControllerRawButtons {

        /* Axis Constants for Xbox Controllers */
        public static final int LEFT_X_AXIS = 0;
        public static final int LEFT_Y_AXIS = 1;
        public static final int LEFT_T_AXIS = 2;
        public static final int RIGHT_T_AXIS = 3;
        public static final int RIGHT_X_AXIS = 4;
        public static final int RIGHT_Y_AXIS = 5;

        /* Button Constants for Xbox Controllers */
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

        /* DPAD Angular Constants for Xbox Controllers */
        public static final int DPAD_NORTH = 0;
        public static final int DPAD_NORTHEAST = 45;
        public static final int DPAD_EAST = 90;
        public static final int DPAD_SOUTHEAST = 135;
        public static final int DPAD_SOUTH = 180;
        public static final int DPAD_SOUTHWEST = 225;
        public static final int DPAD_WEST = 270;
        public static final int DPAD_NORTHWEST = 315;
        public static final int DPAD_NOT_PRESSED = -1;
    }

    public static final class DriverConstants {

        /* Default ports for our two Controllers */
        public static final int driverControllerPort = 0; // Driver Controller only controlls the driving and maneuverability of the robot
        public static final int operatorControllerPort = 1; // Operator Controllers helps with all of the other mechanisms and subsystems attached on the robot.

        public static final double kDeadband = 0.1; // Default deband to help with stick drift on the controllers, recorded values we get is usually (+- 0.05)
    }
}

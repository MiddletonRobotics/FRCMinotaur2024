//Import required packages to apply swerve drive to robot.
package frc.robot.subsystems;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.PoseEstimator;

/* Sets up class that assigns motors to each swerve module and get swerving.
* Methods created to handle different actions taken on the controls.
*/
public class SwerveSubsystem extends SubsystemBase {
    private final AHRS gyro;

    private SwerveDriveOdometry swerveOdometry;
    private SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] swerveModules;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(Constants.SwerveConstants.PhysicalMaxAcceleration);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.SwerveConstants.PhysicalMaxAcceleration);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.SwerveConstants.PhysicalMaxAngularAcceleration);

    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain Debugging");
    private double targetAngleTelemetry = 0;
    GenericEntry targetAngleEntry = tab.add("Target Angle", 0).getEntry();
    GenericEntry currentAngleEntry = tab.add("Current Angle", 0).getEntry();

    private Field2d field;

    /**
     * Initialize the SwerveSubsystem with the four seperate modules, along with the gyro mounted to the RoboRIO. Also
     * includes built-in odometry and pathfinding to make life on the driver easier and to map where the robot is on the field
     */

    public SwerveSubsystem() {
        gyro = new AHRS(SPI.Port.kMXP);

        resetHeading();
        
        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.ModuleConstants.FrontLeftModule.constants),
            new SwerveModule(1,Constants.ModuleConstants.FrontRightModule.constants),
            new SwerveModule(2,Constants.ModuleConstants.BackLeftModule.constants),
            new SwerveModule(3,Constants.ModuleConstants.BackRightModule.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.SwerveKinematics, getHeading(), getSwerveModulePositions(), new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
        field = new Field2d();

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetSwerveOdometry,
            this::getRobotRelativeSpeeds, 
            this::driveRobotRelative, 
            new HolonomicPathFollowerConfig(
                Constants.AutonomousConstants.TranslationPID, 
                Constants.AutonomousConstants.RotationalPID, 
                Constants.AutonomousConstants.PhysicalMaxSpeedMetersPerSecond, 
                Constants.AutonomousConstants.DriveBaseRadius, 
                new ReplanningConfig()
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        var tab = Shuffleboard.getTab("Swerve");
        tab.add(this);

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.SwerveKinematics, getHeading(), getSwerveModulePositions(), new Pose2d());

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);
    }

    /**
    * Allows the robot to drive using field oriented view, and the ability to change the view of the robot to robotCentric, as
    * well as support to slow down the drive for testing purposes
    *
    * @param xSpeed Controller input that controls the strafing movement.
    * @param ySpeed Controller input that controls the forward movement.
    * @param rot Controller input that controls the rotational movement.
    * @param robotCentric Changing the view from the front of the field to the front of the robot.
    * @param slowSpeed Enables slow mode for the drivetrain (uesful for testing).
    */

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getHeading())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], false);
        }
    }

    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
            Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond, Constants.SwerveConstants.PhysicalMaxAcceleration,
            Constants.SwerveConstants.PhysicalAngularMaxVelocity, Constants.SwerveConstants.PhysicalAngularMaxVelocity
        );

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }

    public Command goToPoint(Pose2d targetPose) {
        PIDController xController = new PIDController(Constants.ModuleConstants.driveKP, Constants.ModuleConstants.driveKI, Constants.ModuleConstants.driveKD);
        PIDController yController = new PIDController(Constants.ModuleConstants.driveKP, Constants.ModuleConstants.driveKI, Constants.ModuleConstants.driveKD);
        PIDController thetaController = new PIDController(Constants.ModuleConstants.angleKP, Constants.ModuleConstants.angleKI, Constants.ModuleConstants.angleKD);

        return new FunctionalCommand(
            () ->
                System.out.println(String.format("Traveling to x:%s, y:%s, z:%s", targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees())),
            () -> {
                double sX = xController.calculate(getPose().getX(), targetPose.getX());
                double sY = yController.calculate(getPose().getY(), targetPose.getY());
                double sR = thetaController.calculate(getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

                double xSpeedDelivered = translationLimiter.calculate(sX) * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
                double ySpeedDelivered = strafeLimiter.calculate(sY) * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
                double rotDelivered = rotationLimiter.calculate(sR) * Constants.SwerveConstants.PhysicalAngularMaxVelocity;

                drive(new Translation2d(xSpeedDelivered, ySpeedDelivered).times(Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond), rotDelivered * Constants.SwerveConstants.PhysicalAngularMaxVelocity, false, true);
            },
            interrupted -> {
                xController.close();
                yController.close();
                thetaController.close();
            },
            () -> xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint(),
            this
        );
    }

    public Command chasePoseCommand(Supplier<Pose2d> target){
        TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1.5);
        
        ProfiledPIDController xController = new ProfiledPIDController(0.1, 0, 0, X_CONSTRAINTS);
        ProfiledPIDController yController = new ProfiledPIDController(0.1, 0, 0, Y_CONSTRAINTS);
        ProfiledPIDController omegaController = new ProfiledPIDController(0.01, 0, 0, OMEGA_CONSTRAINTS);

        xController.setTolerance(0.3);
        yController.setTolerance(0.3);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-180, 180);

        return new DeferredCommand(() ->
            new RepeatCommand(
                new FunctionalCommand(
                () -> {
                    // Init
                },
                () -> {
                    double xSpeed = xController.calculate(this.getPose().getX(), target.get().getX());
                    double ySpeed = yController.calculate(this.getPose().getY(), target.get().getY());
                    double omegaSpeed = omegaController.calculate(this.getRawHeading(), target.get().getRotation().getDegrees());

                    double xSpeedDelivered = translationLimiter.calculate(xSpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);
                    double ySpeedDelivered = strafeLimiter.calculate(ySpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);
                    double rotDelivered = rotationLimiter.calculate(omegaSpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);

                    drive(new Translation2d(xSpeedDelivered, ySpeedDelivered).times(Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond), rotDelivered * Constants.SwerveConstants.PhysicalAngularMaxVelocity, false, true);
                },
                interrupted -> {
                    this.drive(new Translation2d(0, 0),0, true, true);
                    System.out.println("30 cm away now");
                },
                () -> {
                    return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
                },
                this)
            ), Set.of(this)
        );
    }

    public Command chasePoseRobotRelativeCommand(Supplier<Pose2d> target){
        TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        //TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1.5);
        
        ProfiledPIDController xController = new ProfiledPIDController(0.5, 0, 0, X_CONSTRAINTS);
        ProfiledPIDController yController = new ProfiledPIDController(0.5, 0, 0, Y_CONSTRAINTS);
        PIDController omegaPID = new PIDController(0.01, 0, 0);
    
        xController.setTolerance(0.10);
        yController.setTolerance(0.03);
        omegaPID.setTolerance(1.5);
        omegaPID.enableContinuousInput(-180, 180);
    
        return new DeferredCommand(() ->
            new FunctionalCommand(
                () -> {
                    // Init
                },
                () -> {
                    double xSpeed = xController.calculate(0, target.get().getX());
                    double ySpeed = yController.calculate(0, target.get().getY());
                    double omegaSpeed = omegaPID.calculate(0, target.get().getRotation().getDegrees());
        
                    double xSpeedDelivered = translationLimiter.calculate(xSpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);
                    double ySpeedDelivered = strafeLimiter.calculate(ySpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);
                    double rotDelivered = rotationLimiter.calculate(omegaSpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);

                    drive(new Translation2d(xSpeedDelivered, ySpeedDelivered).times(Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond), rotDelivered * Constants.SwerveConstants.PhysicalAngularMaxVelocity, true, true);
                },
                interrupted -> {
                    this.drive(new Translation2d(0, 0),0, true, true);
                    omegaPID.close();
                    System.out.println("Aligned now");
                },
        
                () -> {
                    return omegaPID.atSetpoint() && xController.atGoal() && yController.atGoal();
                },
                this
            ), Set.of(this)
        );
    }


    public Command followUnflippedPathCommand(PathPlannerPath pathName){
      return new FollowPathHolonomic(
            pathName,
            this::getPose, // Robot pose supplier
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond, // Max module speed, in m/s
                Constants.AutonomousConstants.DriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
            ),
            () -> {
                return false;
            },
            this
        );
    }

    public Command pathFindThenFollowPathCommand(String pathName){
    return new PathfindThenFollowPathHolonomic(
        PathPlannerPath.fromPathFile(pathName),
        new PathConstraints(
                Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond,
                Constants.SwerveConstants.PhysicalMaxAcceleration,
                Constants.SwerveConstants.PhysicalAngularMaxVelocity,
                Constants.SwerveConstants.PhysicalMaxAngularAcceleration),
        this::getPose,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                // in
                // your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond, // Max module speed, in m/s
                Constants.AutonomousConstants.DriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig(true, true) // Default path replanning config. See the API for the
            // options
            // here
            ),
        1.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
            
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            getShouldFlip(),
            
        this // Reference to drive subsystem to set requirements
      );
  }


    public Command onTheFlyPathCommand(Supplier<Pose2d> targetPose) {
        return new DeferredCommand(() -> followUnflippedPathCommand(
            new PathPlannerPath(
                PathPlannerPath.bezierFromPoses(new Pose2d(this.getPose().getTranslation(),
                                                    Rotation2d.fromDegrees(0)),
                                                targetPose.get()),
                new PathConstraints(
                    Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond,
                    Constants.SwerveConstants.PhysicalMaxAcceleration,
                    Constants.SwerveConstants.PhysicalAngularMaxVelocity,
                    Constants.SwerveConstants.PhysicalMaxAngularAcceleration),
                new GoalEndState(0, targetPose.get().getRotation()),
                false)),
            //.until(
                // () -> targetPose.get().getTranslation().getDistance(PoseEstimator.getInstance().getPosition().getTranslation())<1.5))
                //.finallyDo(() -> System.out.println("uwu owo"));
        Set.of(this));
    }

    public Command chaseThenOnTheFlyCommand(Supplier<Pose2d> target){
        return new SequentialCommandGroup(chasePoseCommand(target), onTheFlyPathCommand(target));
    }

    public Command alignCommand(Supplier<Translation2d> target){
        PIDController pid = new PIDController(0.01, 0, 0);
        pid.setTolerance(1.5);
        pid.enableContinuousInput(-180, 180);
        return new DeferredCommand(() ->
            new FunctionalCommand(
                () -> {
                // Init
                },
                () -> {
                    Translation2d currentTranslation = this.getPose().getTranslation();
                    Translation2d targetVector = currentTranslation.minus(target.get());
                    Rotation2d targetAngle = targetVector.getAngle();
                    double newSpeed;
                    // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                    if(Constants.DriverConstants.IS_ALLIANCE_RED)
                    newSpeed = pid.calculate(this.getRawHeading(), targetAngle.getDegrees());
                    else
                    newSpeed = pid.calculate(this.getRawHeading(), targetAngle.getDegrees());
                    this.drive(new Translation2d(0, 0),0, false, true);
        
                    targetAngleEntry.setDouble(targetAngle.getDegrees());
                    currentAngleEntry.setDouble(this.getRawHeading());
                },
                interrupted -> {
                    pid.close();
                    //this.drive(0,0,0,true,true);
                    System.out.println("Allignment Over");  
                },
                () -> {
                    return pid.atSetpoint();
                },
                this), 
            Set.of(this)
        );
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states =  Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);
        setModuleStates(states);
    }

    public void xLock() { 
        SwerveModuleState[] swerveModuleStates = {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
        };

        for (SwerveModule module : swerveModules) {
            module.setDesiredStateForXlock(swerveModuleStates[module.moduleNumber], true);
        } 
  }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module : swerveModules) {
            states[module.moduleNumber] = module.getSwerveModuleState();
        }

        return states;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule module : swerveModules) {
            positions[module.moduleNumber] = module.getSwerveModulePosition();
        }

        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.SwerveConstants.SwerveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for (SwerveModule module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], true);
        } 
    }

    public BooleanSupplier getShouldFlip() {
        return () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };
    }

    public void startingOdometry(Pose2d startingPose) {
        PoseEstimator.getInstance().resetPoseEstimate(new Pose2d(startingPose.getX(), startingPose.getY(), this.getHeading()));
    }

    public void resetSwerveOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getHeading(), getSwerveModulePositions(), pose);
    }

    public Pose2d getPose() {
       return swervePoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public double getRawHeading() {
        return -Double.valueOf(((AHRS)gyro).getYaw());
    }

    public double getTurnRate() {
        return gyro.getRate() * (Constants.SwerveConstants.gyroInverted ? -1.0 : 1.0);
      }


    public void resetModulesToAbsolute() {
        for(SwerveModule module : swerveModules) {
            module.resetToAbsolute();
        }
    }

    public void resetHeading() {
        gyro.zeroYaw();
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getHeading(), new SwerveModulePosition[] {
            swerveModules[0].getSwerveModulePosition(), 
            swerveModules[1].getSwerveModulePosition(), 
            swerveModules[2].getSwerveModulePosition(), 
            swerveModules[3].getSwerveModulePosition()
        });

        targetAngleEntry.setDouble(targetAngleTelemetry);
        currentAngleEntry.setDouble(getRawHeading() % 360);

        double measuredStates[] = {
            swerveModules[0].getSwerveModuleState().angle.getDegrees(),
            swerveModules[0].getSwerveModuleState().speedMetersPerSecond,
            swerveModules[1].getSwerveModuleState().angle.getDegrees(),
            swerveModules[1].getSwerveModuleState().speedMetersPerSecond,
            swerveModules[2].getSwerveModuleState().angle.getDegrees(),
            swerveModules[2].getSwerveModuleState().speedMetersPerSecond,
            swerveModules[3].getSwerveModuleState().angle.getDegrees(),
            swerveModules[3].getSwerveModuleState().speedMetersPerSecond,
        };

        double desiredStates[] = {
            swerveModules[0].getDesiredState().angle.getDegrees(),
            swerveModules[0].getDesiredState().speedMetersPerSecond,
            swerveModules[1].getDesiredState().angle.getDegrees(),
            swerveModules[1].getDesiredState().speedMetersPerSecond,
            swerveModules[2].getDesiredState().angle.getDegrees(),
            swerveModules[2].getDesiredState().speedMetersPerSecond,
            swerveModules[3].getDesiredState().angle.getDegrees(),
            swerveModules[3].getDesiredState().speedMetersPerSecond,
        };

        double loggingEncoders[] = {
            swerveModules[0].getSwerveEncoder().getDegrees(),
            swerveModules[1].getSwerveEncoder().getDegrees(),
            swerveModules[2].getSwerveEncoder().getDegrees(),
            swerveModules[3].getSwerveEncoder().getDegrees(),
        };

        SmartDashboard.putNumberArray("MeasuredSwerveStates", measuredStates);
        SmartDashboard.putNumberArray("DesiredSwerveStates", desiredStates);

        SmartDashboard.putNumber("Front-Left Encoder Position", loggingEncoders[0]);
        SmartDashboard.putNumber("Front-Right Encoder Position", loggingEncoders[1]);
        SmartDashboard.putNumber("Back-Left Encoder Position", loggingEncoders[2]);
        SmartDashboard.putNumber("Back-Right Encoder Position", loggingEncoders[3]);

        SmartDashboard.putNumber("NavX Yaw Value", getHeading().getDegrees());
  }
}

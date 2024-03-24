//Import required packages to apply swerve drive to robot.
package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.constants.Constants;

/* Sets up class that assigns motors to each swerve module and get swerving.
* Methods created to handle different actions taken on the controls.
*/
public class SwerveSubsystem extends SubsystemBase {
    private final AHRS gyro;

    private SwerveDriveOdometry swerveOdometry;
    private SwerveDrivePoseEstimator swervePoseEstimator;
    private SwerveModule[] swerveModules;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(Constants.SwerveConstants.PhysicalMaxAcceleration);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.SwerveConstants.PhysicalMaxAcceleration);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.SwerveConstants.PhysicalMaxAngularAcceleration);

    private Field2d field;

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

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.SwerveKinematics, getHeading(), getSwerveModulePositions(), getPose());

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean robotCentric, boolean slowSpeed) {
        double xSpeedDelivered, ySpeedDelivered, rotDelivered;
    
        if (slowSpeed) {
          // Convert the commanded speeds into the correct units for the drivetrain
          xSpeedDelivered = translationLimiter.calculate(xSpeed * 0.3) * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
          ySpeedDelivered = strafeLimiter.calculate(ySpeed * 0.3) * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
          rotDelivered = rotationLimiter.calculate(rot * 0.3) * Constants.SwerveConstants.PhysicalAngularMaxVelocity;
    
        } else {
          // Convert the commanded speeds into the correct units for the drivetrain
          xSpeedDelivered = translationLimiter.calculate(xSpeed) * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
          ySpeedDelivered = strafeLimiter.calculate(ySpeed) * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
          rotDelivered = rotationLimiter.calculate(rot) * Constants.SwerveConstants.PhysicalAngularMaxVelocity;
    
        }
    
        var swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(
            robotCentric ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeading()) : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
            setModuleStates(swerveModuleStates);
      }

    /*

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], false);
        }
    }

    public void goStraight(Translation2d translation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(new ChassisSpeeds(translation.getX(), translation.getY(), 0.0));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for (SwerveModule module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
        }
    }

    */

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
            module.setDesiredState(desiredStates[module.moduleNumber]);
        } 
      }

    /*

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }

    */

    public Pose2d getPose() {
       return swervePoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public double getTurnRate() {
        return gyro.getRate() * (Constants.SwerveConstants.gyroInverted ? -1.0 : 1.0);
      }

    public void resetSwerveOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getHeading(), getSwerveModulePositions(), pose);
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule module : swerveModules) {
            module.resetToAbsolute();
        }
    }

    public void resetHeading() {
        gyro.zeroYaw();
    }

    public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
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

    @Override
    public void periodic() {

        swerveOdometry.update(getHeading(), getSwerveModulePositions());
        field.setRobotPose(getPose());

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

//Import required packages to apply swerve drive to robot.
package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

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
    private SwerveModule[] swerveModules;

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

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.SwerveKinematics, getYaw(), getSwerveModulePositions());
        field = new Field2d();

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetSwerveOdometry,
            this::getRobotRelativeSpeeds, 
            this::driveRobotRelative, 
            new HolonomicPathFollowerConfig(
                Constants.AutonomousConstants.TranslationPID, 
                Constants.AutonomousConstants.RotationalPID, 
                Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond, 
                Constants.SwerveConstants.TrackWidth / 2, 
                new ReplanningConfig()
            ), 
            () -> {
                return false;
            },
            this
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);
    }

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

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states =  Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);
        setModuleStates(states);
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

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
       return swerveOdometry.getPoseMeters();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(),360)*(Constants.SwerveConstants.swerveEncoderInverted ? -1.0 : 1.0);
    }

    public Rotation2d getYaw() {
        return (Constants.SwerveConstants.swerveEncoderInverted) ? Rotation2d.fromDegrees(getHeading()) : Rotation2d.fromDegrees(360-getHeading());
    }

    public void resetSwerveOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getSwerveModulePositions(), pose);
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

        swerveOdometry.update(getYaw(), getSwerveModulePositions());
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

        SmartDashboard.putNumber("NavX Yaw Value", getYaw().getDegrees());
  }
}

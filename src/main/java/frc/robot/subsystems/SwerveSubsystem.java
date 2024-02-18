//Import required packages to apply swerve drive to robot.
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        // swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.SwerveKinematics, getYaw(), getSwerveModulePositions());
        
        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.ModuleConstants.FrontLeftModule.constants),
            new SwerveModule(1,Constants.ModuleConstants.FrontRightModule.constants),
            new SwerveModule(2,Constants.ModuleConstants.BackLeftModule.constants),
            new SwerveModule(3,Constants.ModuleConstants.BackRightModule.constants)
        };

        field = new Field2d();
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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }

    /*

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetSwerveOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getSwerveModulePositions(), pose);
    }

    */

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module : swerveModules) {
            states[module.moduleNumber] = module.getSwerveModuleState();
        }

        return states;
    }

    public void resetHeading() {
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (Constants.SwerveConstants.swerveEncoderInverted)? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {
        // field.setRobotPose(getPose());

        for (SwerveModule module : swerveModules) {
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Cancoder", module.getSwerveEncoder().getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Integrated", module.getSwerveModuleState().angle.getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Velocity", module.getSwerveModuleState().speedMetersPerSecond);
    }
  }
}

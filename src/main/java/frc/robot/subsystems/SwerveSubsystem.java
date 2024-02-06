package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.utilities.constants.Constants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule FrontLeftSwerveModule = new SwerveModule(
        Constants.SwerveConstants.FrontLeftForwardMotorID, 
        Constants.SwerveConstants.FrontLeftThetaMotorID, 
        Constants.SwerveConstants.FrontLeftForwardEncoderReversed, 
        Constants.SwerveConstants.FrontLeftThetaEncoderReversed, 
        Constants.SwerveConstants.FrontLeftCANcoderID, 
        Constants.SwerveConstants.FrontLeftCANcoderOffsetRadians, 
        Constants.SwerveConstants.FrontLeftCANcoderReversed
    );

    private final SwerveModule FrontRightSwerveModule = new SwerveModule(
        Constants.SwerveConstants.FrontRightForwardMotorID, 
        Constants.SwerveConstants.FrontRightThetaMotorID, 
        Constants.SwerveConstants.FrontRightForwardEncoderReversed, 
        Constants.SwerveConstants.FrontRightThetaEncoderReversed, 
        Constants.SwerveConstants.FrontRightCANcoderID, 
        Constants.SwerveConstants.FrontRightCANcoderOffsetRadians, 
        Constants.SwerveConstants.FrontRightCANcoderReversed
    );

    private final SwerveModule BackLeftSwerveModule = new SwerveModule(
        Constants.SwerveConstants.BackLeftForwardMotorID, 
        Constants.SwerveConstants.BackLeftThetaMotorID, 
        Constants.SwerveConstants.BackLeftForwardEncoderReversed, 
        Constants.SwerveConstants.BackLeftThetaEncoderReversed, 
        Constants.SwerveConstants.BackLeftCANcoderID, 
        Constants.SwerveConstants.BackLeftCANcoderOffsetRadians, 
        Constants.SwerveConstants.BackLeftCANcoderReversed
    );

    private final SwerveModule BackRightSwerveModule = new SwerveModule(
        Constants.SwerveConstants.BackRightForwardMotorID, 
        Constants.SwerveConstants.BackRightThetaMotorID, 
        Constants.SwerveConstants.BackRightForwardEncoderReversed, 
        Constants.SwerveConstants.BackRightThetaEncoderReversed, 
        Constants.SwerveConstants.BackRightCANcoderID, 
        Constants.SwerveConstants.BackRightCANcoderOffsetRadians, 
        Constants.SwerveConstants.BackRightCANcoderReversed
    );

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(5000);
                zeroHeading();
            } catch (Exception e) {}
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopSwerveModules() {
        FrontLeftSwerveModule.stop();
        FrontRightSwerveModule.stop();
        BackLeftSwerveModule.stop();
        BackRightSwerveModule.stop();
    }

    public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        FrontLeftSwerveModule.setDesiredState(desiredStates[0]);
        FrontRightSwerveModule.setDesiredState(desiredStates[1]);
        BackLeftSwerveModule.setDesiredState(desiredStates[2]);
        BackRightSwerveModule.setDesiredState(desiredStates[3]);
    }

    public Command resetHeading() {
        return run(() -> zeroHeading());
    }
}

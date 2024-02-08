package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.constants.Constants;

public class SwerveController extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> forwardSpeed, strafeSpeed, rotationSpeed;
    private final Supplier<Boolean> fieldRelative;
    private final SlewRateLimiter forwardLimiter, strafeLimiter, rotationLimiter;

    public SwerveController(SwerveSubsystem swerveSubsystem, Supplier<Double> forwardSpeed, Supplier<Double> strafeSpeed, Supplier<Double> rotationSpeed, Supplier<Boolean> fieldRelative) {
        this.swerveSubsystem = swerveSubsystem;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.rotationSpeed = rotationSpeed;
        this.fieldRelative = fieldRelative;
        this.forwardLimiter = new SlewRateLimiter(Constants.SwerveConstants.TeleopMaxAccelerationUnitsPerSecond);
        this.strafeLimiter = new SlewRateLimiter(Constants.SwerveConstants.TeleopMaxAccelerationUnitsPerSecond);
        this.rotationLimiter = new SlewRateLimiter(Constants.SwerveConstants.TeleopMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double forward = forwardSpeed.get();
        double strafe = strafeSpeed.get();
        double rotation = rotationSpeed.get();

        forward = Math.abs(forward) > Constants.DriverConstants.kDeadband ? forward : 0;
        strafe = Math.abs(strafe) > Constants.DriverConstants.kDeadband ? strafe : 0;
        rotation = Math.abs(rotation) > Constants.DriverConstants.kDeadband ? rotation : 0;

        forward = forwardLimiter.calculate(forward) * Constants.SwerveConstants.TeleopMaxSpeedMetersPerSecond;
        strafe = strafeLimiter.calculate(strafe) * Constants.SwerveConstants.TeleopMaxSpeedMetersPerSecond;
        rotation = rotationLimiter.calculate(rotation) * Constants.SwerveConstants.TeleopMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;

        if(fieldRelative.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(forward, strafe, rotation);           
        }

        SwerveModuleState[] moduleStates = Constants.SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setSwerveModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
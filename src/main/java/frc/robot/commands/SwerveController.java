package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utilities.constants.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;

public class SwerveController extends Command {
    private SwerveSubsystem swerveSubsystem;
    private CommandXboxController driverController;
    private BooleanSupplier robotCentricSupplier;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);

    public SwerveController(SwerveSubsystem swerveSubsystem, CommandXboxController driverController, BooleanSupplier robotCentricSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.driverController = driverController;
        this.robotCentricSupplier = robotCentricSupplier;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double multiplier = driverController.rightBumper().getAsBoolean() ? 0.4 : 1.0;
        double omega = 
            Constants.SwerveConstants.MaximumRotationPercent * 
            Constants.SwerveConstants.PhysicalAngularMaxVelocity * 
            MathUtil.applyDeadband(driverController.getRightX(), Constants.DriverConstants.kDeadband) * multiplier;

        Translation2d strafeVecter =
            new Translation2d(
                Constants.SwerveConstants.MaximumTranslationPercent
                    * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond
                    * MathUtil.applyDeadband(-driverController.getLeftY(), 0.05)
                    * multiplier,
                Constants.SwerveConstants.MaximumTranslationPercent
                    * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond
                    * MathUtil.applyDeadband(driverController.getLeftX(), 0.05)
                    * multiplier
            ).rotateBy(Rotation2d.fromDegrees(90.0));


        swerveSubsystem.drive(strafeVecter.getX(), strafeVecter.getY(), omega, true);
    }
}
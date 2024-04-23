package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.utilities.constants.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveController extends Command {
    private SwerveSubsystem swerveSubsystem;
    private DoubleSupplier translationSupplier;
    private DoubleSupplier strafeSupplier;
    private DoubleSupplier rotationSupplier;
    private BooleanSupplier robotCentricSupplier;
    private BooleanSupplier isSlowMode;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);

    private enum SpeedController {
        FAST,
        SLOW
    }

    private SpeedController speedControl = SpeedController.FAST;

    public SwerveController(SwerveSubsystem swerveSubsystem, DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier robotCentricSupplier, BooleanSupplier isSlowMode) {
        this.swerveSubsystem = swerveSubsystem;
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
        this.isSlowMode = isSlowMode;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        boolean leftBumperButtonPreviousState = false;

        double translationValue = translationLimiter.calculate(MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.DriverConstants.kDeadband));
        double strafeValue = strafeLimiter.calculate(MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.DriverConstants.kDeadband));
        double rotationValue = rotationLimiter.calculate(MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.DriverConstants.kDeadband));

        if(isSlowMode.getAsBoolean() && speedControl == SpeedController.FAST && !leftBumperButtonPreviousState) {
            speedControl = SpeedController.SLOW;
            swerveSubsystem.drive(new Translation2d(translationValue, strafeValue).times(Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.4), rotationValue * Constants.SwerveConstants.PhysicalAngularMaxVelocity * 0.4, !robotCentricSupplier.getAsBoolean(), true);
        } else if(isSlowMode.getAsBoolean() && speedControl == SpeedController.SLOW && !leftBumperButtonPreviousState) {
            speedControl = SpeedController.FAST;
        } else {
            swerveSubsystem.drive(new Translation2d(translationValue, strafeValue).times(Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond), rotationValue * Constants.SwerveConstants.PhysicalAngularMaxVelocity, !robotCentricSupplier.getAsBoolean(), true);
        }

        leftBumperButtonPreviousState = isSlowMode.getAsBoolean();
    }
}
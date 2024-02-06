package frc.robot.commands;

import frc.robot.utilities.constants.Constants;
import frc.robot.subsystems.TankSubsystem;
import frc.robot.utilities.Controller;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TankController extends Command {
    private final TankSubsystem TankDrive;

    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier turningSpeed;
    private double ForwardReduction = Constants.TankConstants.ForwardReductionSpeed;
    private double RotationReduction = Constants.TankConstants.RotationReductionSpeed;

    public TankController(TankSubsystem TankDrive, DoubleSupplier forwardSpeed, DoubleSupplier turningSpeed) {
        this.TankDrive = TankDrive;
        this.forwardSpeed = forwardSpeed;
        this.turningSpeed = turningSpeed;

        addRequirements(TankDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        TankDrive.ArcadeDrive(-(forwardSpeed.getAsDouble()), turningSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        TankDrive.stopTankDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

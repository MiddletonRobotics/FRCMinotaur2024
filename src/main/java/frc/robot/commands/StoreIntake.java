package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StoreIntake extends Command {
    private IntakeSubsystem intakeSubsystem;

    public StoreIntake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.storePosition();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.reset();
    }
}

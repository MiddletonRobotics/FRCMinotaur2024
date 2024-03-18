package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePull extends Command {
    private IntakeSubsystem intakeSubsystem;

    public IntakePull(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeConsume();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.reset();
    }
}

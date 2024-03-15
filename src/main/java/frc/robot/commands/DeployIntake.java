package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
    private IntakeSubsystem intakeSubsystem;

    public DeployIntake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.deployPosition();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.reset();
    }
}

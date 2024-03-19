package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDown extends Command {
    private ClimberSubsystem climberSubsystem;

    public ClimberDown(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.climbDown();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.reset();
    }
}

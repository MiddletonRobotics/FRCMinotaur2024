package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUp extends Command {
    private ClimberSubsystem climberSubsystem;

    public ClimberUp(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.climbUp();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.reset();
    }
}

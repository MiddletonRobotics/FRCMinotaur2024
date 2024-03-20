package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class UpClimb extends Command {
    private ClimberSubsystem climberSubsystem;

    public UpClimb(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.climbUp();
        Timer.delay(2);
        climberSubsystem.reset();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.reset();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utilities.constants.Constants;


public class ClimberController extends Command {
    private ClimberSubsystem climberSubsystem;

    public ClimberController(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.climbUp();
        //thingy
        climberSubsystem.climbDown();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.reset();
    }
}

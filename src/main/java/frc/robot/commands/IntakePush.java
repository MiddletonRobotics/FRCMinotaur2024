package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePush extends Command {
    private IntakeSubsystem intakeSubsystem;

    public IntakePush(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeRegurgitate();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.reset();
    }
}

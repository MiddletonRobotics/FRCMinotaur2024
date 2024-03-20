package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class IntakeNote extends Command {
    private IntakeSubsystem intakeSubsystem;

    public IntakeNote(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    public void execute() {
        intakeSubsystem.deployIntake();
        Timer.delay(1); //maybe change
        intakeSubsystem.intakeConsume();
        Timer.delay(1); //maybe change
        intakeSubsystem.storeIntake();
        Timer.delay(1); //maybe change
        intakeSubsystem.reset();
    }
}

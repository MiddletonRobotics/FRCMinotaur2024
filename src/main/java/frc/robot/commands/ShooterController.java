package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterController extends Command {

    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    
    public ShooterController(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.shooterSpeakerScoring();
        Timer.delay(2);
        intakeSubsystem.intakeToShooter();
        Timer.delay(1.5);
        shooterSubsystem.stopShooter();
        intakeSubsystem.stopIntake();
    }

    public void end() {
        shooterSubsystem.stopShooter();
    }
    
}

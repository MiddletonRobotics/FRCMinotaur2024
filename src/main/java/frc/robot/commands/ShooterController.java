package frc.robot.commands;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    public void initialize() {
        new Timer();
    }

    @Override
    public void execute() {
        shooterSubsystem.shooterSpeakerScoring();
        new WaitCommand(1);
        intakeSubsystem.outtakeToShooter();
        new WaitCommand(2);
        shooterSubsystem.stopShooter();
        intakeSubsystem.stopIntaking();
    }

    public void end() {
        shooterSubsystem.stopShooter();
        intakeSubsystem.stopIntaking();
    }
}

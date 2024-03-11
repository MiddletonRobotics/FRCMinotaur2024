package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpController extends Command {

    private ShooterSubsystem shooterSubsystem;
    
    public AmpController(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.shooterAmpScoring();
        Timer.delay(3);
        shooterSubsystem.stopShooter();
    }

    public void end() {
        shooterSubsystem.stopShooter();
    }
    
}

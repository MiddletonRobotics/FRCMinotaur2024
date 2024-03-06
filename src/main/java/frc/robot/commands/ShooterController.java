package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterController extends Command {

    private ShooterSubsystem shooterSubsystem;
    
    public ShooterController(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.shooterSlowSpin();
    }

    public void end() {
        shooterSubsystem.shooterStop();
    }
    
}

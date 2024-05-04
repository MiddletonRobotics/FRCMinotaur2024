package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utilities.CANSparkMaxUtil;
import frc.robot.utilities.CANSparkMaxUtil.Usage;
import frc.robot.utilities.constants.Constants;

public class ShooterSubsystem extends SubsystemBase {
    
    public CANSparkMax lowerShooterMotor;
    public CANSparkMax upperShooterMotor;

    public ShooterSubsystem() {
        lowerShooterMotor = new CANSparkMax(Constants.ShooterConstants.lowerShooterMotorID, MotorType.kBrushless);
        upperShooterMotor = new CANSparkMax(Constants.ShooterConstants.upperShooterMotorID, MotorType.kBrushless);

        configureLowerShooterMotor();
        configureUpperShooterMotor();
    }

    public void configureLowerShooterMotor() {
        lowerShooterMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(lowerShooterMotor, Usage.kAll);
        lowerShooterMotor.setInverted(Constants.ShooterConstants.lowerShooterMotorInvert);
        lowerShooterMotor.setIdleMode(Constants.ShooterConstants.lowerShooterMotorNeutralMode);
        lowerShooterMotor.enableVoltageCompensation(Constants.ShooterConstants.voltageCompensation);
        lowerShooterMotor.burnFlash();
    }

    public void configureUpperShooterMotor() {
        upperShooterMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(upperShooterMotor, Usage.kAll);
        upperShooterMotor.setInverted(Constants.ShooterConstants.upperShooterMotorInvert);
        upperShooterMotor.setIdleMode(Constants.ShooterConstants.upperShooterMotorNeutralMode);
        upperShooterMotor.enableVoltageCompensation(Constants.ShooterConstants.voltageCompensation);
        upperShooterMotor.burnFlash();
    }

    public void shooterAmpScoring() {
        setSpeed(Constants.ShooterConstants.ampScorerSpeed);
    }

    public void shooterSpeakerScoring() {
        setSpeed(Constants.ShooterConstants.speakerScorerSpeed);
    }

    public void setSpeed(double speed) {
        lowerShooterMotor.set(speed);
        upperShooterMotor.set(speed);
    }

    public void stopShooter() {
        lowerShooterMotor.set(0.0);
        upperShooterMotor.set(0.0);
    }

    public void shooterOnDisabled() {
        lowerShooterMotor.setIdleMode(Constants.ShooterConstants.lowerShooterMotorNeutralMode);
        upperShooterMotor.setIdleMode(Constants.ShooterConstants.upperShooterMotorNeutralMode);
    }

    public SequentialCommandGroup shooterAmpScoringCommand(IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(new ParallelCommandGroup( 
            new RunCommand(() -> shooterAmpScoring()), 
            new WaitCommand(0.75), 
            new RunCommand(() -> intakeSubsystem.outtakeToShooter()),
            new WaitCommand(1.5),
            new RunCommand(() -> stopShooter()).alongWith(new RunCommand(() -> intakeSubsystem.stopIntaking()))
        ));
    }

    public SequentialCommandGroup shooterSpeakerScoringCommand(IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(new ParallelCommandGroup( 
            new RunCommand(() -> shooterSpeakerScoring()), 
            new WaitCommand(0.75), 
            new RunCommand(() -> intakeSubsystem.outtakeToShooter()).alongWith(),
            new WaitCommand(1.5),
            new RunCommand(() -> stopShooter()).alongWith(new RunCommand(() -> intakeSubsystem.stopIntaking()))
        ));
    }

    @Override
    public void periodic() {

    }
}

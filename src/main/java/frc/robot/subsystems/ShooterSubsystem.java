package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        lowerShooterMotor.setInverted(Constants.ShooterConstants.rightShooterMotorInvert);
        lowerShooterMotor.setIdleMode(Constants.ShooterConstants.rightShooterMotorNeutralMode);
        lowerShooterMotor.enableVoltageCompensation(Constants.ShooterConstants.voltageCompensation);
        lowerShooterMotor.burnFlash();
    }

    public void configureUpperShooterMotor() {
        upperShooterMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(upperShooterMotor, Usage.kAll);
        upperShooterMotor.setInverted(Constants.ShooterConstants.leftShooterMotorInvert);
        upperShooterMotor.setIdleMode(Constants.ShooterConstants.leftShooterMotorNeutralMode);
        upperShooterMotor.enableVoltageCompensation(Constants.ShooterConstants.voltageCompensation);
        upperShooterMotor.burnFlash();
    }

    public void shooterSlowSpin() {
        lowerShooterMotor.set(Constants.ShooterConstants.ampScorerSpeed);
        upperShooterMotor.set(Constants.ShooterConstants.ampScorerSpeed);
    }

    public void shooterFastSpin() {
        lowerShooterMotor.set(Constants.ShooterConstants.shooterScorerSpeed);
        upperShooterMotor.set(Constants.ShooterConstants.shooterScorerSpeed);
    }

    public void shooterStop() {
        lowerShooterMotor.set(0.0);
        upperShooterMotor.set(0.0);
    }

    @Override
    public void periodic() {

    }
}

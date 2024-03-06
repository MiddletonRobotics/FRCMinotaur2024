package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CANSparkMaxUtil;
import frc.robot.utilities.CANSparkMaxUtil.Usage;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    
    public CANSparkMax rightShooterMotor;
    public CANSparkMax leftShooterMotor;

    public ShooterSubsystem() {
        rightShooterMotor = new CANSparkMax(ShooterConstants.rightShooterMotorID, MotorType.kBrushless);
        leftShooterMotor = new CANSparkMax(ShooterConstants.leftShooterMotorID, MotorType.kBrushless);
    }

    public void shooterSlowSpin() {
        rightShooterMotor.set(0.45);
        leftShooterMotor.set(0.45);
    }

    public void shooterFastSpin() {
        rightShooterMotor.set(0.8);
        leftShooterMotor.set(0.8);
    }

    public void shooterStop() {
        rightShooterMotor.set(0.0);
        leftShooterMotor.set(0.0);
    }

    public void configureRightShooterMotor() {
        rightShooterMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(rightShooterMotor, Usage.kAll);
        rightShooterMotor.setInverted(Constants.ShooterConstants.rightShooterMotorInvert);
        rightShooterMotor.setIdleMode(Constants.ShooterConstants.rightShooterMotorNeutralMode);
        rightShooterMotor.enableVoltageCompensation(Constants.ShooterConstants.voltageCompensation);
        rightShooterMotor.burnFlash();
    }

    public void configureLeftShooterMotor() {
        leftShooterMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(leftShooterMotor, Usage.kAll);
        leftShooterMotor.setInverted(Constants.ShooterConstants.leftShooterMotorInvert);
        leftShooterMotor.setIdleMode(Constants.ShooterConstants.leftShooterMotorNeutralMode);
        leftShooterMotor.enableVoltageCompensation(Constants.ShooterConstants.voltageCompensation);
        leftShooterMotor.burnFlash();
    }

    @Override
    public void periodic() {

    }
}

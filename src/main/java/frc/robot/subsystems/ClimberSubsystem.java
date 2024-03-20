package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utilities.CANSparkMaxUtil;
import frc.robot.utilities.CANSparkMaxUtil.Usage;
import frc.robot.utilities.constants.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax rightClimbMotor;
    private CANSparkMax leftClimbMotor;

    private RelativeEncoder rightClimbEncoder;
    private RelativeEncoder leftClimbEncoder;

    public ClimberSubsystem() {
        rightClimbMotor = new CANSparkMax(ClimberConstants.RightClimbMotorID, MotorType.kBrushless);
        leftClimbMotor = new CANSparkMax(ClimberConstants.LeftClimbMotorID, MotorType.kBrushless);

        rightClimbEncoder = rightClimbMotor.getEncoder();
        leftClimbEncoder = leftClimbMotor.getEncoder();

        configureRightClimbMotor();
        configureLeftClimbMotor();
    }

    private void configureRightClimbMotor() {
        rightClimbMotor.restoreFactoryDefaults();
        rightClimbMotor.setSmartCurrentLimit(ClimberConstants.rightClimbContinuousCurrentLimit);
        rightClimbMotor.setInverted(ClimberConstants.rightClimbInvert);
        rightClimbMotor.setIdleMode(ClimberConstants.rightClimbNeutralMode);
        rightClimbMotor.enableVoltageCompensation(ClimberConstants.voltageCompensation);
        rightClimbMotor.burnFlash();
        rightClimbEncoder.setPosition(0.0);
    }

    private void configureLeftClimbMotor() {
        leftClimbMotor.restoreFactoryDefaults();
        leftClimbMotor.setSmartCurrentLimit(ClimberConstants.leftClimbContinuousCurrentLimit);
        leftClimbMotor.setInverted(ClimberConstants.leftClimbInvert);
        leftClimbMotor.setIdleMode(ClimberConstants.leftClimbNeutralMode);
        leftClimbMotor.enableVoltageCompensation(ClimberConstants.voltageCompensation);
        leftClimbMotor.burnFlash();
        leftClimbEncoder.setPosition(0.0);
    }

    public void climbUp() {
        rightClimbMotor.set(ClimberConstants.climbSpeed);
        leftClimbMotor.set(ClimberConstants.climbSpeed);
    }

    public void climbDown() {
        rightClimbMotor.set(-ClimberConstants.climbSpeed);
        leftClimbMotor.set(-ClimberConstants.climbSpeed);
    }

    public void climbHold() {
        rightClimbMotor.set(-0.06);
        leftClimbMotor.set(-0.06);
    }

    public void reset() {
        rightClimbMotor.set(0.0);
        leftClimbMotor.set(0.0);
    }

    @Override
    public void periodic() {

    }
}
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

    public SparkPIDController rightPID;
    public SparkPIDController leftPID;
    

    public ClimberSubsystem() {
        rightClimbMotor = new CANSparkMax(ClimberConstants.RightClimbMotorID, MotorType.kBrushless);
        leftClimbMotor = new CANSparkMax(ClimberConstants.LeftClimbMotorID, MotorType.kBrushless);

        rightClimbEncoder = rightClimbMotor.getEncoder();
        leftClimbEncoder = leftClimbMotor.getEncoder();

        rightPID = rightClimbMotor.getPIDController();
        leftPID = leftClimbMotor.getPIDController();
        configureRightClimbMotor();
        configureLeftClimbMotor();
    }

    private void configureRightClimbMotor() {
        rightClimbMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(rightClimbMotor, Usage.kPositionOnly);
        rightClimbMotor.setSmartCurrentLimit(ClimberConstants.rightClimbContinuousCurrentLimit);
        rightClimbMotor.setInverted(ClimberConstants.rightClimbInvert);
        rightClimbMotor.setIdleMode(ClimberConstants.rightClimbNeutralMode);
        //rightClimbEncoder.setVelocityConversionFactor(ClimberConstants.rightClimbConversionVelocityFactor);
        //rightClimbEncoder.setPositionConversionFactor(ClimberConstants.rightClimbConversionPositionFactor);
        rightPID.setFeedbackDevice(rightClimbEncoder); // Setting the encoder to be the feedback device (EXPERIMENTAL)
        rightPID.setP(ClimberConstants.rClimbKP);
        rightPID.setI(ClimberConstants.rClimbKI);
        rightPID.setD(ClimberConstants.rClimbKD);
        rightPID.setFF(ClimberConstants.rClimbKFF);
        rightClimbMotor.enableVoltageCompensation(ClimberConstants.voltageCompensation);
        rightClimbMotor.burnFlash();
        rightClimbEncoder.setPosition(0.0);
    }

    private void configureLeftClimbMotor() {
        leftClimbMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(leftClimbMotor, Usage.kAll);
        leftClimbMotor.setSmartCurrentLimit(ClimberConstants.leftClimbContinuousCurrentLimit);
        leftClimbMotor.setInverted(ClimberConstants.leftClimbInvert);
        leftClimbMotor.setIdleMode(ClimberConstants.leftClimbNeutralMode);
        //leftClimbEncoder.setVelocityConversionFactor(ClimberConstants.leftClimbConversionVelocityFactor);
        //leftClimbEncoder.setPositionConversionFactor(ClimberConstants.leftClimbConversionPositionFactor);
        leftPID.setFeedbackDevice(leftClimbEncoder); // Setting the encoder to be the feedback device (EXPERIMENTAL)
        leftPID.setP(ClimberConstants.lClimbKP);
        leftPID.setI(ClimberConstants.lClimbKI);
        leftPID.setD(ClimberConstants.lClimbKD);
        leftPID.setFF(ClimberConstants.lClimbKFF);
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

    public void reset() {
        rightClimbMotor.set(0.0);
        leftClimbMotor.set(0.0);
    }

    @Override
    public void periodic() {

    }
}
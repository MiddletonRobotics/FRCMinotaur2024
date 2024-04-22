package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utilities.CANSparkMaxUtil;
import frc.robot.utilities.CANSparkMaxUtil.Usage;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax rightClimbMotor;
    private CANSparkMax leftClimbMotor;

    private RelativeEncoder rightClimbEncoder;
    private RelativeEncoder leftClimbEncoder;
    private SparkPIDController rightClimbPIDController;
    private SparkPIDController leftClimbPIDController;

    public ClimberSubsystem() {
        rightClimbMotor = new CANSparkMax(ClimberConstants.RightClimbMotorID, MotorType.kBrushless);
        rightClimbEncoder = rightClimbMotor.getEncoder();
        rightClimbPIDController = rightClimbMotor.getPIDController();

        leftClimbMotor = new CANSparkMax(ClimberConstants.LeftClimbMotorID, MotorType.kBrushless);
        leftClimbEncoder = leftClimbMotor.getEncoder();
        leftClimbPIDController = leftClimbMotor.getPIDController();

        configureRightClimbMotor();
        configureLeftClimbMotor();
    }

    private void configureRightClimbMotor() {
        rightClimbMotor.restoreFactoryDefaults();
        rightClimbMotor.setSmartCurrentLimit(ClimberConstants.rightClimbContinuousCurrentLimit);
        rightClimbMotor.setInverted(ClimberConstants.rightClimbInvert);
        rightClimbMotor.setIdleMode(ClimberConstants.rightClimbNeutralMode);
        rightClimbMotor.enableVoltageCompensation(ClimberConstants.voltageCompensation);
        rightClimbPIDController.setFeedbackDevice(rightClimbEncoder);
        rightClimbPIDController.setP(ClimberConstants.rclimbKI);
        rightClimbPIDController.setI(ClimberConstants.rclimbKI);
        rightClimbPIDController.setD(ClimberConstants.rclimbKD);
        rightClimbPIDController.setFF(ClimberConstants.rclimbKFF);
        rightClimbMotor.burnFlash();
        rightClimbEncoder.setPosition(0.0);
    }

    private void configureLeftClimbMotor() {
        leftClimbMotor.restoreFactoryDefaults();
        leftClimbMotor.setSmartCurrentLimit(ClimberConstants.leftClimbContinuousCurrentLimit);
        leftClimbMotor.setInverted(ClimberConstants.leftClimbInvert);
        leftClimbMotor.setIdleMode(ClimberConstants.leftClimbNeutralMode);
        leftClimbMotor.enableVoltageCompensation(ClimberConstants.voltageCompensation);
        leftClimbPIDController.setFeedbackDevice(leftClimbEncoder);
        leftClimbPIDController.setP(ClimberConstants.lclimbKP);
        leftClimbPIDController.setI(ClimberConstants.lclimbKI);
        leftClimbPIDController.setD(ClimberConstants.lclimbKD);
        leftClimbPIDController.setFF(ClimberConstants.lclimbKFF);
        leftClimbMotor.burnFlash();
        leftClimbEncoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Right Climber Enocder", rightClimbEncoder.getPosition());
        SmartDashboard.putNumber("Left Climber Enocder", leftClimbEncoder.getPosition());
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.utilities.constants.Constants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankSubsystem extends SubsystemBase {
    private final TalonFX rightMaster;
    private final TalonFX rightSlave;
    private final TalonFX leftMaster;
    private final TalonFX leftSlave;

    private MotorOutputConfigs rightMotorConfiguration;
    private MotorOutputConfigs leftMotorConfiguration;

    private TalonFXConfigurator rightMasterConfigurator;
    private TalonFXConfigurator rightSlaveConfigurator;
    private TalonFXConfigurator leftMasterConfigurator;
    private TalonFXConfigurator leftSlaveConfigurator;

    private FeedbackConfigs feedbackConfiguration;

    private DifferentialDrive TankDrive;

    public TankSubsystem() {
        rightMaster = new TalonFX(Constants.TankConstants.RightMasterID);
        rightSlave = new TalonFX(Constants.TankConstants.RightSlaveID);
        leftMaster = new TalonFX(Constants.TankConstants.LeftMasterID);
        leftSlave = new TalonFX(Constants.TankConstants.LeftSlaveID);

        rightMasterConfigurator = rightMaster.getConfigurator();
        rightSlaveConfigurator = rightSlave.getConfigurator();
        leftMasterConfigurator = leftMaster.getConfigurator();
        leftSlaveConfigurator = leftSlave.getConfigurator();

        rightMotorConfiguration = new MotorOutputConfigs();
        leftMotorConfiguration = new MotorOutputConfigs();

        rightMotorConfiguration.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfiguration.Inverted = InvertedValue.CounterClockwise_Positive;

        rightMotorConfiguration.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfiguration.NeutralMode = NeutralModeValue.Brake;

        feedbackConfiguration = new FeedbackConfigs();
        feedbackConfiguration.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        rightMasterConfigurator.apply(new TalonFXConfiguration().withMotorOutput(rightMotorConfiguration).withFeedback(feedbackConfiguration));
        rightSlaveConfigurator.apply(new TalonFXConfiguration().withMotorOutput(rightMotorConfiguration).withFeedback(feedbackConfiguration));
        leftMasterConfigurator.apply(new TalonFXConfiguration().withMotorOutput(leftMotorConfiguration).withFeedback(feedbackConfiguration));
        leftSlaveConfigurator.apply(new TalonFXConfiguration().withMotorOutput(leftMotorConfiguration).withFeedback(feedbackConfiguration));

        leftSlave.setControl(new Follower(leftMaster.getDeviceID(), false));
        rightSlave.setControl(new Follower(rightMaster.getDeviceID(), false));

        TankDrive = new DifferentialDrive(rightMaster, leftMaster);
        TankDrive.setSafetyEnabled(false);
    }

    public void setRightInverted() {
        boolean invertedState = rightMaster.getInverted();

        if(invertedState) {
            rightMasterConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
            rightSlaveConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        } else {
            rightMasterConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
            rightSlaveConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        }
    }

    public void setLeftInverted() {
        boolean invertedState = leftMaster.getInverted();

        if(invertedState) {
            leftMasterConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
            leftSlaveConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        } else {
            leftMasterConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
            leftSlaveConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        }
    }

    public void setIdleMode(String mode) {
        if(mode == "Coast") {
            rightMasterConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
            rightSlaveConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
            leftMasterConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
            leftSlaveConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        } else if(mode == "Brake") {
            rightMasterConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
            rightSlaveConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
            leftMasterConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
            leftSlaveConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        }
    }

    public void ArcadeDrive(double y, double x) {
        TankDrive.feed();

        double rotationSpeed = Math.abs(x) > Constants.DriverConstants.kDeadband ? x : 0.0;
        double forwardSpeed = Math.abs(y) > Constants.DriverConstants.kDeadband ? y : 0.0;

        TankDrive.arcadeDrive(forwardSpeed, rotationSpeed);
    }
    
    public void stopTankDrive() {
        TankDrive.stopMotor();
    }
}

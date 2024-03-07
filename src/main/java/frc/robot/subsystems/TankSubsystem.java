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
    private final TalonFX frontRight;
    private final TalonFX backRight;
    private final TalonFX frontLeft;
    private final TalonFX backLeft;

    private MotorOutputConfigs rightMotorConfiguration;
    private MotorOutputConfigs leftMotorConfiguration;

    private TalonFXConfigurator frontRightConfigurator;
    private TalonFXConfigurator backRightConfigurator;
    private TalonFXConfigurator frontLeftConfigurator;
    private TalonFXConfigurator backLeftConfigurator;

    private FeedbackConfigs feedbackConfiguration;

    private DifferentialDrive TankDrive;

    public TankSubsystem() {
        frontRight = new TalonFX(Constants.TankConstants.FrontRightID);
        backRight = new TalonFX(Constants.TankConstants.BackRightID);
        frontLeft = new TalonFX(Constants.TankConstants.FrontLeftID);
        backLeft = new TalonFX(Constants.TankConstants.BackLeftID);

        frontRightConfigurator = frontRight.getConfigurator();
        backRightConfigurator = backRight.getConfigurator();
        frontLeftConfigurator = frontLeft.getConfigurator();
        backLeftConfigurator = backLeft.getConfigurator();

        rightMotorConfiguration = new MotorOutputConfigs();
        leftMotorConfiguration = new MotorOutputConfigs();

        rightMotorConfiguration.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfiguration.Inverted = InvertedValue.CounterClockwise_Positive;

        rightMotorConfiguration.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfiguration.NeutralMode = NeutralModeValue.Brake;

        feedbackConfiguration = new FeedbackConfigs();
        feedbackConfiguration.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        frontRightConfigurator.apply(new TalonFXConfiguration().withMotorOutput(rightMotorConfiguration).withFeedback(feedbackConfiguration));
        backRightConfigurator.apply(new TalonFXConfiguration().withMotorOutput(rightMotorConfiguration).withFeedback(feedbackConfiguration));
        frontLeftConfigurator.apply(new TalonFXConfiguration().withMotorOutput(leftMotorConfiguration).withFeedback(feedbackConfiguration));
        backLeftConfigurator.apply(new TalonFXConfiguration().withMotorOutput(leftMotorConfiguration).withFeedback(feedbackConfiguration));

        backLeft.setControl(new Follower(frontLeft.getDeviceID(), false));
        backRight.setControl(new Follower(frontRight.getDeviceID(), false));

        TankDrive = new DifferentialDrive(frontRight, frontLeft);
        TankDrive.setSafetyEnabled(false);
    }

    public void setRightInverted() {
        boolean invertedState = frontRight.getInverted();

        if(invertedState) {
            frontRightConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
            backRightConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        } else {
            frontRightConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
            backRightConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        }
    }

    public void setLeftInverted() {
        boolean invertedState = frontLeft.getInverted();

        if(invertedState) {
            frontLeftConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
            backLeftConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        } else {
            frontLeftConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
            backLeftConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        }
    }

    public void setIdleMode(String mode) {
        if(mode == "Coast") {
            frontRightConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
            backRightConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
            frontLeftConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
            backLeftConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        } else if(mode == "Brake") {
            frontRightConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
            backRightConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
            frontLeftConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
            backLeftConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
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

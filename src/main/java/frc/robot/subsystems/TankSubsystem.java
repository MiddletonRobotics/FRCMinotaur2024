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
import frc.robot.utilities.XboxController;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class TankSubsystem extends SubsystemBase {
    private final TalonFX rightMaster;
    private final TalonFX rightSlave;
    private final TalonFX leftMaster;
    private final TalonFX leftSlave;

    private MotorOutputConfigs leftMotorConfiguration;
    private MotorOutputConfigs rightMotorConfiguration;

    private TalonFXConfigurator rightMasterConfigurator;
    private TalonFXConfigurator rightSlaveConfigurator;
    private TalonFXConfigurator leftMasterConfigurator;
    private TalonFXConfigurator leftSlaveConfigurator;

    private FeedbackConfigs feedbackConfiguration;

    private DifferentialDrive TankDrive;

    private CommandXboxController DriverController;
    private CommandXboxController OperatorController;

    public TankSubsystem() {
        rightMaster = new TalonFX(Constants.TankConstants.rightMasterID);
        rightSlave = new TalonFX(Constants.TankConstants.rightSlaveID);
        leftMaster = new TalonFX(Constants.TankConstants.leftMasterID);
        leftSlave = new TalonFX(Constants.TankConstants.leftSlaveID);

        rightMasterConfigurator = rightMaster.getConfigurator();
        rightSlaveConfigurator = rightSlave.getConfigurator();
        leftMasterConfigurator = leftMaster.getConfigurator();
        leftSlaveConfigurator = leftSlave.getConfigurator();

        leftMotorConfiguration = new MotorOutputConfigs();
        rightMotorConfiguration = new MotorOutputConfigs();

        leftMotorConfiguration.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfiguration.Inverted = InvertedValue.CounterClockwise_Positive;

        leftMotorConfiguration.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfiguration.NeutralMode = NeutralModeValue.Brake;

        feedbackConfiguration = new FeedbackConfigs();
        feedbackConfiguration.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        rightMasterConfigurator.apply(new TalonFXConfiguration().withMotorOutput(rightMotorConfiguration).withFeedback(feedbackConfiguration));
        rightSlaveConfigurator.apply(new TalonFXConfiguration().withMotorOutput(rightMotorConfiguration).withFeedback(feedbackConfiguration));
        leftMasterConfigurator.apply(new TalonFXConfiguration().withMotorOutput(leftMotorConfiguration).withFeedback(feedbackConfiguration));
        leftSlaveConfigurator.apply(new TalonFXConfiguration().withMotorOutput(leftMotorConfiguration).withFeedback(feedbackConfiguration));

        leftSlave.setControl(new Follower(leftMaster.getDeviceID(), false));
        rightSlave.setControl(new Follower(rightMaster.getDeviceID(), false));

        leftMaster.setSafetyEnabled(true);
        leftSlave.setSafetyEnabled(true);

        TankDrive = new DifferentialDrive(leftMaster, rightMaster);
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

    public Command driveCommand(DoubleSupplier x, DoubleSupplier y) {
        DriverController = XboxController.getDriverController();

        // x = Math.abs(x.getAsDouble()) > Constants.TankConstants.kDeadband ? x : 0.0;
        // y = Math.abs(y.getAsDouble()) > Constants.TankConstants.kDeadband ? y : 0.0;

        return run(() -> TankDrive.arcadeDrive(x.getAsDouble(), y.getAsDouble())).withName("drive");
    }
    
}

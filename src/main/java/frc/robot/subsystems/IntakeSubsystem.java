package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import frc.robot.utilities.CANSparkMaxUtil;
import frc.robot.utilities.CANSparkMaxUtil.Usage;
import frc.robot.utilities.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax rollerMotor;
    private CANSparkMax pivotMotor;

    private RelativeEncoder pivotEncoder;
    private CANcoder intakeEncoder;
    private CANcoderConfigurator pivotEncoderConfigurator;

    private SparkPIDController pivotPIDController;

    private Rotation2d angleOffset;
    public boolean deployPosition;

    public IntakeSubsystem() {
        angleOffset = Constants.IntakeConstants.angleOffset;
        deployPosition = false;

        rollerMotor = new CANSparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless);
        rollerMotor.setInverted(Constants.IntakeConstants.rollerMotorInvert);
        configureRollerMotor();

        pivotMotor = new CANSparkMax(Constants.IntakeConstants.pivotMotorID, MotorType.kBrushless);
        pivotMotor.setInverted(Constants.IntakeConstants.pivotMotorInvert);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = pivotMotor.getPIDController();
        configurePivotMotor();

        intakeEncoder = new CANcoder(Constants.IntakeConstants.pivotEncoderID);
        configurePivotEncoder();
    }

    private void configureRollerMotor() {
        rollerMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(rollerMotor, Usage.kAll);
        rollerMotor.setIdleMode(Constants.IntakeConstants.rollerMotorNeutralMode);
        rollerMotor.enableVoltageCompensation(Constants.IntakeConstants.voltageCompensation);
        rollerMotor.burnFlash();
    }

    private void configurePivotMotor() {
        pivotMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor, Usage.kAll);
        pivotMotor.setIdleMode(Constants.IntakeConstants.pivotMotorNeutralMode);
        pivotEncoder.setPositionConversionFactor(Constants.IntakeConstants.AngleConversionFactor);
        pivotPIDController.setFeedbackDevice(pivotEncoder);
        pivotPIDController.setP(Constants.IntakeConstants.pivotKP);
        pivotPIDController.setI(Constants.IntakeConstants.pivotKI);
        pivotPIDController.setD(Constants.IntakeConstants.pivotKD);
        pivotPIDController.setFF(Constants.IntakeConstants.pivotKFF);
        pivotMotor.enableVoltageCompensation(Constants.IntakeConstants.voltageCompensation);
        pivotMotor.burnFlash();
    }

    private void configurePivotEncoder() {
        pivotEncoderConfigurator = intakeEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        pivotEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    public double getIntakeEncoder() {
        return intakeEncoder.getAbsolutePosition().getValueAsDouble(); // Getting
    }

    public void storePosition() {
        if(getIntakeEncoder() > Constants.IntakeConstants.deployPosition) {
            pivotMotor.set(-0.5);
        } else if(getIntakeEncoder() < Constants.IntakeConstants.deployPosition) {
            pivotMotor.set(0.5);
        } else {
            pivotMotor.set(0);
            deployPosition = false;
        }
    }

    public void deployPosition() {
        if(getIntakeEncoder() < Constants.IntakeConstants.deployPosition) {
            pivotMotor.set(0.5);
        } else if(getIntakeEncoder() > Constants.IntakeConstants.deployPosition) {
            pivotMotor.set(-0.5);
        } else {
            pivotMotor.set(0);
            deployPosition = true;
        }
    }

    //we felt a little silly with the names
    public void intakeConsume() {
        rollerMotor.set(-0.5); //whatever makes motor take thingy
    }

    public void intakeRegurgitate() {
        rollerMotor.set(0.5); //whatever makes motor release thingy
    }

    public void reset() {
        rollerMotor.set(0);
        pivotMotor.set(0);
    }

    @Override
    public void periodic() {

    }
}

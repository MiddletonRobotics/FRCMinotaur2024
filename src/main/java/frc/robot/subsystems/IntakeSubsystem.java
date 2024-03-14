package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
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

    public CANSparkMax rollerMotor;
    public CANSparkMax pivotMotor;
    public CANcoder pivotEncoder;
    public CANcoderConfigurator pivotEncoderConfigurator;
    private Rotation2d angleOffset;

    public IntakeSubsystem() {
        rollerMotor = new CANSparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(Constants.IntakeConstants.pivotMotorID, MotorType.kBrushless); //ints are just there as placeholders till we get actual ones, same with ids
        pivotEncoder = new CANcoder(Constants.IntakeConstants.pivotEncoderID);

        angleOffset = Constants.IntakeConstants.angleOffset;

        configureRollerMotor();
        configurePivotMotor();
        configurePivotEncoder();
    }

    public void configureRollerMotor() {
        rollerMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(rollerMotor, Usage.kAll);
        rollerMotor.setInverted(Constants.IntakeConstants.rollerMotorInvert);
        rollerMotor.setIdleMode(Constants.IntakeConstants.rollerMotorNeutralMode);
        rollerMotor.enableVoltageCompensation(Constants.IntakeConstants.voltageCompensation);
        rollerMotor.burnFlash();
    }

    public void configurePivotMotor() {
        pivotMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor, Usage.kAll);
        pivotMotor.setInverted(Constants.IntakeConstants.pivotMotorInvert);
        pivotMotor.setIdleMode(Constants.IntakeConstants.pivotMotorNeutralMode);
        pivotMotor.enableVoltageCompensation(Constants.IntakeConstants.voltageCompensation);
        pivotMotor.burnFlash();
    }

    private void configurePivotEncoder() {
        pivotEncoderConfigurator = pivotEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        pivotEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    public double getPivotEncoder() {
        return pivotEncoder.getAbsolutePosition().getValueAsDouble(); // Getting
    }

    //we felt a little silly with the names
    public void intakeConsume() {
        rollerMotor.set(0.5); //whatever makes motor take thingy
    }

    public void intakeRegurgitate() {
        rollerMotor.set(-0.5); //whatever makes motor release thingy
    }

    public void storePosition() {
        if(getPivotEncoder() > Constants.IntakeConstants.deployPosition) {
            pivotMotor.set(-0.5);
        } else {
            pivotMotor.set(0);
        }
    }

    public void deployPosition() {
        if(getPivotEncoder() < Constants.IntakeConstants.deployPosition) {
            pivotMotor.set(0.5);
        } else {
            pivotMotor.set(0);
        }
    }

    public void reset() {
        rollerMotor.set(0);
        pivotMotor.set(0);
    }

    @Override
    public void periodic() {

    }
}

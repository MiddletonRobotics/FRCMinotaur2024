package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public static enum IntakeDirection {
        FORWARD, REVERSE, STOPPED
    }

    public static enum IntakeStatus {
        DEPLOYED, STORED
    }

    private IntakeStatus status = IntakeStatus.STORED;
    private IntakeDirection direction = IntakeDirection.STOPPED;

    public IntakeSubsystem() {
        setName("Intakaur");

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

        magnetSensorConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        pivotEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    public double getIntakeEncoder() {
        return intakeEncoder.getAbsolutePosition().getValueAsDouble(); // Getting
    }

    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public Command deployIntake() {
        return run(() -> {
            pivotMotor.set(0.25);
        }).withName("Deploy Intake");
    }

    public void intkeOut(){
        pivotMotor.set(0.3);
        System.out.println("out");
    }

    public void intkeIn(){
        pivotMotor.set(-0.3);
    }

    public void stop(){
        pivotMotor.set(0);
    }

    public Command storeIntake() {
        return run(() -> {
            pivotMotor.set(-0.25);
        }).withName("Store Intake");
    }

    public void setIntakeState(IntakeDirection direction) {
        if (direction == IntakeDirection.FORWARD) {
            this.direction = IntakeDirection.FORWARD;
        } else if(direction == IntakeDirection.STOPPED){
            this.direction = IntakeDirection.STOPPED;
        } else if(direction == IntakeDirection.REVERSE){
            this.direction = IntakeDirection.REVERSE;
        }
    }

    //we felt a little silly with the names
    public void intakeConsume() {
        rollerMotor.set(-0.3); //whatever makes motor take thingy
        setIntakeState(IntakeDirection.FORWARD);
    }

    public void intakeRegurgitate() {
        rollerMotor.set(0.3); //whatever makes motor release thingy
        setIntakeState(IntakeDirection.REVERSE);
    }

    public void intakeToShooter() {
        rollerMotor.set(0.1);
    }

    public void stopIntake() {
        rollerMotor.set(0);
        setIntakeState(IntakeDirection.STOPPED);
    }

    public void reset() {
        rollerMotor.set(0);
    }

    @Override
    public void periodic() {
        
    }
}

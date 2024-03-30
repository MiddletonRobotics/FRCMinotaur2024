package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
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

    private double localSetpoint, processVariable;

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
        //resetToAbsolute();
    }

    private void configurePivotEncoder() {
        pivotEncoderConfigurator = intakeEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        pivotEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    /* 

    public Rotation2d getIntakeEncoder() {
        return Rotation2d.fromRotations(intakeEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(pivotEncoder.getPosition());
    }

    public double getMeasurement(){
        return pivotEncoder.getVelocity();
    }

    public double getnewFF(){
        double GRAVITY_FF = 0.01;
        double cosineScalar = Math.cos(Units.degreesToRadians(pivotEncoder.getPosition()));
        double feedForward = GRAVITY_FF * cosineScalar;
        return feedForward;
      }

    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void setSetpoint(double setpoint) {
        localSetpoint = setpoint;
    }

    public void resetToAbsolute() {
        double absolutePosition = getIntakeEncoder().getDegrees();
        pivotEncoder.setPosition(absolutePosition);
    }

    */

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

    /* 

    @Override
    public void periodic() {
        SmartDashboard.getNumber("intake p", Constants.IntakeConstants.pivotKP);
        SmartDashboard.getNumber("intake i", Constants.IntakeConstants.pivotKI);
        SmartDashboard.getNumber("intake d", Constants.IntakeConstants.pivotKD);

        if(localSetpoint == 0) {
            pivotPIDController.setReference(0, ControlType.kDutyCycle);
        } else {
            pivotPIDController.setP(Constants.IntakeConstants.pivotKP);
            pivotPIDController.setI(Constants.IntakeConstants.pivotKI);
            pivotPIDController.setD(Constants.IntakeConstants.pivotKD);
            pivotPIDController.setReference(localSetpoint, CANSparkMax.ControlType.kSmartMotion, 0, getnewFF(), ArbFFUnits.kPercentOut); //feedIN
            processVariable = pivotEncoder.getPosition();
        }
    }

    */
}

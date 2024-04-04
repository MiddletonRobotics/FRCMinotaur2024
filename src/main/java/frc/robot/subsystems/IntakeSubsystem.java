package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private ShuffleboardTab intakeLogger;

    private CANSparkMax rollerMotor;
    private CANSparkMax pivotMotor;

    private RelativeEncoder pivotEncoder;
    private RelativeEncoder rollerEncoder;
    private CANcoder intakeEncoder;
    private CANcoderConfigurator pivotEncoderConfigurator;

    private SparkPIDController pivotPIDController;
    private SparkPIDController rollerPIDController;

    private double localSetpoint;

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
        intakeLogger = Shuffleboard.getTab("Intake Subsystem");

        angleOffset = Constants.IntakeConstants.angleOffset;
        deployPosition = false;

        rollerMotor = new CANSparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless);
        rollerMotor.setInverted(Constants.IntakeConstants.rollerMotorInvert);
        rollerEncoder = rollerMotor.getEncoder();
        rollerPIDController = rollerMotor.getPIDController();
        configureRollerMotor();

        pivotMotor = new CANSparkMax(Constants.IntakeConstants.pivotMotorID, MotorType.kBrushless);
        pivotMotor.setInverted(Constants.IntakeConstants.pivotMotorInvert);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = pivotMotor.getPIDController();
        configurePivotMotor();

        intakeEncoder = new CANcoder(Constants.IntakeConstants.pivotEncoderID);
        configureIntakeEncoder();
    }

    private void configureRollerMotor() {
        rollerMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(rollerMotor, Usage.kAll);
        rollerMotor.setIdleMode(Constants.IntakeConstants.rollerMotorNeutralMode);
        pivotPIDController.setFeedbackDevice(rollerEncoder);
        rollerPIDController.setP(Constants.IntakeConstants.rollerKP);
        rollerPIDController.setI(Constants.IntakeConstants.rollerKI);
        rollerPIDController.setD(Constants.IntakeConstants.rollerKD);
        rollerPIDController.setFF(Constants.IntakeConstants.rollerKFF);
        rollerMotor.enableVoltageCompensation(Constants.IntakeConstants.voltageCompensation);
        rollerMotor.burnFlash();
        rollerEncoder.setPosition(0);
    }

    private void configurePivotMotor() {
        pivotMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor, Usage.kPositionOnly);
        pivotMotor.setIdleMode(Constants.IntakeConstants.pivotMotorNeutralMode);
        pivotEncoder.setPositionConversionFactor(Constants.IntakeConstants.AngleConversionFactor);
        pivotPIDController.setFeedbackDevice(pivotEncoder);
        pivotPIDController.setOutputRange(0, 0.4);
        //pivotPIDController.setP(Constants.IntakeConstants.pivotKP);
        //pivotPIDController.setI(Constants.IntakeConstants.pivotKI);
        //pivotPIDController.setD(Constants.IntakeConstants.pivotKD);
        //pivotPIDController.setFF(Constants.IntakeConstants.pivotKFF);
        pivotMotor.enableVoltageCompensation(Constants.IntakeConstants.voltageCompensation);
        pivotMotor.burnFlash();
        resetToAbsolute();
    }

    private void configureIntakeEncoder() {
        pivotEncoderConfigurator = intakeEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        pivotEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    public Rotation2d getIntakeEncoder() {
        return Rotation2d.fromRotations(intakeEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(pivotEncoder.getPosition());
    }

    public double getnewFF() {
        double GRAVITY_FF = 0.01;
        double cosineScalar = Math.cos(Units.degreesToRadians(pivotEncoder.getPosition()));
        double feedForward = GRAVITY_FF * cosineScalar;
        return feedForward;
    }

    public double getPivotError() {
        return localSetpoint - pivotEncoder.getPosition();
    }

    public boolean getAtGoal() {
        return (Math.abs(localSetpoint - pivotEncoder.getPosition()) < Constants.IntakeConstants.goalSetpointErrorTolerence);
    }

    public double getSetpoint() {
        return localSetpoint;
    }

    public IntakeStatus getIntakeStatus() {
        return status;
    }

    public IntakeDirection getIntakeDirection() {
        return direction;
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void setSetpoint(double setpoint) {
        localSetpoint = setpoint;
    }

    public void setIntakeDirection(IntakeDirection direction) {
        if (direction == IntakeDirection.FORWARD) {
            this.direction = IntakeDirection.FORWARD;
        } else if(direction == IntakeDirection.STOPPED){
            this.direction = IntakeDirection.STOPPED;
        } else if(direction == IntakeDirection.REVERSE){
            this.direction = IntakeDirection.REVERSE;
        }
    }

    public void setIntakeStatus(IntakeStatus status) {
        if (status == IntakeStatus.DEPLOYED) {
            this.status = IntakeStatus.DEPLOYED;
        } else if(status == IntakeStatus.STORED){
            this.status = IntakeStatus.STORED;
        }
    }

    public void outtakeGamePiece() {
        rollerMotor.set(0.3);
        setIntakeDirection(IntakeDirection.REVERSE);
    }

    public void intakeGamePiece() {
        rollerMotor.set(-0.3);
        setIntakeDirection(IntakeDirection.FORWARD);
    }

    public void outtakeToShooter() {
        rollerMotor.set(0.1);
        setIntakeDirection(IntakeDirection.REVERSE);
    }

    public void stopIntaking() {
        rollerMotor.set(0);
        setIntakeDirection(IntakeDirection.STOPPED);
    }

    public void deployIntake() {
        setSetpoint(Constants.IntakeConstants.deployIntakeSetpoint);
    }

    public void storeIntake() {
        setSetpoint(Constants.IntakeConstants.storeIntakeSetpoint);
    }

    public void resetToAbsolute() {
        double absolutePosition = getIntakeEncoder().getDegrees();
        pivotEncoder.setPosition(absolutePosition);
    }

    @Override
    public void periodic() {
        intakeLogger.add("Pivot Proportional", Constants.IntakeConstants.pivotKP);
        intakeLogger.add("Pivot Integral", Constants.IntakeConstants.pivotKI);
        intakeLogger.add("Pivot Derivative ", Constants.IntakeConstants.pivotKD);
        intakeLogger.add("Pivot Error", getPivotError());

        intakeLogger.add("Roller Proportional", Constants.IntakeConstants.rollerKP);
        intakeLogger.add("Roller Integral", Constants.IntakeConstants.rollerKI);
        intakeLogger.add("Roller Derivative ", Constants.IntakeConstants.rollerKD);

        intakeLogger.add("Intake Status ", getIntakeStatus());
        intakeLogger.add("Intake Direction ", getIntakeDirection());

        if(localSetpoint == Constants.IntakeConstants.storeIntakeSetpoint) { 
            pivotPIDController.setReference(0, ControlType.kDutyCycle);
            setIntakeStatus(IntakeStatus.STORED);
        } else {
            pivotPIDController.setReference(localSetpoint, CANSparkMax.ControlType.kPosition, 0, getnewFF(), ArbFFUnits.kPercentOut); //feedIN
            if(getAtGoal()) {
                setIntakeStatus(IntakeStatus.DEPLOYED);
            }
        }
    }
}

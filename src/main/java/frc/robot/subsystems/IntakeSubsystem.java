package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import frc.robot.utilities.CANSparkMaxUtil;
import frc.robot.utilities.CANSparkMaxUtil.Usage;
import frc.robot.utilities.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {

    public static enum IntakeDirection {
        FORWARD, REVERSE, STOPPED
    }

    public static enum IntakeStatus {
        DEPLOYED, STORED
    }

    private IntakeStatus status = IntakeStatus.STORED;
    private IntakeDirection direction = IntakeDirection.STOPPED;

    private CANSparkMax rollerMotor;
    private CANSparkMax pivotMotor;

    private RelativeEncoder pivotEncoder;
    private RelativeEncoder rollerEncoder;
    private CANcoder intakeEncoder;
    private CANcoderConfigurator pivotEncoderConfigurator;

    private DigitalInput intakeLimitSwitch;

    private SparkPIDController pivotPIDController;
    private SparkPIDController rollerPIDController;

    private double localSetpoint;
    private double localVelocity;
    public boolean deployPosition;

    public IntakeSubsystem() {
        localSetpoint = Constants.IntakeConstants.storeIntakeSetpoint;
        intakeLimitSwitch = new DigitalInput(Constants.IntakeConstants.intakeLimitSwitch);
        intakeEncoder = new CANcoder(Constants.IntakeConstants.pivotEncoderID);
        configureIntakeEncoder();

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
    }

    private void configureRollerMotor() {
        rollerMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(rollerMotor, Usage.kAll);
        rollerMotor.setIdleMode(Constants.IntakeConstants.rollerMotorNeutralMode);
        rollerPIDController.setFeedbackDevice(rollerEncoder);
        rollerPIDController.setP(Constants.IntakeConstants.rollerKP, 0);
        rollerPIDController.setI(Constants.IntakeConstants.rollerKI, 0);
        rollerPIDController.setD(Constants.IntakeConstants.rollerKD, 0);
        rollerPIDController.setFF(Constants.IntakeConstants.rollerKFF, 0);
        rollerMotor.enableVoltageCompensation(Constants.IntakeConstants.voltageCompensation);
        rollerMotor.burnFlash();
        rollerEncoder.setPosition(0);
    }

    private void configurePivotMotor() {
        pivotMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor, Usage.kPositionOnly);
        pivotMotor.setIdleMode(Constants.IntakeConstants.pivotMotorNeutralMode);
        pivotEncoder.setPositionConversionFactor(Constants.IntakeConstants.PivotConversionPositionFactor);
        pivotPIDController.setFeedbackDevice(pivotEncoder);
        pivotPIDController.setOutputRange(-0.1, 0.23);
        pivotPIDController.setP(Constants.IntakeConstants.pivotKP, 0);
        pivotPIDController.setI(Constants.IntakeConstants.pivotKI, 0);
        pivotPIDController.setD(Constants.IntakeConstants.pivotKD, 0);
        pivotPIDController.setFF(Constants.IntakeConstants.pivotKFF, 0);
        pivotMotor.enableVoltageCompensation(Constants.IntakeConstants.voltageCompensation);
        pivotMotor.burnFlash();
        resetToAbsolute();
    }

    private void configureIntakeEncoder() {
        pivotEncoderConfigurator = intakeEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    public Rotation2d getIntakeEncoder() {
        return Rotation2d.fromRotations(intakeEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(pivotEncoder.getPosition());
    }

    public double getnewFF() {
        double GRAVITY_FF = 1.68;
        double cosineScalar = Math.cos(Units.degreesToRadians(pivotEncoder.getPosition()));
        double feedForward = GRAVITY_FF * cosineScalar;
        return feedForward;
    }

    public double getPivotError() {
        return localSetpoint - pivotEncoder.getPosition();
    }

    public BooleanSupplier getAtGoal() {
        BooleanSupplier targetGoalStatus = () -> (Math.abs(localSetpoint - pivotEncoder.getPosition()) < Constants.IntakeConstants.goalSetpointErrorTolerence);
        return targetGoalStatus;
    }

    public BooleanSupplier getLimitSwitch() {
        BooleanSupplier limitSwitchStatus = () -> intakeLimitSwitch.get();
        return limitSwitchStatus;
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
        setRollerSpeed(0.3);
        setIntakeDirection(IntakeDirection.REVERSE);
    }

    public void feedOuttake() {
        setRollerSpeed(0.8);
        setIntakeDirection(IntakeDirection.REVERSE);
    }

    public void intakeGamePiece() {
        setRollerSpeed(-0.3);
        setIntakeDirection(IntakeDirection.FORWARD);
    }

    public void outtakeToShooter() {
        setRollerSpeed(0.1);
        setIntakeDirection(IntakeDirection.REVERSE);
    }

    public void stopIntaking() {
        setRollerSpeed(0);
        setIntakeDirection(IntakeDirection.STOPPED);
    }

    public void setDeploySetpoint() {
        setSetpoint(Constants.IntakeConstants.deployIntakeSetpoint);
    }

    public void setStoreSetpoint() {
        setSetpoint(Constants.IntakeConstants.storeIntakeSetpoint);
    }

    public BooleanSupplier isReadyToShoot() {
        boolean requirements;

        if(getIntakeStatus() == IntakeStatus.STORED) {
            if(getLimitSwitch().getAsBoolean()) {
                requirements = true;
            } else {
                requirements = false;
            }
        } else {
            requirements = false;
        }

        BooleanSupplier readyToShoot = () -> requirements;
        return readyToShoot;
    }

    public BooleanSupplier isStored() {
        boolean requirements;

        if(getIntakeStatus() == IntakeStatus.STORED) {
            if(!getLimitSwitch().getAsBoolean()) {
                requirements = true;
            } else {
                requirements = false;
            }
        } else {
            requirements = false;
        }

        BooleanSupplier readyToShoot = () -> requirements;
        return readyToShoot;
    }

    public BooleanSupplier isDeployed() {
        boolean requirements;

        if(getIntakeStatus() == IntakeStatus.DEPLOYED) {
            requirements = true;
        } else {
            requirements = false;
        }

        BooleanSupplier readyToShoot = () -> requirements;
        return readyToShoot;
    }

    public void resetToAbsolute() {
        double absolutePosition = getIntakeEncoder().getDegrees();
        pivotEncoder.setPosition(absolutePosition);
    }

    public Command resetIntake() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setStoreSetpoint()).alongWith(new InstantCommand(() -> stopIntaking()))
        );
    }

    /* 

    public Command storeIntake() {
        return new SequentialCommandGroup(
            new ConditionalCommand(new InstantCommand(() -> setStoreSetpoint()), null, isDeployed())
        );
    }


    public Command deployIntake() {
        return new SequentialCommandGroup(
            new ConditionalCommand(new InstantCommand(() -> setDeploySetpoint()), null, isStored())
        );
    }

    */

    public Command storeIntake() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setStoreSetpoint())
        );
    }


    public Command deployIntake() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setDeploySetpoint())
        );
    }

    public Command dropNoteCommand() {
        return new SequentialCommandGroup(new ParallelCommandGroup(
            new InstantCommand(() -> setDeploySetpoint()),
            new WaitCommand(0.3),
            new RunCommand(() -> feedOuttake())).alongWith(new InstantCommand(() -> setStoreSetpoint()),
            new WaitCommand(0.3),
            new InstantCommand(() -> stopIntaking())
        ));
    }

    public Command automaticIntakingCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setDeploySetpoint()),
            new InstantCommand(() -> intakeGamePiece()).onlyIf(getAtGoal()),
            new InstantCommand(() -> stopIntaking()).alongWith(new InstantCommand(() -> setStoreSetpoint())).onlyIf(getLimitSwitch()) 
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Proportional", Constants.IntakeConstants.pivotKP);
        SmartDashboard.putNumber("Pivot Integral", Constants.IntakeConstants.pivotKI);
        SmartDashboard.putNumber("Pivot Derivative ", Constants.IntakeConstants.pivotKD);
        SmartDashboard.putNumber("Pivot Error", getPivotError());

        SmartDashboard.putNumber("Pivot Target Setpoint", localSetpoint);
        SmartDashboard.putNumber("Pivot Current Setpoint", pivotEncoder.getPosition());

        SmartDashboard.putNumber("Roller Proportional", Constants.IntakeConstants.rollerKP);
        SmartDashboard.putNumber("Roller Integral", Constants.IntakeConstants.rollerKI);
        SmartDashboard.putNumber("Roller Derivative ", Constants.IntakeConstants.rollerKD);

        SmartDashboard.putString("Intake Status ", getIntakeStatus().toString());
        SmartDashboard.putString("Intake Direction ", getIntakeDirection().toString());

        //pivotPIDController.setReference(localSetpoint, ControlType.kPosition, 0, getnewFF());
    }
}

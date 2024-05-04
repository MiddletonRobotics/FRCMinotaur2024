//Import all that jazz you need for swerve drive coding.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Robot;
import frc.robot.utilities.Alert;
import frc.robot.utilities.CANSparkMaxUtil;
import frc.robot.utilities.OnboardModuleState;
import frc.robot.utilities.CANSparkMaxUtil.Usage;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.TunableNumber;
import frc.robot.utilities.Alert.AlertType;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.constants.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SwerveModuleState expectedState = new SwerveModuleState();

    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;
    private CANcoder swerveEncoder;
    private CANcoderConfigurator swerveEncoderConfigurator;

    private final SparkPIDController drivePIDController;
    private final SparkPIDController anglePIDController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ModuleConstants.driveKS, Constants.ModuleConstants.driveKV, Constants.ModuleConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        swerveEncoder = new CANcoder(moduleConstants.swerveEncoderID);
        configureSwerveEncoder();

        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePIDController = angleMotor.getPIDController();
        configureAngleMotor();

        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveMotor.setInverted(Constants.SwerveConstants.driveInvert);
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        configureDriveMotor();

        lastAngle = getSwerveModuleState().angle;
    }

    private void configureSwerveEncoder() {
        swerveEncoderConfigurator = swerveEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        swerveEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    private void configureAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Constants.ModuleConstants.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.SwerveConstants.angleInvert);
        angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
        angleEncoder.setPositionConversionFactor(Constants.SwerveConstants.AngleConversionFactor);
        anglePIDController.setFeedbackDevice(angleEncoder);
        anglePIDController.setP(Constants.ModuleConstants.driveKP);
        anglePIDController.setI(Constants.ModuleConstants.driveKI);
        anglePIDController.setD(Constants.ModuleConstants.driveKD);
        angleMotor.enableVoltageCompensation(Constants.ModuleConstants.voltageCompensation);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configureDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.ModuleConstants.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.SwerveConstants.driveInvert);
        driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.DriveConversionPositionFactor);
        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.DriveConversionVelocityFactor);
        drivePIDController.setFeedbackDevice(driveEncoder);
        drivePIDController.setP(Constants.ModuleConstants.angleKP);
        drivePIDController.setI(Constants.ModuleConstants.angleKI);
        drivePIDController.setD(Constants.ModuleConstants.angleKD);
        driveMotor.enableVoltageCompensation(Constants.ModuleConstants.voltageCompensation);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    public Rotation2d getSwerveEncoder() {
        return Rotation2d.fromRotations(swerveEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    public double getMotorVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getDrivePositionMeters() {
        return driveEncoder.getPosition();
    }

    public double getMotorVoltage() {
        return driveMotor.getBusVoltage();
    }

    public SwerveModuleState getDesiredState() {
        return expectedState;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getAngle());
    }

    public void resetToAbsolute() {
        double absolutePosition = getSwerveEncoder().getDegrees();
        angleEncoder.setPosition(absolutePosition);
    }

    public void moduleOnDisabled() {
        driveMotor.setIdleMode(Constants.SwerveConstants.driveIdleMode);
        angleMotor.setIdleMode(Constants.SwerveConstants.angleIdleMode);
    }

    public void moduleOnEnabled() {
        driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(Math.abs(desiredState.speedMetersPerSecond) < 0.006) {
            driveMotor.set(0);
            angleMotor.set(0);

            if(desiredState.angle == lastAngle) {
                resetToAbsolute();
            }

            return;
        }

        desiredState = OnboardModuleState.optimize(desiredState, getSwerveModuleState().angle);
        this.expectedState = desiredState;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
            driveMotor.set(percentOutput);
        } else {
            drivePIDController.setReference(desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.01)) ? lastAngle : desiredState.angle;
        anglePIDController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);

        if(Robot.isSimulation()) {
            angleEncoder.setPosition(angle.getDegrees());
        }

        lastAngle = angle;
    }

    public void voltageDrive(double Volts) {
        driveMotor.setVoltage(Volts);
    }

    public void stopDriveMotor() {
        driveMotor.set(0);
    }

    public void stopAngleMotor() {
        angleMotor.set(0);
    }

    public void stop() {
        stopDriveMotor();
        stopAngleMotor();
    }
}
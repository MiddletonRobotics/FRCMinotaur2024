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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.utilities.CANSparkMaxUtil;
import frc.robot.utilities.OnboardModuleState;
import frc.robot.utilities.CANSparkMaxUtil.Usage;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.SwerveModuleConstants;

//Sets up swerve drive class with encoders. This section can and should be added to.
public class SwerveModule {
    public int moduleNumber;
    private double chassisAngularOffset = 0;

    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SwerveModuleState expectedState = new SwerveModuleState(0.0, new Rotation2d());

    private CANSparkMax driveMotor;
    private CANSparkMax steeringMotor;

    private RelativeEncoder driveEncocder;
    private RelativeEncoder steeringEncoder;
    private CANcoder swerveEncoder;
    private CANcoderConfigurator swerveEncoderConfigurator;

    private final SparkPIDController drivePIDController;
    private final SparkPIDController steeringPIDController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ModuleConstants.driveKS, Constants.ModuleConstants.driveKV, Constants.ModuleConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        swerveEncoder = new CANcoder(moduleConstants.swerveEncoderID);
        configureSwerveEncoder();

        steeringMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        steeringEncoder = steeringMotor.getEncoder();
        steeringPIDController = steeringMotor.getPIDController();
        configureAngleMotor();

        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveMotor.setInverted(Constants.SwerveConstants.driveInvert);
        driveEncocder = driveMotor.getEncoder();
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
        steeringMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(steeringMotor, Usage.kPositionOnly);
        steeringMotor.setSmartCurrentLimit(Constants.ModuleConstants.angleContinuousCurrentLimit);
        steeringMotor.setInverted(Constants.SwerveConstants.angleInvert);
        steeringMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
        steeringEncoder.setPositionConversionFactor(Constants.SwerveConstants.AngleConversionPositionFactor);
        steeringEncoder.setVelocityConversionFactor(Constants.SwerveConstants.AngleConversionVelocityFactor);
        steeringPIDController.setFeedbackDevice(steeringEncoder);
        steeringPIDController.setPositionPIDWrappingEnabled(true);
        steeringPIDController.setPositionPIDWrappingMinInput(Constants.ModuleConstants.angleEncoderPIDMinInput);
        steeringPIDController.setPositionPIDWrappingMaxInput(Constants.ModuleConstants.angleEncoderPIDMaxInput);
        steeringPIDController.setP(Constants.ModuleConstants.angleKP);
        steeringPIDController.setI(Constants.ModuleConstants.angleKI);
        steeringPIDController.setD(Constants.ModuleConstants.angleKD);
        steeringPIDController.setFF(Constants.ModuleConstants.angleKFF);
        steeringPIDController.setOutputRange(Constants.ModuleConstants.angleMinOutput, Constants.ModuleConstants.angleMaxOutput);
        steeringMotor.enableVoltageCompensation(Constants.ModuleConstants.voltageCompensation);
        steeringMotor.burnFlash();
        resetToAbsolute();
    }

    private void configureDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.ModuleConstants.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.SwerveConstants.driveInvert);
        driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        driveEncocder.setVelocityConversionFactor(Constants.SwerveConstants.DriveConversionPositionFactor);
        driveEncocder.setPositionConversionFactor(Constants.SwerveConstants.DriveConversionVelocityFactor);
        drivePIDController.setFeedbackDevice(driveEncocder);
        drivePIDController.setP(Constants.ModuleConstants.angleKP);
        drivePIDController.setI(Constants.ModuleConstants.angleKI);
        drivePIDController.setD(Constants.ModuleConstants.angleKD);
        drivePIDController.setFF(Constants.ModuleConstants.angleKFF);
        drivePIDController.setOutputRange(Constants.ModuleConstants.driveMinOutput, Constants.ModuleConstants.driveMaxOutput);
        driveMotor.enableVoltageCompensation(Constants.ModuleConstants.voltageCompensation);
        driveMotor.burnFlash();
        driveEncocder.setPosition(0.0);
    }

    public Rotation2d getSwerveEncoder() {
        return Rotation2d.fromRotations(swerveEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(steeringEncoder.getPosition());
    }

    public SwerveModuleState getDesiredState() {
        return expectedState;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(driveEncocder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(driveEncocder.getPosition(), getAngle());
    }

    public void resetToAbsolute() {
        double absolutePosition = getSwerveEncoder().getDegrees();
        steeringEncoder.setPosition(absolutePosition);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.005) { 
            driveMotor.set(0);
            steeringMotor.set(0);

            if (desiredState.angle == lastAngle) {   
                resetToAbsolute();
            }

            return; 
        }

        desiredState = OnboardModuleState.optimize(desiredState, getSwerveModuleState().angle);

        this.expectedState = desiredState;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /* 

    public void setDesiredState(SwerveModuleState desiredState) {        
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
    
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = OnboardModuleState.optimize(correctedDesiredState,
            new Rotation2d(steeringEncoder.getPosition()));
    
        // Command driving and turning SPARKS MAX towards their respective setpoints.
        driveMotor.set(optimizedDesiredState.speedMetersPerSecond / Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);
        if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.006)
          driveMotor.stopMotor();
        steeringPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    
        expectedState = desiredState;
    }

    */

    public void setDesiredStateForXlock(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = OnboardModuleState.optimize(desiredState, getSwerveModuleState().angle);
    
        setAngleForXlock(desiredState);
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
        steeringPIDController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);

        if(Robot.isSimulation()) {
            steeringEncoder.setPosition(angle.getDegrees());
        }

        lastAngle = angle;
    }

    private void setAngleForXlock(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.01)) ? desiredState.angle : desiredState.angle;
        steeringPIDController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        lastAngle = angle;
    }
}
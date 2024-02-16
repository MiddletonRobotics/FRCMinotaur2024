//Import various files (encoders, constants, etc.) for swerve drive programming.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ModuleConstants;

//Swerve module class for all your swerve-y needs.
public class SwerveModule {
    //Set up the encoder and motor variables
    private final CANSparkMax forwardMotor;
    private final CANSparkMax thetaMotor;

    private final RelativeEncoder forwardEncoder;
    private final RelativeEncoder thetaEncoder;

    private final PIDController SwervePIDController;

    private final CANcoder swerveEncoder;

    private final boolean swerveEncoderReversed;
    private final double swerveEncoderOffset;

    /**Swerve module constructor: 
    * Takes values for motor and encoder identification, whether or 
    * not motors and encoders are reversed and takes encoder offset value.
    * Configures encoders and sensors and inverts motors depending on which
    * motor is being adjusted (some motors are positioned funky and need reversed movement).
    **/
    public SwerveModule(int forwardMotorID, int thetaMotorID, boolean forwardMotorReversed, boolean thetaMotorReversed, int absoluteEncoderID, double encoderOffset, boolean encoderReversed) {
        this.swerveEncoderOffset = encoderOffset;
        this.swerveEncoderReversed = encoderReversed;
        swerveEncoder = new CANcoder(absoluteEncoderID);

        CANcoderConfigurator configurator = swerveEncoder.getConfigurator();

        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();
        magnetSensorConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        configurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));

        forwardMotor = new CANSparkMax(forwardMotorID, MotorType.kBrushless);
        thetaMotor = new CANSparkMax(thetaMotorID, MotorType.kBrushless);

        forwardMotor.setInverted(forwardMotorReversed);
        thetaMotor.setInverted(thetaMotorReversed);

        /*

        if(forwardMotorReversed = thetaMotorReversed) {
            throw new Exception("The forward motors and theta motors can't be both reversed, please check Constants.java");
        }

        */

        //Get encoders from motors, base position/movement on rpm
        forwardEncoder = forwardMotor.getEncoder();
        thetaEncoder = thetaMotor.getEncoder();

        forwardEncoder.setPositionConversionFactor(Constants.ModuleConstants.DriveEncoderRot2Meter);
        forwardEncoder.setVelocityConversionFactor(Constants.ModuleConstants.DriveEncoderRPM2MeterPerSec);
        thetaEncoder.setPositionConversionFactor(Constants.ModuleConstants.TurningEncoderRot2Rad);
        thetaEncoder.setVelocityConversionFactor(Constants.ModuleConstants.TurningEncoderRPM2RadPerSec);

        SwervePIDController = new PIDController(ModuleConstants.kP, 0, 0);
        SwervePIDController.enableContinuousInput(-Math.PI, Math.PI); // EXPERIMENTAL

        resetMotorEncoders();
    }

    //The following methods return the velocities and positions of the forward and theta encoders.
    public double getForwardPosition() {
        return forwardEncoder.getPosition();
    }

    public double getThetaPosition() {
        return thetaEncoder.getPosition();
    }

    public double getForwardVelocity() {
        return forwardEncoder.getVelocity();
    }

    public double getThetaVelocity() {
        return thetaEncoder.getVelocity();
    }

    /** The following method calculates the angle of the CANCoder by dividing
    * the encoder's current voltage by the total voltage. Unit circle is your friend.
    * Multiply by 2pi to convert to radians and subtract the offset to get the current position.
    * Multiply by the reversed amount since the encoder is backwards.
    */
    public double getCANCoderPosition() {
        double angle = swerveEncoder.getSupplyVoltage().getValueAsDouble() / RobotController.getVoltage5V();

        angle *= 2 * Math.PI;
        angle -= swerveEncoderOffset;
        return angle * (swerveEncoderReversed ? -1.0 : 1.0);
    }

    //Reset the position of the encoders.
    public void resetMotorEncoders() {
        forwardEncoder.setPosition(0);
        thetaEncoder.setPosition(getCANCoderPosition());
    }

    //Use current velocity and the position of the theta encoder to get results for module.
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getForwardVelocity(), new Rotation2d(getThetaPosition()));
    }

    /* This could therodically work, although the swerve module state will reset the modules to 0 when an input is not given as per here: 

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState  = SwerveModuleState.optimize(desiredState, new Rotation2d(getCANCoderPosition()));
        forwardMotor.set(desiredState.speedMetersPerSecond);
        thetaMotor.set(SwervePIDController.calculate(getThetaPosition(), desiredState.angle.getRadians()));

        SmartDashboard.putString("Swerve [" + swerveEncoder.getChannel() + "]", desiredState.toString());
    }

    */

    /** This method checks if there is substantial change movement in the encoders. 
    * If motor movement is less than 0.001, stop the motors as there is not enough movement
    * to start the motors. If there is a large amount of movement set the motors to the 
    * appropriate power based on the encoder positions and velocity.
    *
    **/
    public void setDesiredState(SwerveModuleState desiredState) {

        if(Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        desiredState  = SwerveModuleState.optimize(desiredState, new Rotation2d(getCANCoderPosition()));
        forwardMotor.set(desiredState.speedMetersPerSecond);
        thetaMotor.set(SwervePIDController.calculate(getThetaPosition(), desiredState.angle.getRadians()));

        SmartDashboard.putString("Swerve [" + swerveEncoder.getDeviceID() + "]", desiredState.toString());

    }

    //This method stops the motors.
    public void stop() {
        forwardMotor.set(0);
        thetaMotor.set(0);
    }
}

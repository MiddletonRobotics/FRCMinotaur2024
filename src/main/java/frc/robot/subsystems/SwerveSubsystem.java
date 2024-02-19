//Import required packages to apply swerve drive to robot.
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utilities.constants.Constants;

class SwerveModule {

    private CANSparkMax driveMotor;
    private CANSparkMax steeringMotor;
    private SwerveModuleState currentState;

    public SwerveModule(int driveMotorPort, int steeringMotorPort) {
        driveMotor = new CANSparkMax(driveMotorPort, CANSparkMax.MotorType.kBrushless);
        steeringMotor = new CANSparkMax(steeringMotorPort, CANSparkMax.MotorType.kBrushless);
        currentState = new SwerveModuleState(1, new Rotation2d(Units.degreesToRadians(30)));
    }

    public SwerveModuleState getSwerveModuleState() {
        return currentState;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        currentState = desiredState;
    }
}

public class SwerveSubsystem extends SubsystemBase {

    // We have four swerve modules on our robot, so create a "list" of them
    SwerveModule frontLeftModule = new SwerveModule(1, 2);
    SwerveModule frontRightModule = new SwerveModule(3, 4);
    SwerveModule backLeftModule = new SwerveModule(5, 6);
    SwerveModule backRightModule = new SwerveModule(7, 8);

    double chassisWidth = Units.inchesToMeters(28);
    double chassisLength = Units.inchesToMeters(28);

    // Defining the location of each module on the robot realtive to the center of the robot
    Translation2d frontLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
    Translation2d frontRightLocation = new Translation2d(chassisLength / 2, -chassisWidth / 2);
    Translation2d backLeftLocation = new Translation2d(-chassisLength / 2, chassisWidth / 2);
    Translation2d backRightLocation = new Translation2d(-chassisLength / 2, -chassisWidth / 2);

    // Define a kinematics object. (Takes ChassisSpeeds and returns SwerveModuleStates)
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation
    );

    CommandXboxController controller;
    public SwerveSubsystem(CommandXboxController io) {
        controller = io;
    }

    public void setChassisSpeed(ChassisSpeeds desired) {

        // Get the desired states for each module. Uses fancy math
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desired);

        // Set the desired states for each module (speed/direction)
        frontLeftModule.setDesiredState(newStates[0]);
        frontRightModule.setDesiredState(newStates[1]);
        backLeftModule.setDesiredState(newStates[2]);
        backRightModule.setDesiredState(newStates[3]);
    }

    @Override
    public void periodic() {

        ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            -controller.getLeftY() * 4,
            -controller.getLeftX() * 4,
            controller.getRightX() * 4
        );

        System.out.println(newDesiredSpeeds);

        setChassisSpeed(newDesiredSpeeds);

        double loggingState[] = {
            frontLeftModule.getSwerveModuleState().angle.getDegrees(),
            frontLeftModule.getSwerveModuleState().speedMetersPerSecond,
            frontRightModule.getSwerveModuleState().angle.getDegrees(),
            frontRightModule.getSwerveModuleState().speedMetersPerSecond,
            backLeftModule.getSwerveModuleState().angle.getDegrees(),
            backLeftModule.getSwerveModuleState().speedMetersPerSecond,
            backRightModule.getSwerveModuleState().angle.getDegrees(),
            backRightModule.getSwerveModuleState().speedMetersPerSecond,
        };

        SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
    }
}

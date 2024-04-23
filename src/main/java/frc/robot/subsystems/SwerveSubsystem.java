//Import required packages to apply swerve drive to robot.
package frc.robot.subsystems;

import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.utilities.constants.Constants;

/* Sets up class that assigns motors to each swerve module and get swerving.
* Methods created to handle different actions taken on the controls.
*/
public class SwerveSubsystem extends SubsystemBase {
    private final AHRS gyro;

    private SwerveDrivePoseEstimator swerveOdometry;
    private SwerveModule[] swerveModules;

    private final PIDController headingController;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
            voltageDrive(volts.in(Volts));
        },
        this::sysidroutine,
        this)
    );

    private Field2d field;

    public SwerveSubsystem() {
        setName("SwerveSubsystem");

        gyro = new AHRS(SPI.Port.kMXP);
        headingController = new PIDController(Constants.SwerveConstants.headingKp, Constants.SwerveConstants.headingKi, Constants.SwerveConstants.headingKd);
        
        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.ModuleConstants.FrontLeftModule.constants),
            new SwerveModule(1,Constants.ModuleConstants.FrontRightModule.constants),
            new SwerveModule(2,Constants.ModuleConstants.BackLeftModule.constants),
            new SwerveModule(3,Constants.ModuleConstants.BackRightModule.constants)
        };

        swerveOdometry = new SwerveDrivePoseEstimator(Constants.SwerveConstants.SwerveKinematics, getYawRotation2D(), getSwerveModulePositions(), new Pose2d());
        field = new Field2d();

        configureGyroscope();
        configureController();
        configurePathPlanner();
        configureField2D();
    }

    public void configureGyroscope() {
        gyro.zeroYaw();
        gyro.setAngleAdjustment(0.0);
    }

    public double getYawDegrees() {
        return gyro.getYaw();
    }

    public Rotation2d getYawRotation2D() {
        return gyro.getRotation2d();
    }

    public void zeroYaw() {
        gyro.zeroYaw();
    }

    private void configureController() {
        headingController.setPID(Constants.SwerveConstants.headingKp, Constants.SwerveConstants.headingKi, Constants.SwerveConstants.headingKd);
        headingController.enableContinuousInput(-180.0, 180.0);
        headingController.setTolerance(1.5);
      
    }

    public void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetSwerveOdometry,
            this::getRobotRelativeSpeeds, 
            this::drive, 
            new HolonomicPathFollowerConfig(
                Constants.AutonomousConstants.TranslationPID, 
                Constants.AutonomousConstants.RotationalPID, 
                Constants.AutonomousConstants.PhysicalMaxSpeedMetersPerSecond, 
                Constants.AutonomousConstants.DriveBaseRadius, 
                new ReplanningConfig()
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    private void configureField2D() {
        SmartDashboard.putData(this.field);
    
        PathPlannerLogging.setLogActivePathCallback(poses -> this.field.getObject("pathplanner path poses").setPoses(poses));
        PathPlannerLogging.setLogTargetPoseCallback(target -> this.field.getObject("pathplanner targ pose").setPose(target));
        PathPlannerLogging.setLogCurrentPoseCallback(current -> this.field.getObject("pathplanner curr pose").setPose(current));
      }

      public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYawRotation2D())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], false);
        }
    }

    /**
    * Command to drive the robot using robot relative speeds
    * @param speeds The ChassisSpeeds that contains the movement as vx, vy, and omega
    */

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states =  Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);
        setModuleStates(states);
    }

    public void voltageDrive(double Voltage) {
        for (SwerveModule module : swerveModules) {
            module.voltageDrive(Voltage);
        }
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module : swerveModules) {
            states[module.moduleNumber] = module.getSwerveModuleState();
        }

        return states;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule module : swerveModules) {
            positions[module.moduleNumber] = module.getSwerveModulePosition();
        }

        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.SwerveConstants.SwerveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }

    public void setStatesForX() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135))
        });
    }

    public Pose2d getPose() {
       return swerveOdometry.getEstimatedPosition();
    }

    public void resetSwerveOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYawRotation2D(), getSwerveModulePositions(), pose);
    }

    public void resetModulesForward() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0))
        });
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule module : swerveModules) {
            module.resetToAbsolute();
        }
    }

    public void stop() {
        for(SwerveModule module : swerveModules) {
            module.stop();
        }
    }

    public void sysidroutine(SysIdRoutineLog log) {
        log.motor("drive-BR")
            .voltage(m_appliedVoltage.mut_replace(swerveModules[3].getMotorVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(swerveModules[3].getSwerveModulePosition().distanceMeters, Meters))
            .linearVelocity(m_velocity.mut_replace(swerveModules[3].getMotorVelocity(), MetersPerSecond));

        log.motor("drive-FL")
            .voltage(m_appliedVoltage.mut_replace(swerveModules[0].getMotorVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(swerveModules[0].getSwerveModulePosition().distanceMeters, Meters))
            .linearVelocity(m_velocity.mut_replace(swerveModules[0].getMotorVelocity(), MetersPerSecond));

        log.motor("drive-FR")
            .voltage(m_appliedVoltage.mut_replace(swerveModules[1].getMotorVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(swerveModules[1].getSwerveModulePosition().distanceMeters, Meters))
            .linearVelocity(m_velocity.mut_replace(swerveModules[1].getMotorVelocity(), MetersPerSecond));

        log.motor("drive-BL")
            .voltage(m_appliedVoltage.mut_replace(swerveModules[2].getMotorVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(swerveModules[2].getSwerveModulePosition().distanceMeters, Meters))
            .linearVelocity(m_velocity.mut_replace(swerveModules[2].getMotorVelocity(), MetersPerSecond));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new SequentialCommandGroup(
            new InstantCommand(this::resetModulesToAbsolute, this),
            new WaitCommand(0.5),
            sysIdRoutine.quasistatic(direction)
        );
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return new SequentialCommandGroup(
            new InstantCommand(this::resetModulesToAbsolute, this),
            new WaitCommand(0.5),
            sysIdRoutine.dynamic(direction)
        );
    }


    @Override
    public void periodic() {

        swerveOdometry.update(getYawRotation2D(), getSwerveModulePositions());
        field.setRobotPose(getPose());

        double measuredStates[] = {
            swerveModules[0].getSwerveModuleState().angle.getDegrees(),
            swerveModules[0].getSwerveModuleState().speedMetersPerSecond,
            swerveModules[1].getSwerveModuleState().angle.getDegrees(),
            swerveModules[1].getSwerveModuleState().speedMetersPerSecond,
            swerveModules[2].getSwerveModuleState().angle.getDegrees(),
            swerveModules[2].getSwerveModuleState().speedMetersPerSecond,
            swerveModules[3].getSwerveModuleState().angle.getDegrees(),
            swerveModules[3].getSwerveModuleState().speedMetersPerSecond,
        };

        double desiredStates[] = {
            swerveModules[0].getDesiredState().angle.getDegrees(),
            swerveModules[0].getDesiredState().speedMetersPerSecond,
            swerveModules[1].getDesiredState().angle.getDegrees(),
            swerveModules[1].getDesiredState().speedMetersPerSecond,
            swerveModules[2].getDesiredState().angle.getDegrees(),
            swerveModules[2].getDesiredState().speedMetersPerSecond,
            swerveModules[3].getDesiredState().angle.getDegrees(),
            swerveModules[3].getDesiredState().speedMetersPerSecond,
        };

        SmartDashboard.putNumberArray("Measured Swerve States", measuredStates);
        SmartDashboard.putNumberArray("Desired Swerve States", desiredStates);
        SmartDashboard.putNumber("NavX Raw Yaw Value", getYawDegrees());

        SmartDashboard.putNumber("FL Absolute Encoder Position", swerveModules[0].getSwerveEncoder().getDegrees());
        SmartDashboard.putNumber("FR Absolute Encoder Position", swerveModules[1].getSwerveEncoder().getDegrees());
        SmartDashboard.putNumber("BL Absolute Encoder Position", swerveModules[2].getSwerveEncoder().getDegrees());
        SmartDashboard.putNumber("BR Absolute Encoder Position", swerveModules[3].getSwerveEncoder().getDegrees());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Imported subsystem/controller classes to control parts of robot.
* Also imported camera, constants file, command files, and information 
* related to controls so robot can do thing when button pressed.
*/
package frc.robot;

import java.sql.Driver;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.TankSubsystem;
// import frc.robot.commands.TankController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import frc.robot.commands.ShooterController;
import frc.robot.commands.SwerveController;
import frc.robot.commands.AmpController;
import frc.robot.commands.CycleShooter;
import frc.robot.commands.IntakePull;
import frc.robot.commands.IntakePush;
import frc.robot.commands.StopIntake;
import frc.robot.commands.ScorePositionQuad;

import frc.robot.utilities.Controller;
import frc.robot.utilities.constants.Constants;


/** Below class holds all info related to controlling robot.

* DriverController is created to drive robot.
* Buttons are assigned so robot can change movement style related to
* heading or score speaker or amp when told to. Swerve drive is 
* connected to left joystick with values to help movement.
* Shooter subsystem/controller is also created.
* 
* Commands are used to tell each system what to do when buttons
* are pressed in ConfigureButtonBindings.
* 
* In constructor, swerve drive is told to take values from
* joystick and robotCentric button to determine movement.
* Camera begins taking in information, and buttons are configured.
* 
* Autonomous command is called (currently to move in straight line).
*/

public class RobotContainer {

  private final CommandXboxController DriverController;
  private final CommandXboxController OperatorController;

  private final SendableChooser<Command> autonomousChooser;
  private final PowerDistribution pdp;

  private final Trigger resetHeading;
  private final Trigger robotCentric;
  private final Trigger speakerScoring;
  private final Trigger ampScoring;
  private final Trigger deployIntake;
  private final Trigger storeIntake;
  private final Trigger intakeGamePiece;
  private final Trigger outtakeGamePiece;

  public final SwerveSubsystem swerveSubsystem;
  private final int translationAxis;
  private final int strafeAxis;
  private final int rotationAxis;

  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterController shooterController;
  private final AmpController ampController;
  private final IntakePull pullNote;
  private final IntakePush pushNote;
  private final StopIntake stopIntake;

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    intakeSubsystem = new IntakeSubsystem();

    pdp = new PowerDistribution(0, ModuleType.kCTRE);

    NamedCommands.registerCommand("Speaker Shooter",  new ShooterController(shooterSubsystem, intakeSubsystem));
    NamedCommands.registerCommand("Amp Shooter",  new AmpController(shooterSubsystem, intakeSubsystem));
    NamedCommands.registerCommand("Spin Intake", new IntakePull(intakeSubsystem));
    NamedCommands.registerCommand("Stop Intake", new StopIntake(intakeSubsystem));

    autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Chooser", autonomousChooser);

    DriverController = Controller.getDriverController();
    OperatorController = Controller.getOperatorController();

    resetHeading = DriverController.y();
    robotCentric = DriverController.x();

    speakerScoring = OperatorController.rightBumper();
    ampScoring = OperatorController.leftBumper();
    deployIntake = OperatorController.a();
    storeIntake = OperatorController.b();
    intakeGamePiece = OperatorController.x();
    outtakeGamePiece = OperatorController.y();

    translationAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftY.value;
    strafeAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftX.value;
    rotationAxis = Constants.ControllerRawButtons.XboxController.Axis.kRightX.value;

    shooterController = new ShooterController(shooterSubsystem, intakeSubsystem);
    ampController = new AmpController(shooterSubsystem, intakeSubsystem);
    pullNote = new IntakePull(intakeSubsystem);
    pushNote = new IntakePush(intakeSubsystem);
    stopIntake = new StopIntake(intakeSubsystem);

    swerveSubsystem.setDefaultCommand(new SwerveController(
      swerveSubsystem, 
      () -> DriverController.getRawAxis(translationAxis),
      () -> DriverController.getRawAxis(strafeAxis), 
      () -> -DriverController.getRawAxis(rotationAxis), 
      () -> robotCentric.getAsBoolean(),
      () -> DriverController.leftBumper().getAsBoolean())
    );
      
    configureButtonBindings();
    RobotController.setBrownoutVoltage(6.75);
    SmartDashboard.putNumber("PDP Current Draw", pdp.getTotalCurrent());
  }

  private void configureButtonBindings() {
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.zeroYaw()));
    ampScoring.whileTrue(ampController);
    speakerScoring.whileTrue(shooterController);
    //deployIntake.whileTrue(intakeSubsystem.deployIntake());
    //storeIntake.whileTrue(intakeSubsystem.storeIntake());
    intakeGamePiece.whileTrue(pushNote);
    outtakeGamePiece.whileTrue(pullNote);
    intakeGamePiece.whileFalse(stopIntake);
    outtakeGamePiece.whileFalse(stopIntake);
  }

  public void onTeleopInit() {
    swerveSubsystem.resetModulesForward(); // TODO: Check if it is needed for this to be here
    //intakeSubsystem.resetIntake();
  }

  public void onDisabled() {
    swerveSubsystem.setStatesForX();
  }
 
  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}

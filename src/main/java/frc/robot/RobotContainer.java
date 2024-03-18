// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Imported subsystem/controller classes to control parts of robot.
* Also imported camera, constants file, command files, and information 
* related to controls so robot can do thing when button pressed.
*/
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// import frc.robot.subsystems.TankSubsystem;
// import frc.robot.commands.TankController;
import frc.robot.commands.SwerveController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterController;
import frc.robot.commands.AmpController;
import frc.robot.commands.ClimberController;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.StoreIntake;
import frc.robot.utilities.Controller;
import frc.robot.utilities.constants.Constants;

/*Below class holds all info related to controlling robot.

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

  private final Joystick DriverController;
  private final Joystick OperatorController;

  private final JoystickButton resetHeading;
  private final JoystickButton robotCentric;
  private final JoystickButton speakerScoring;
  private final JoystickButton ampScoring;
  private final JoystickButton deployIntake;
  private final JoystickButton storeIntake;
  private final JoystickButton intakeGamePiece;
  private final JoystickButton robotClimb;

  private final SwerveSubsystem swerveSubsystem;
  private final int translationAxis;
  private final int strafeAxis;
  private final int rotationAxis;

  private final ShooterSubsystem shooterSubsystem;
  private final ShooterController shooterController;
  private final AmpController ampController;
  private final ClimberSubsystem climberSubsystem;
  private final ClimberController climberController;
  private final IntakeSubsystem intakeSubsystem;
  private final DeployIntake deployIntakeCommand;
  private final StoreIntake storeIntakeCommand;

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    climberSubsystem = new ClimberSubsystem();
    intakeSubsystem = new IntakeSubsystem();

    NamedCommands.registerCommand("Speaker Shooter",  new ShooterController(shooterSubsystem));
    NamedCommands.registerCommand("Amp Shooter",  new AmpController(shooterSubsystem));

    DriverController = Controller.getDriverController();
    OperatorController = Controller.getOperatorController();

    resetHeading = new JoystickButton(DriverController, Constants.ControllerRawButtons.Button.kY.value);
    robotCentric = new JoystickButton(DriverController, Constants.ControllerRawButtons.Button.kLeftBumper.value);
    speakerScoring = new JoystickButton(DriverController, Constants.ControllerRawButtons.Button.kRightBumper.value);
    ampScoring = new JoystickButton(DriverController, Constants.ControllerRawButtons.Button.kLeftBumper.value);
    deployIntake = new JoystickButton(OperatorController, Constants.ControllerRawButtons.Button.kA.value);
    storeIntake = new JoystickButton(OperatorController, Constants.ControllerRawButtons.Button.kB.value);
    intakeGamePiece = new JoystickButton(OperatorController, Constants.ControllerRawButtons.Button.kX.value);
    robotClimb = new JoystickButton(OperatorController, Constants.ControllerRawButtons.Button.kLeftTrigger.value);

    translationAxis = Constants.ControllerRawButtons.Axis.kLeftY.value;
    strafeAxis = Constants.ControllerRawButtons.Axis.kLeftX.value;
    rotationAxis = Constants.ControllerRawButtons.Axis.kRightX.value;

    shooterController = new ShooterController(shooterSubsystem);
    ampController = new AmpController(shooterSubsystem);
    climberController = new ClimberController(climberSubsystem);
    deployIntakeCommand = new DeployIntake(intakeSubsystem);
    storeIntakeCommand = new StoreIntake(intakeSubsystem);

    swerveSubsystem.setDefaultCommand(new SwerveController(
      swerveSubsystem, 
      () -> -DriverController.getRawAxis(translationAxis),
      () -> -DriverController.getRawAxis(strafeAxis), 
      () -> -DriverController.getRawAxis(rotationAxis), 
      () -> robotCentric.getAsBoolean())
    );
      
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));
    ampScoring.whileTrue(new InstantCommand(() -> ampController.execute()));
    speakerScoring.whileTrue(new InstantCommand(() -> shooterController.execute()));
    deployIntake.whileTrue(new InstantCommand(() -> deployIntakeCommand.execute()));
    storeIntake.whileTrue(new InstantCommand(() -> storeIntakeCommand.execute()));
    intakeGamePiece.whileTrue(new InstantCommand(() -> intakeSubsystem.intakeConsume()));
    robotClimb.whileTrue(new InstantCommand(() -> climberController.execute()));
  }

 
  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Line");
    return AutoBuilder.followPath(path);
  }
}

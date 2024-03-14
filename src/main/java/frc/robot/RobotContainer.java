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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterController;
import frc.robot.commands.AmpController;
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

  private final Joystick DriverController = Controller.getDriverController();
  // private final Joystick OperatorController = Controller.getOperatorController();

  private final JoystickButton resetHeading = new JoystickButton(DriverController, Constants.ControllerRawButtons.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(DriverController, Constants.ControllerRawButtons.Button.kLeftBumper.value);
  private final JoystickButton speakerScoring = new JoystickButton(DriverController, Constants.ControllerRawButtons.Button.kRightBumper.value);
  private final JoystickButton ampScoring = new JoystickButton(DriverController, Constants.ControllerRawButtons.Button.kLeftBumper.value);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final int translationAxis = Constants.ControllerRawButtons.Axis.kLeftY.value;
  private final int strafeAxis = Constants.ControllerRawButtons.Axis.kLeftX.value;
  private final int rotationAxis = Constants.ControllerRawButtons.Axis.kRightX.value;

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ShooterController shooterController = new ShooterController(shooterSubsystem);
  private final AmpController ampController = new AmpController(shooterSubsystem);


  private void configureButtonBindings() {
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));
    ampScoring.whileTrue(new InstantCommand(() -> ampController.execute()));
    speakerScoring.whileTrue(new InstantCommand(() -> shooterController.execute()));
  }

  public RobotContainer() {

    NamedCommands.registerCommand("Speaker Shooter", new InstantCommand(() -> shooterController.execute()));
    NamedCommands.registerCommand("Amp Shooter", new InstantCommand(() -> ampController.execute()));

    swerveSubsystem.setDefaultCommand(new SwerveController(
      swerveSubsystem, 
      () -> -DriverController.getRawAxis(translationAxis),
      () -> -DriverController.getRawAxis(strafeAxis), 
      () -> -DriverController.getRawAxis(rotationAxis), 
      () -> robotCentric.getAsBoolean()
    ));

    CameraServer.startAutomaticCapture();
      
    configureButtonBindings();
  };

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Line");
    return AutoBuilder.followPath(path);
  }
}

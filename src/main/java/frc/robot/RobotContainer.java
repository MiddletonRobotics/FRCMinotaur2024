// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Imported swerve, tank drive, and controller packages to control robot's movement and have 
* resources for setup.
*/
package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    swerveSubsystem.setDefaultCommand(new SwerveController(
      swerveSubsystem, 
      () -> -DriverController.getRawAxis(translationAxis),
      () -> -DriverController.getRawAxis(strafeAxis), 
      () -> -DriverController.getRawAxis(rotationAxis), 
      () -> robotCentric.getAsBoolean()));
      
      configureButtonBindings();
  };

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Line");
  }
}

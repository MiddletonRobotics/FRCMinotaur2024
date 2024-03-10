// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Imported swerve, tank drive, and controller packages to control robot's movement and have 
* resources for setup.
*/
package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.SwerveController;
// import frc.robot.commands.TankController;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.TankSubsystem;
import frc.robot.utilities.Controller;

/* Setup class that handles robot movement operations
* and assigns movement to joysticks/buttons. Methods allow driver to
* control robot during teleop.
* Autonomous is yet to be set up.
*/

public class RobotContainer {

  private final Joystick DriverController = Controller.getDriverController();
  // private final Joystick OperatorController = Controller.getOperatorController();

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton resetHeading = new JoystickButton(DriverController, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(DriverController, XboxController.Button.kLeftBumper.value);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //private final TankSubsystem drivetrain = new TankSubsystem();
  //private DoubleSupplier forwardSpeed = () -> DriverController.getRawAxis(Constants.ControllerRawButtons.LEFT_Y_AXIS);
  //private DoubleSupplier rotationSpeed = () -> DriverController.getRawAxis(Constants.ControllerRawButtons.RIGHT_X_AXIS);
  //private TankController ArcadeDrive = new TankController(drivetrain, forwardSpeed, rotationSpeed);

  // If button is pressed, swerve drive is oriented to field or other
  private void configureButtonBindings() {
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));
  }

  //Set up the arcade drive and swerve drive with all the kooky things like getting speed/movement from joystick values and switching heading style from button
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

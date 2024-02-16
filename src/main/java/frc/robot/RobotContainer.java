// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Imported swerve, tank drive, and controller packages to control robot's movement and have 
* resources for setup.
*/
package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TankSubsystem;
import frc.robot.commands.SwerveController;
import frc.robot.commands.TankController;
import frc.robot.utilities.Controller;
import frc.robot.utilities.constants.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/* Setup class that handles robot movement operations
* and assigns movement to joysticks/buttons. Methods allow driver to
* control robot during teleop.
* Autonomous is yet to be set up.
*/

public class RobotContainer {

  private final Joystick DriverController = Controller.getDriverController();
  private final Joystick OperatorController = Controller.getOperatorController();

  // Get speed from values from controls and set up drive systems
  // Left stick moves forward back, right stick turns robot
  private final TankSubsystem drivetrain = new TankSubsystem();
  private DoubleSupplier forwardSpeed = () -> DriverController.getRawAxis(Constants.ControllerRawButtons.LEFT_Y_AXIS);
  private DoubleSupplier rotationSpeed = () -> DriverController.getRawAxis(Constants.ControllerRawButtons.RIGHT_X_AXIS);
  private TankController ArcadeDrive = new TankController(drivetrain, forwardSpeed, rotationSpeed);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // If button is pressed, swerve drive is oriented to field or other
  private void configureButtonBindings() {
    new JoystickButton(DriverController, Controller.b()).toggleOnTrue(swerveSubsystem.resetHeading());
  }

  //Set up the arcade drive and swerve drive with all the kooky things like getting speed/movement from joystick values and switching heading style from button
  public RobotContainer() {
    drivetrain.setDefaultCommand(ArcadeDrive);
    swerveSubsystem.setDefaultCommand(new SwerveController(
      swerveSubsystem,
      () -> -DriverController.getRawAxis(Constants.ControllerRawButtons.LEFT_Y_AXIS),
      () -> DriverController.getRawAxis(Constants.ControllerRawButtons.LEFT_X_AXIS),
      () -> DriverController.getRawAxis(Constants.ControllerRawButtons.RIGHT_X_AXIS),
      () -> !DriverController.getRawButton(Constants.ControllerRawButtons.BTN_A)));

      configureButtonBindings();
  };

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

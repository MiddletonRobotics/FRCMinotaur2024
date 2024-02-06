// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class RobotContainer {

  private final Joystick DriverController = new Joystick(Constants.DriverConstants.driverControllerPort);
  private final Joystick OperatorController = new Joystick(Constants.DriverConstants.operatorControllerPort);

  private final TankSubsystem drivetrain = new TankSubsystem();
  private DoubleSupplier forwardSpeed = () -> DriverController.getRawAxis(Constants.ControllerRawButtons.LEFT_Y_AXIS);
  private DoubleSupplier rotationSpeed = () -> DriverController.getRawAxis(Constants.ControllerRawButtons.RIGHT_X_AXIS);
  private TankController ArcadeDrive = new TankController(drivetrain, forwardSpeed, rotationSpeed);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private void configureButtonBindings() {
    new JoystickButton(DriverController, Constants.ControllerRawButtons.BTN_B).toggleOnTrue(swerveSubsystem.resetHeading());
  }

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

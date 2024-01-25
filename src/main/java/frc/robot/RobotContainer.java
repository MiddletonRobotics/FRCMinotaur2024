// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.TankDrive;
import frc.robot.utilities.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final TankDrive tankDrive = new TankDrive();
  private final CommandXboxController DriverController = XboxController.getDriverController();
  private final CommandXboxController OperatorController = XboxController.getOperatorController();

  private void configureBindings() {
    tankDrive.setDefaultCommand(tankDrive.driveCommand(() -> DriverController.getLeftY(), () -> DriverController.getLeftX()));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
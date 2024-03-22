// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ShooterController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* This section pulls from autonomous code and the
* robotContainer to tell robot to follow basic commands
* including: initialize robotContainer, get information
* periodically, run autonomous command when initialized,
* cancel autonomous command when teleop begins, and cancel
* other commands in test.
*/

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private SwerveSubsystem swerveSubsystem;
  private ShooterController shooterController;

  private double startTimer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    //intakeSubsystem = new IntakeSubsystem();
    //shooterSubsystem = new ShooterSubsystem();
    //swerveSubsystem = new SwerveSubsystem();

    //shooterController = new ShooterController(shooterSubsystem, intakeSubsystem);
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //m_robotContainer.disabled();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
  // m_robotContainer.disabled();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    /* 
    try{}

    startTimer = Timer.getFPGATimestamp();
    shooterController.execute();
  }catch(Exception e){
    e.printStackTrace();
  }
  */
  }

  @Override
  public void autonomousPeriodic() {
    //if(startTimer > 7 && startTimer < 9) {
      //swerveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 2, 0));
  //}
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    disabledPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

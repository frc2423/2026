// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import gg.questnav.questnav.QuestNav;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // @Logged(name = "robotContainer")
  private final RobotContainer m_robotContainer;

  QuestNav questNav = new QuestNav();

  public Robot() {
    m_robotContainer = new RobotContainer();
    DriverStation.silenceJoystickConnectionWarning(true);
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    SignalLogger.stop();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    questNav.commandPeriodic();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}

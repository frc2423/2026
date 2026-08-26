// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.DataLogManager;
import frc.robot.utils.HubTracker;
import frc.robot.utils.HubTracker.Shift;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Logged(name = "robotContainer")
  private final RobotContainer m_robotContainer;

  private String phrases[] = { "duck", "Exterminate", "Destroy", "Eradicate", "Shoot", "fire", "attack", "go",
      "onwards", "fire in the hole", "Hasta la vista", "Duck and cover" };
  int randomNumber;
  boolean areAtSetpoint;
  private String endings[] = { "I'll be Back", "Bye Bye", "See you later, alligator", "gg no re", "good game",
      "The first law of robotics is: A robot may not injure a human being or, through inaction, allow a human being to come to harm.",
      "The second law of robotics is: A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.",
      "The second law of robotics is: A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.",
      "The zero-ith law of robotics is: A robot may not harm humanity, or, by inaction, allow humanity to come to harm." };

  public Robot() {
    DataLogManager.start(); // Optional to mirror the NetworkTables-logged data to a file on disk
    Epilogue.configure(config -> {
      // Change the root data path
      config.root = "/Robot";
    });
    Epilogue.bind(this);
    URCL.start(Map.of(
        21, "arm",
        23, "twindexer",
        24, "intakeMotor1",
        32, "hood",
        34, "feederLeft",
        35, "shootLeft",
        36, "feederRight",
        37, "shooterRight",
        38, "intakeMotor2"));
    m_robotContainer = new RobotContainer();
    DriverStation.silenceJoystickConnectionWarning(true);
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    SignalLogger.stop();
  }

  @Override
  public void robotPeriodic() {
    boolean wereAtSetpoint = areAtSetpoint;
    areAtSetpoint = Math.abs(m_robotContainer.shooterRight.getVelocity() - m_robotContainer.shooterRight.getSetpoint()) <= 300 && Math.abs(m_robotContainer.shooterLeft.getVelocity() - m_robotContainer.shooterLeft.getSetpoint()) <= 300
        && m_robotContainer.shooterRight.getSetpoint() != 0 && m_robotContainer.shooterLeft.getSetpoint() != 0;
      System.out.println(areAtSetpoint+"   "+wereAtSetpoint);    
      if (areAtSetpoint && !wereAtSetpoint) {
      int oldNumber = randomNumber;
      randomNumber = (int) (Math.random() * phrases.length);
      while (oldNumber == randomNumber) {
        randomNumber = (int) (Math.random() * phrases.length);
      }
      Speech.say(phrases[randomNumber]);
    }
    CommandScheduler.getInstance().run();
    m_robotContainer.robotHealth.update();
    if (160 - HubTracker.getMatchTime() < 15) {
      Speech.say((Integer.toString((int) (HubTracker.timeRemainingInCurrentShift().in(Seconds)) - 1)));
    }
    // System.out.println(HubTracker.timeRemainingInCurrentShift().in(Seconds));
    if (HubTracker.timeRemainingInCurrentShift().in(Seconds) < 1) {
      // System.out.println("I'm working!");
      String shift = HubTracker.getNextShift().toString();
      if (shift.contains("_")) {
        int index = shift.indexOf("_");
        shift = shift.substring(0,index) +" "+ shift.substring(index+1);
      }
      Speech.say("Starting " + shift);
    }
    if (160 - HubTracker.getMatchTime() == 1){
      int randomIndex = (int)(Math.random() * endings.length);
      Speech.say(endings[randomIndex]);
    }

  }

  @Override
  public void disabledInit() {
    m_robotContainer.auto.resetPaths();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_robotContainer.auto.timer.reset();
      m_robotContainer.auto.timer.start();
      m_autonomousCommand.schedule();
    }
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

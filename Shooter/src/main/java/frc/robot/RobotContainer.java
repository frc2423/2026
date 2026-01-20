// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.PrintWriter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  XboxController xboxController = new XboxController(0);
  ShooterSubsystem shooter = new ShooterSubsystem();

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("shooterSubsystem", shooter);

  }

  private void configureBindings() {
    new JoystickButton(xboxController, XboxController.Button.kA.value).onTrue(shooter.spin()).onFalse(shooter.stop());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

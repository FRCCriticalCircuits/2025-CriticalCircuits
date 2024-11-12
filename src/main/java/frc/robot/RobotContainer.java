// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.teleopDrive;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  Controller controller = Controller.getInstance();
  CommandXboxController driveController = new CommandXboxController(0); 

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
      new teleopDrive(
        () -> -controller.getDriverLY(),    // Left-Positive
        () -> -controller.getDriverLX(),    // Forward-Positive
        () -> -controller.getDriverRX(),    // CCW Positive
        () -> controller.getDriverLT(),
        () -> controller.getDriverRT()
      )
    );
    configureBindings();
  }

  private void configureBindings() {
    driveController.button(8).debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          swerveSubsystem.resetGyro();
        }
      )
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

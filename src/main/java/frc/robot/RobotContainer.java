// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;

import frc.robot.commands.TeleopSwerve;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick Controller1 = new Joystick(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new TeleopSwerve(
      swerveSubsystem,
      () -> -Controller1.getRawAxis(OperatorConstants.kDriverYAxis),
      () -> Controller1.getRawAxis(OperatorConstants.kDriverXAxis),
      () -> Controller1.getRawAxis(OperatorConstants.kDriverRotAxis),
      () -> !Controller1.getRawButton(OperatorConstants.kDriverFieldOrientedButtonIdx)));

    configureBindings();

  }

  private void configureBindings() {

    new JoystickButton(Controller1, 2).whenPressed(() -> swerveSubsystem.zeroHeading());

  }

  public Command getAutonomousCommand() {

    return null;

  }

}

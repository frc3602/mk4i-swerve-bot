/*
 * Copyright (C) 2023, Team 3602. All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.team3602.robot.subsystems.DrivetrainSubsytem;

public class RobotContainer {
  // Subsystems
  private final DrivetrainSubsytem drivetrainSubsys = Constants.DrivetrainConstants.DRIVETRAIN_SUBSYTEM;

  // Operator interfaces
  private final CommandXboxController xboxController = new CommandXboxController(
      Constants.OperatorInterfaceConstants.XBOX_CONTROLLER_PORT);

  // Autonomous
  private SendableChooser<Command> sendableChooser = new SendableChooser<>();

  public RobotContainer() {
    configDefaultCommands();
    configButtonBindings();
    configAutonomous();

    drivetrainSubsys.registerTelemetry(drivetrainSubsys.telemetryLogger::telemeterize);
  }

  private void configDefaultCommands() {
    drivetrainSubsys.setDefaultCommand(drivetrainSubsys.applyRequest(() -> drivetrainSubsys.fieldCentricDrive
        .withVelocityX(
            MathUtil.applyDeadband(xboxController.getLeftY(), 0.02) * Constants.DrivetrainConstants.MAX_SPEED)
        .withVelocityY(
            MathUtil.applyDeadband(xboxController.getLeftX(), 0.02) * Constants.DrivetrainConstants.MAX_SPEED)
        .withRotationalRate(
            MathUtil.applyDeadband(xboxController.getRightX(), 0.02)
                * Constants.DrivetrainConstants.MAX_ANGULAR_RATE)));
  }

  private void configButtonBindings() {
    xboxController.a().whileTrue(drivetrainSubsys.applyRequest(() -> drivetrainSubsys.swerveDriveBrake));

    xboxController.b()
        .whileTrue(drivetrainSubsys.applyRequest(() -> drivetrainSubsys.pointWheelsAt
            .withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))));

    if (Utils.isSimulation()) {
      drivetrainSubsys.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
  }

  private void configAutonomous() {
    SmartDashboard.putData(sendableChooser);
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}
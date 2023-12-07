/*
 * Copyright (C) 2023, Team 3602. All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.team3602.robot.Constants;
import frc.team3602.robot.Telemetry;

public class DrivetrainSubsytem extends SwerveDrivetrain implements Subsystem {
  public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric().withIsOpenLoop(true);
  public final SwerveRequest.SwerveDriveBrake swerveDriveBrake = new SwerveRequest.SwerveDriveBrake();
  public final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt();

  public final Telemetry telemetryLogger = new Telemetry(Constants.DrivetrainConstants.MAX_SPEED);

  private final ChoreoTrajectory trajectory = Choreo.getTrajectory("Trajectory");

  public DrivetrainSubsytem(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
      SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, odometryUpdateFrequency, moduleConstants);
  }

  public DrivetrainSubsytem(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, moduleConstants);
  }

  @Override
  public void simulationPeriodic() {
    // This is an estimate.
    updateSimState(0.02, 12);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Construct a swerve command for the robot to follow the trajectory.
   */
  public Command swerveControllerCommand() {
    return Choreo.choreoSwerveCommand(
        trajectory,
        () -> telemetryLogger.robotPose,
        new PIDController(Constants.AutonomousConstants.KP_X_CONTROLLER, 0.0, 0.0),
        new PIDController(Constants.AutonomousConstants.KP_Y_CONTROLLER, 0.0, 0.0),
        new PIDController(Constants.AutonomousConstants.KP_THETA_CONTROLLER, 0.0, 0.0),
        (ChassisSpeeds speeds) -> {
          this.applyRequest(() -> fieldCentricDrive
              .withVelocityX(speeds.vxMetersPerSecond)
              .withVelocityY(speeds.vyMetersPerSecond)
              .withRotationalRate(speeds.omegaRadiansPerSecond));
        },
        true,
        this);
  }
}
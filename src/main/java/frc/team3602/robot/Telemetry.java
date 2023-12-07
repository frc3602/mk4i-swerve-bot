/*
 * Copyright (C) 2023, Team 3602. All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
  private final double maxSpeed;

  /**
   * Construct a telemetry object, with the specified max speed of the robot.
   * 
   * @param maxSpeed Maximum speed in meters per second.
   */
  public Telemetry(double maxSpeed) {
    this.maxSpeed = maxSpeed;
  }

  // What to publish over network tables for telemetry.
  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();

  // Robot pose for field positioning.
  public final NetworkTable robotPoseTable = instance.getTable("Pose");
  public DoubleArrayPublisher fieldPublishier = robotPoseTable.getDoubleArrayTopic("robotPose").publish();
  public StringPublisher fieldTypePublisher = robotPoseTable.getStringTopic(".type").publish();

  // Robot speeds for general checking.
  public final NetworkTable robotSpeeds = instance.getTable("Drive");
  public DoublePublisher velocityX = robotSpeeds.getDoubleTopic("Velocity X").publish();
  public DoublePublisher velocityY = robotSpeeds.getDoubleTopic("Velocity Y").publish();
  public DoublePublisher speed = robotSpeeds.getDoubleTopic("Speed").publish();
  public DoublePublisher odometryPeriod = robotSpeeds.getDoubleTopic("Odometry Period").publish();

  // Keep a reference of the last pose to calculate the speeds.
  private Pose2d lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();

  // Mechanism to represent the swerve module states.
  private Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
      new Mechanism2d(1, 1),
      new Mechanism2d(1, 1),
      new Mechanism2d(1, 1),
      new Mechanism2d(1, 1),
  };

  // A direction of length changing ligament for speed representation.
  MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
      moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
  };

  // A direction changing and length constant ligament for module direction.
  MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
      moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
  };

  /**
   * Accept the swerve drive state and telemeterize it to smartdashboard.
   * 
   * @param state State of the swerve drive.
   */
  public void telemeterize(SwerveDriveState state) {
    // Telemeterize the pose.
    Pose2d pose = state.Pose;
    fieldPublishier.set(new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees()
    });
    fieldTypePublisher.set("Field2d");

    // Telemeterize the robot's general speeds.
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;

    Translation2d distanceDiff = pose.minus(lastPose).getTranslation();
    lastPose = pose;

    Translation2d velocities = distanceDiff.div(diffTime);

    speed.set(velocities.getNorm());
    velocityX.set(velocities.getX());
    velocityY.set(velocities.getY());
    odometryPeriod.set(state.OdometryPeriod);

    // Telemeterize the module's states.
    for (int i = 0; i < 4; ++i) {
      moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2.0 * maxSpeed));

      SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
    }
  }
}
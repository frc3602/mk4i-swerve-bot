package frc.team3602.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class VisionSubsystem implements Subsystem {
  // PhotonVision camera
  private final PhotonCamera camera = new PhotonCamera("photonvision");

  public VisionSubsystem() {
    configVisionSubsystem();
  }

  @Override
  public void periodic() {

  }

  private void configVisionSubsystem() {

  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem for handling camera input and vision processing.
 * Currently a baseline implementation that initializes and does nothing.
 */
public class VisionSubsystem extends SubsystemBase {
  // Camera instance - declared as a class member so it persists across methods
  private final PhotonCamera kCameraFrontLeft = new PhotonCamera(VisionConstants.kCameraFrontLeft);

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    // Baseline initialization - nothing yet
  }

  @Override
  public void periodic() {
    // get pipeline result from front left camera
    var result = kCameraFrontLeft.getLatestResult();

    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();

    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> targets = result.getTargets();

    //If resullt.hastargets() is true, we can process the targets as needed.
    if (hasTargets) {
      // Get the current best target.
    PhotonTrackedTarget target = result.getBestTarget();
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    Transform3d transform = target.getBestCameraToTarget();

    int targetID = target.getFiducialId();
    }

    //report the transform of the best target
    SmartDashboard.putNumber("FrontLeft Target Yaw", hasTargets ? result.getBestTarget().getYaw() : 0.0);
    SmartDashboard.putNumber("FrontLeft Target Pitch", hasTargets ? result.getBestTarget().getPitch() : 0.0);
    SmartDashboard.putString("FrontLeft Target Transform", hasTargets ? result.getBestTarget().getBestCameraToTarget().toString() : "No Target");
    //report targets list to the dashboard
    SmartDashboard.putNumber("FrontLeft Target Count", targets.size());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Currently does nothing
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * AprilTag tracking subsystem using PhotonVision.
 * 
 * This subsystem implements frame-centered tracking where we move the turret
 * to keep the detected AprilTag centered in the camera frame using P-based control.
 * 
 * Current implementation: Simple proportional control based on tag position in image.
 * Future: Add coordinate tracking and dead reckoning when tag is lost.
 */
public class AprilTagTracker extends SubsystemBase {
  // NetworkTables entries - cached to reduce memory pressure
  private final NetworkTable m_photonvisionTable;
  private final DoubleEntry m_txEntry; // Horizontal offset in degrees
  private final DoubleEntry m_tyEntry; // Vertical offset in degrees
  private final BooleanEntry m_hasTargetEntry;
  private final DoubleEntry m_targetAreaEntry; // As percentage of image
  private final DoubleEntry m_targetIdEntry;

  // Tracking state
  private boolean m_hasTarget = false;
  private double m_targetX = 0.0; // Pixel X of tag center
  private double m_targetY = 0.0; // Pixel Y of tag center
  private double m_targetArea = 0.0; // Percentage of frame
  private int m_targetId = 14;

  // Throttle NetworkTable reads
  private int m_updateCounter = 0;
  private static final int UPDATE_FREQUENCY = 2; // Read every 40ms at 50Hz

  // Dashboard logging counter - log every 25 cycles (~500ms at 50Hz)
  private int m_dashboardCounter = 0;
  private static final int DASHBOARD_LOG_FREQUENCY = 25;

  /**
   * Creates an AprilTag tracker connected to PhotonVision via NetworkTables.
   */
  public AprilTagTracker() {
    m_photonvisionTable = NetworkTableInstance.getDefault()
        .getTable("photonvision")
        .getSubTable(VisionConstants.kCameraName);

    // Cache all required entries to avoid creating new subscribers every cycle
    m_txEntry = m_photonvisionTable.getDoubleTopic("targetYaw").getEntry(0.0);
    m_tyEntry = m_photonvisionTable.getDoubleTopic("targetPitch").getEntry(0.0);
    m_hasTargetEntry = m_photonvisionTable.getBooleanTopic("hasTarget").getEntry(false);
    m_targetAreaEntry = m_photonvisionTable.getDoubleTopic("targetArea").getEntry(0.0);
    m_targetIdEntry = m_photonvisionTable.getDoubleTopic("targetID").getEntry(-1.0);
  }

  /**
   * Returns whether a target is currently detected.
   */
  public boolean hasTarget() {
    return m_hasTarget;
  }

  /**
   * Gets the current target's ID.
   */
  public int getTargetId() {
    return m_targetId;
  }

  /**
   * Gets the horizontal offset of the tag center from camera center (pixels).
   * Negative = tag is left of center, positive = tag is right of center
   */
  public double getTargetPixelX() {
    return m_targetX;
  }

  /**
   * Gets the vertical offset of the tag center from camera center (pixels).
   * Negative = tag is above center, positive = tag is below center
   */
  public double getTargetPixelY() {
    return m_targetY;
  }

  /**
   * Gets the target area as percentage of frame (0-100).
   */
  public double getTargetArea() {
    return m_targetArea;
  }

  /**
   * Calculates proportional yaw command to center tag in horizontal frame.
   * 
   * @return Desired yaw angular velocity (rad/s), bounded by max speed
   */
  public double calculateYawCommand() {
    if (!m_hasTarget) {
      return 0.0;
    }

    // Horizontal offset from center (pixels)
    double offsetX = m_targetX; // Already centered around 0

    // Apply dead zone
    if (Math.abs(offsetX) < VisionConstants.kTrackingCenterDeadzone) {
      return 0.0;
    }

    // P-based command: offset * gain
    double yawCommand = offsetX * VisionConstants.kYawTrackingP;

    // Clamp to max speed
    return MathUtil.clamp(yawCommand, 
        -VisionConstants.kMaxYawTrackingSpeed, 
        VisionConstants.kMaxYawTrackingSpeed);
  }

  /**
   * Calculates the yaw angle offset needed to center the tag.
   * Based on tag position in frame and camera FOV.
   * 
   * @return Desired yaw angle offset from camera boresight (radians)
   */
  public double getYawAngleOffset() {
    if (!m_hasTarget) {
      return 0.0;
    }

    // Camera FOV in radians
    double fovHorizontalRadians = Math.toRadians(60.48);
    
    // Convert pixel offset to angle offset
    // pixels go from -320 to +320 (for 640 width)
    // angles go from -fov/2 to +fov/2
    double pixelRange = VisionConstants.kCameraResolutionWidth / 2.0;
    double angleRange = fovHorizontalRadians / 2.0;
    
    double yawOffset = (m_targetX / pixelRange) * angleRange;
    
    return MathUtil.clamp(yawOffset, -angleRange, angleRange);
  }

  /**
   * Calculates the pitch angle offset needed to center the tag.
   * Based on tag position in frame and camera FOV.
   * 
   * @return Desired pitch angle offset from camera boresight (radians)
   */
  public double getPitchAngleOffset() {
    if (!m_hasTarget) {
      return 0.0;
    }

    // Estimate vertical FOV (typically 4:3 aspect, so ~45° for 60° horizontal)
    double fovVerticalRadians = Math.toRadians(45.0);
    
    // Convert pixel offset to angle offset
    // pixels go from -240 to +240 (for 480 height)
    // angles go from -fov/2 to +fov/2
    double pixelRange = VisionConstants.kCameraResolutionHeight / 2.0;
    double angleRange = fovVerticalRadians / 2.0;
    
    double pitchOffset = (m_targetY / pixelRange) * angleRange;
    
    return MathUtil.clamp(pitchOffset, -angleRange, angleRange);
  }

  @Override
  public void periodic() {
    // Throttle NetworkTable reads
    m_updateCounter++;
    boolean shouldUpdate = (m_updateCounter >= UPDATE_FREQUENCY);
    if (shouldUpdate) {
      m_updateCounter = 0;

      // Read target detection status
      m_hasTarget = m_hasTargetEntry.get();

      if (m_hasTarget) {
        // Read tag position: PhotonVision returns offsets in degrees from camera center
        double txDegrees = m_txEntry.get(); // Yaw offset
        double tyDegrees = m_tyEntry.get(); // Pitch offset

        // Convert degrees to approximate pixel offset using camera FOV
        // Arducam OV2311 ~60° horizontal FOV
        double fovHorizontalDegrees = 60.48; // degrees
        double fovHorizontalRadians = Math.toRadians(fovHorizontalDegrees);
        double pixelsPerRadian = VisionConstants.kCameraResolutionWidth / fovHorizontalRadians;

        // Convert degree offsets to radians, then to pixels
        m_targetX = Math.toRadians(txDegrees) * pixelsPerRadian; // Horizontal pixel offset
        m_targetY = Math.toRadians(tyDegrees) * pixelsPerRadian; // Vertical pixel offset

        m_targetArea = m_targetAreaEntry.get();
        m_targetId = (int) m_targetIdEntry.get();

        // Intermittent dashboard logging
        m_dashboardCounter++;
        if (m_dashboardCounter >= DASHBOARD_LOG_FREQUENCY) {
          m_dashboardCounter = 0;
          SmartDashboard.putNumber("Vision/Target X (px)", m_targetX);
          SmartDashboard.putNumber("Vision/Target Y (px)", m_targetY);
          SmartDashboard.putNumber("Vision/Target ID", m_targetId);
          SmartDashboard.putNumber("Vision/Target Area (%)", m_targetArea);
        }
      } else {
        m_targetX = 0.0;
        m_targetY = 0.0;
        m_targetArea = 0.0;
        m_targetId = -1;
      }
    }
    // Always publish dashboard indicator every cycle for responsiveness
    SmartDashboard.putBoolean("Vision/Tag14Seen", m_hasTarget && m_targetId == 14);
  }
}

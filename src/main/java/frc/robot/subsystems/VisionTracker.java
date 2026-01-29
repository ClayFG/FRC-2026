// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem for tracking AprilTags using PhotonVision via NetworkTables.
 * Provides yaw and pitch offsets to align turret with detected targets.
 */
public class VisionTracker extends SubsystemBase {
  private final NetworkTable m_cameraTable;
  
  // Cached NetworkTable entries to avoid recreating subscribers every cycle
  private final BooleanEntry m_hasTargetEntry;
  private final DoubleEntry m_yawOffsetEntry;
  private final DoubleEntry m_pitchOffsetEntry;
  private final DoubleEntry m_targetIdEntry;
  
  private double m_latestYawOffset = 0.0;    // Yaw offset in degrees (negative = turn left)
  private double m_latestPitchOffset = 0.0;  // Pitch offset in degrees (negative = pitch down)
  private boolean m_hasTarget = false;
  private int m_targetId = -1;

  // Throttle NetworkTable reads to every 2 cycles (~40ms at 50Hz)
  private int m_updateCounter = 0;
  private static final int UPDATE_FREQUENCY = 2;

  /**
   * Creates a VisionTracker for AprilTag detection via PhotonVision.
   * Communicates with PhotonVision through NetworkTables.
   *
   * @param cameraName The name of the PhotonVision camera (must match NT entry)
   */
  public VisionTracker(String cameraName) {
    // Connect to PhotonVision's NetworkTables entries
    // PhotonVision publishes results at /photonvision/[cameraName]/
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_cameraTable = inst.getTable("photonvision").getSubTable(cameraName);
    
    // Cache entry references to avoid creating new subscribers every periodic() call
    m_hasTargetEntry = m_cameraTable.getBooleanTopic("hasTarget").getEntry(false);
    m_yawOffsetEntry = m_cameraTable.getDoubleTopic("targetYaw").getEntry(0.0);
    m_pitchOffsetEntry = m_cameraTable.getDoubleTopic("targetPitch").getEntry(0.0);
    m_targetIdEntry = m_cameraTable.getDoubleTopic("targetID").getEntry(-1.0);
  }

  /**
   * Gets the latest yaw offset (horizontal) to the target in degrees.
   * Negative values mean turn left, positive mean turn right.
   *
   * @return Yaw offset in degrees
   */
  public double getYawOffset() {
    return m_latestYawOffset;
  }

  /**
   * Gets the latest pitch offset (vertical) to the target in degrees.
   * Negative values mean pitch down, positive mean pitch up.
   *
   * @return Pitch offset in degrees
   */
  public double getPitchOffset() {
    return m_latestPitchOffset;
  }

  /**
   * Returns whether a valid target is currently detected.
   *
   * @return true if target detected, false otherwise
   */
  public boolean hasTarget() {
    return m_hasTarget;
  }

  /**
   * Gets the ID of the currently tracked AprilTag.
   *
   * @return AprilTag ID, or -1 if no target
   */
  public int getTargetId() {
    return m_targetId;
  }

  @Override
  public void periodic() {
    // Throttle NetworkTable reads to reduce network traffic
    m_updateCounter++;
    if (m_updateCounter < UPDATE_FREQUENCY) {
      return;
    }
    m_updateCounter = 0;

    // Read target data from cached NetworkTable entries
    boolean hasTarget = m_hasTargetEntry.get();
    
    if (hasTarget) {
      m_hasTarget = true;
      m_latestYawOffset = m_yawOffsetEntry.get();
      m_latestPitchOffset = m_pitchOffsetEntry.get();
      m_targetId = (int) m_targetIdEntry.get();
      
    } else {
      m_hasTarget = false;
      m_targetId = -1;
    }
  }
}

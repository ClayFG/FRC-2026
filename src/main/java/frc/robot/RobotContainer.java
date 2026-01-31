// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AprilTagTracker;
import frc.robot.subsystems.TwoAxisPositioner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final TwoAxisPositioner m_positioner = new TwoAxisPositioner();
  private final AprilTagTracker m_visionTracker = new AprilTagTracker();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureDefaultCommands();
  }

  /**
   * Configure default commands for subsystems.
   */
  private void configureDefaultCommands() {
    // Positioner control - pitch/yaw with A/X buttons, or AprilTag tracking with B button
    m_positioner.setDefaultCommand(
        new RunCommand(() -> {
            boolean pitchMode = m_driverController.getAButton();
            boolean yawMode = m_driverController.getXButton();
            boolean trackingMode = m_driverController.getBButton();

            double stickX = m_driverController.getLeftX();
            double stickY = -m_driverController.getLeftY();

            stickX = MathUtil.applyDeadband(stickX, OIConstants.kStickDeadband);
            stickY = MathUtil.applyDeadband(stickY, OIConstants.kStickDeadband);

            if (trackingMode && m_visionTracker.hasTarget()) {
                // === APRILTAG TRACKING MODE (B button) ===
                // Calculate angle offsets based on tag position in frame
                double yawAngleOffset = m_visionTracker.getYawAngleOffset();
                double pitchAngleOffset = m_visionTracker.getPitchAngleOffset();

                // Get current positions
                double currentYaw = m_positioner.getYawPosition();
                double currentPitch = m_positioner.getPitchPosition();
                
                // Target = current position + angle offset
                double targetYaw = currentYaw + yawAngleOffset;
                double targetPitch = currentPitch + pitchAngleOffset;

                // Debug output
                SmartDashboard.putNumber("Tracking/Yaw Offset (rad)", yawAngleOffset);
                SmartDashboard.putNumber("Tracking/Pitch Offset (rad)", pitchAngleOffset);
                SmartDashboard.putNumber("Tracking/Target Yaw (rad)", targetYaw);
                SmartDashboard.putNumber("Tracking/Target Pitch (rad)", targetPitch);
                SmartDashboard.putNumber("Tracking/Current Yaw (rad)", currentYaw);
                SmartDashboard.putNumber("Tracking/Current Pitch (rad)", currentPitch);

        // Clamp integrated targets to configured soft limits to avoid
        // requesting positions outside the allowed range.
        targetYaw = MathUtil.clamp(targetYaw,
          Constants.PositionerConstants.kYawReverseSoftLimit,
          Constants.PositionerConstants.kYawForwardSoftLimit);
        targetPitch = MathUtil.clamp(targetPitch,
          Constants.PositionerConstants.kPitchReverseSoftLimit,
          Constants.PositionerConstants.kPitchForwardSoftLimit);

        m_positioner.setYawPosition(targetYaw);
        m_positioner.setPitchPosition(targetPitch);

            } else if (pitchMode) {
                // A button: pitch control
                double targetPitch = Math.PI * (stickY + 1.0) / 4.0;
                m_positioner.setPitchPosition(targetPitch);

      } else if (yawMode) {
        // X button: yaw control
        // Remap stickX (-1..+1) to yaw range 0..90° (0..π/2)
        double targetYaw = Math.PI * (stickX + 1.0) / 4.0; // (stickX+1)*(π/4)
        // Clamp to configured soft limits for safety
        targetYaw = MathUtil.clamp(targetYaw,
          Constants.PositionerConstants.kYawReverseSoftLimit,
          Constants.PositionerConstants.kYawForwardSoftLimit);
        m_positioner.setYawPosition(targetYaw);

            } else {
                // No buttons held - stop motors
                m_positioner.stop();
            }
        }, m_positioner));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}

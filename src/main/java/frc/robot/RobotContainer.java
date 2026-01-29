// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
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
                // Use P-based control to center AprilTag in frame
                double yawCommand = m_visionTracker.calculateYawCommand();
                double pitchCommand = m_visionTracker.calculatePitchCommand();

                // Convert from angular velocity (rad/s) to position setpoint
                // This is a simplified approach - we add velocity command to current position
                double currentYaw = m_positioner.getYawPosition();
                double currentPitch = m_positioner.getPitchPosition();
                
                // Integrate: new position = current + (velocity * dt)
                // At 50Hz, dt = 0.02 seconds
                double dt = 0.02;
                double targetYaw = currentYaw + (yawCommand * dt);
                double targetPitch = currentPitch + (pitchCommand * dt);

                m_positioner.setYawPosition(targetYaw);
                m_positioner.setPitchPosition(targetPitch);

            } else if (pitchMode) {
                // A button: pitch control
                double targetPitch = Math.PI * (stickY + 1.0) / 4.0;
                m_positioner.setPitchPosition(targetPitch);

            } else if (yawMode) {
                // X button: yaw control
                double targetYaw = Math.PI * stickX / 4.0;
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

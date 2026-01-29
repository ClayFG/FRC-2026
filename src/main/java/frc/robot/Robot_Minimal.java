// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Minimal robot template for testing - no subsystems, no SmartDashboard, just basic joystick input.
 * Use this to verify if memory issues are code-related.
 */
public class Robot_Minimal extends TimedRobot {
  private static final int kDriverControllerPort = 0;
  private final XboxController m_driverController = new XboxController(kDriverControllerPort);

  @Override
  public void robotInit() {
    System.out.println("=== MINIMAL ROBOT INIT ===");
    System.out.println("Robot started - no subsystems, minimal code");
  }

  @Override
  public void robotPeriodic() {
    // Just read controller inputs, do nothing with them
    double leftStickX = m_driverController.getLeftX();
    double leftStickY = m_driverController.getLeftY();
    boolean aButton = m_driverController.getAButton();
    
    // Suppress unused variable warnings
    if (leftStickX == 0 && leftStickY == 0 && !aButton) {
      // Do nothing
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // No-op
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}

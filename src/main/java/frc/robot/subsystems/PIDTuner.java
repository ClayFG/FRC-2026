// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Utility class for tuning PID gains on SparkMax motors.
 * Currently disabled to reduce memory pressure.
 * Can be re-enabled with SmartDashboard integration if needed.
 */
public class PIDTuner {
  private final SparkMax m_motor;
  private final SparkMaxConfig m_baseConfig;
  private final String m_name;
  private double m_currentP;
  private double m_currentI;
  private double m_currentD;

  /**
   * Creates a PID tuner for a SparkMax motor.
   *
   * @param motor The SparkMax motor to tune
   * @param baseConfig The base SparkMaxConfig with all other settings
   * @param name Display name for SmartDashboard (e.g., "Pitch", "Yaw")
   * @param initialP Initial P gain
   * @param initialI Initial I gain
   * @param initialD Initial D gain
   */
  public PIDTuner(SparkMax motor, SparkMaxConfig baseConfig, String name, double initialP,
      double initialI, double initialD) {
    m_motor = motor;
    m_baseConfig = baseConfig;
    m_name = name;
    m_currentP = initialP;
    m_currentI = initialI;
    m_currentD = initialD;
  }

  /**
   * Updates PID gains (currently disabled to reduce memory pressure).
   * To re-enable SmartDashboard tuning, restore the SmartDashboard integration.
   */
  public void update() {
    // Tuning disabled to reduce memory pressure and network traffic
  }

  public double getP() { return m_currentP; }
  public double getI() { return m_currentI; }
  public double getD() { return m_currentD; }
}


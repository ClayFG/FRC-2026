// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;
import frc.robot.Constants.PositionerConstants;

public class TwoAxisPositioner extends SubsystemBase {
  private final SparkMax m_pitchSpark;
  private final SparkMax m_yawSpark;

  private final AbsoluteEncoder m_pitchEncoder;
  private final AbsoluteEncoder m_yawEncoder;

  private final SparkClosedLoopController m_pitchClosedLoopController;
  private final SparkClosedLoopController m_yawClosedLoopController;

  // PID tuners for real-time adjustment via SmartDashboard
  private final PIDTuner m_pitchTuner;
  private final PIDTuner m_yawTuner;

  // Dashboard logging counter - log every 25 cycles (~500ms at 50Hz)
  private int m_dashboardCounter = 0;
  private static final int DASHBOARD_LOG_FREQUENCY = 25;

  /**
   * Constructs a TwoAxisPositioner and configures the pitch and yaw motors
   * with absolute encoders.
   */
  public TwoAxisPositioner() {
    m_pitchSpark = new SparkMax(PositionerConstants.kPitchMotorCanId, MotorType.kBrushless);
    m_yawSpark = new SparkMax(PositionerConstants.kYawMotorCanId, MotorType.kBrushless);

    m_pitchEncoder = m_pitchSpark.getAbsoluteEncoder();
    m_yawEncoder = m_yawSpark.getAbsoluteEncoder();

    m_pitchClosedLoopController = m_pitchSpark.getClosedLoopController();
    m_yawClosedLoopController = m_yawSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_pitchSpark.configure(Configs.TwoAxisPositioner.pitchConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_yawSpark.configure(Configs.TwoAxisPositioner.yawConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize PID tuners with current gains for SmartDashboard tuning
    m_pitchTuner = new PIDTuner(m_pitchSpark, Configs.TwoAxisPositioner.pitchConfig, "Pitch", 0.6, 0.0, 0.6);
    m_yawTuner = new PIDTuner(m_yawSpark, Configs.TwoAxisPositioner.yawConfig, "Yaw", 0.2, 0.0, 0.6);
  }

  /**
   * Returns the current pitch position in radians.
   *
   * @return The current pitch position.
   */
  public double getPitchPosition() {
    return m_pitchEncoder.getPosition();
  }

  /**
   * Returns the current yaw position in radians.
   *
   * @return The current yaw position.
   */
  public double getYawPosition() {
    return m_yawEncoder.getPosition();
  }

  /**
   * Sets the desired pitch position using PID control.
   *
   * @param positionRadians Desired pitch position in radians.
   */
  public void setPitchPosition(double positionRadians) {
    m_pitchClosedLoopController.setSetpoint(positionRadians, ControlType.kPosition);
  }

  /**
   * Sets the desired yaw position using PID control.
   *
   * @param positionRadians Desired yaw position in radians.
   */
  public void setYawPosition(double positionRadians) {
    m_yawClosedLoopController.setSetpoint(positionRadians, ControlType.kPosition);
  }

  /**
   * Manually control pitch motor (for testing/tuning).
   *
   * @param voltage Motor voltage in volts.
   */
  public void manualPitchControl(double voltage) {
    m_pitchSpark.setVoltage(voltage);
  }

  /**
   * Manually control yaw motor (for testing/tuning).
   *
   * @param voltage Motor voltage in volts.
   */
  public void manualYawControl(double voltage) {
    m_yawSpark.setVoltage(voltage);
  }

  /**
   * Stops both motors.
   */
  public void stop() {
    m_pitchSpark.set(0);
    m_yawSpark.set(0);
  }

  @Override
  public void periodic() {
    // Update PID gains from SmartDashboard if changed
    m_pitchTuner.update();
    m_yawTuner.update();

    // Intermittent dashboard logging (every 25 cycles ~ 500ms)
    m_dashboardCounter++;
    if (m_dashboardCounter >= DASHBOARD_LOG_FREQUENCY) {
      m_dashboardCounter = 0;
      SmartDashboard.putNumber("Turret/Pitch (deg)", Math.toDegrees(getPitchPosition()));
      SmartDashboard.putNumber("Turret/Yaw (deg)", Math.toDegrees(getYawPosition()));
    }
  }
}
package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PositionerConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class TwoAxisPositioner {
        public static final SparkMaxConfig pitchConfig = new SparkMaxConfig();
        public static final SparkMaxConfig yawConfig = new SparkMaxConfig();

        static {
            // Configure pitch motor (X-axis)
            pitchConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(4);

            pitchConfig.absoluteEncoder
                    .positionConversionFactor(2 * Math.PI) // radians
                    .velocityConversionFactor(2 * Math.PI / 60.0) // radians per second
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            pitchConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(0.6, 0.00, 0.6) // Conservative gains for light system
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, 2 * Math.PI);

            // Add soft limits for pitch (adjust these values based on your mechanical limits)
            pitchConfig.softLimit
                    .forwardSoftLimit(PositionerConstants.kPitchForwardSoftLimit)
                    .reverseSoftLimit(PositionerConstants.kPitchReverseSoftLimit)
                    .forwardSoftLimitEnabled(true)
                    .reverseSoftLimitEnabled(true);

            // Configure yaw motor (Y-axis)
            yawConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .inverted(true);

            yawConfig.absoluteEncoder
                    .positionConversionFactor(2 * Math.PI) // radians
                    .velocityConversionFactor(2 * Math.PI / 60.0) // radians per second
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            yawConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(0.5, 0.0, 0.6) // Much lower P, zero I, higher D for damping
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, 2 * Math.PI);

            // Add soft limits for yaw (adjust these values based on your mechanical limits)
            yawConfig.softLimit
                    .forwardSoftLimit(PositionerConstants.kYawForwardSoftLimit)
                    .reverseSoftLimit(PositionerConstants.kYawReverseSoftLimit)
                    .forwardSoftLimitEnabled(true)
                    .reverseSoftLimitEnabled(true);
        }
    }
}

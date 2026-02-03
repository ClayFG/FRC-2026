// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 10;
    public static final int kRearRightDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 12;
    public static final int kFrontLeftDrivingCanId = 13;


    public static final int kFrontRightTurningCanId = 20;
    public static final int kRearRightTurningCanId = 21;
    public static final int kRearLeftTurningCanId = 22;
    public static final int kFrontLeftTurningCanId = 23;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    // PhotonVision camera name
    public static final String kCameraFrontLeft = "FrontLeft (1)";

    // Camera position relative to robot center (meters and radians)
    // This is the Transform3d from robot center to camera lens
    // X = forward/back (positive forward), Y = left/right (positive right), Z = up/down (positive up)
    // For a front-left camera positioned at front-left corner:
    public static final double kCameraX = DriveConstants.kWheelBase / 2.0;      // Forward, at front of robot
    public static final double kCameraY = DriveConstants.kTrackWidth / 2.0;     // Right side (positive is right)
    public static final double kCameraZ = 0.3;                                   // Height above ground (adjust as needed)
    public static final double kCameraRollDegrees = 0.0;                        // Roll angle
    public static final double kCameraPitchDegrees = 0.0;                       // Pitch angle (adjust if tilted)
    public static final double kCameraYawDegrees = 0.0;                         // Yaw angle (adjust if rotated)

    // Camera resolution (Arducam OV9782 - USB Camera)
    public static final int kCameraResolutionWidth = 640;
    public static final int kCameraResolutionHeight = 480;

    // Camera center (pixel coordinates)
    public static final double kCameraCenterX = kCameraResolutionWidth / 2.0; // 320
    public static final double kCameraCenterY = kCameraResolutionHeight / 2.0; // 240

    // Distance calibration: distance = kDistanceCalibration / sqrt(targetArea%)
    // This constant needs to be calibrated by measuring distance vs observed area
    // Start with an estimate and adjust based on real measurements
    public static final double kDistanceCalibration = 0.5; // Adjust this value through testing

    // Vision odometry: blend factor for vision pose updates (0.0 = trust wheels only, 1.0 = trust vision only)
    public static final double kVisionOdometryBlendFactor = 0.5; // 50% trust vision updates

    // AprilTag tracking P gains for frame-based centering
    public static final double kYawTrackingP = 0.2; // rad/s per pixel offset in X
    public static final double kPitchTrackingP = 0.2; // rad/s per pixel offset in Y

    // Dead zone for tag tracking (pixels) - ignore small movements
    public static final double kTrackingCenterDeadzone = 15.0; // pixels

    // Maximum tracking speeds (rad/s)
    public static final double kMaxYawTrackingSpeed = Math.PI / 4; // 45°/s
    public static final double kMaxPitchTrackingSpeed = Math.PI / 6; // 30°/s

    // Search routine: delay before starting search after target lost (seconds)
    public static final double kSearchStartDelaySeconds = 1.0; // 1 second delay

    // Confidence threshold for vision pose estimates
    public static final double kMinAprilTagConfidence = 0.5;

    // Maximum distance to trust a vision pose estimate (meters)
    public static final double kMaxVisionPoseDistance = 5.0;
  }
}

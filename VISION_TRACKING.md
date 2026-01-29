# AprilTag Vision Tracking System

## Overview

This is Phase 1 of the turret vision tracking system. It uses **proportional (P-based) control** to automatically center AprilTags in the camera frame.

## Architecture

### Components

1. **AprilTagTracker** - NetworkTables-based vision subsystem
   - Reads AprilTag detection data from PhotonVision
   - Calculates proportional commands to center tag in frame
   - Cached entries reduce memory pressure

2. **RobotContainer** - Control logic integration
   - A button: Manual pitch control (stick-based)
   - X button: Manual yaw control (stick-based)  
   - B button: Automatic AprilTag tracking (vision-based)

3. **TwoAxisPositioner** - Motor control subsystem
   - Executes position commands from manual or vision sources
   - PID position control via SparkMax

### Control Flow

```
PhotonVision (Camera Feed)
         ↓
AprilTagTracker (reads NetworkTables)
         ↓
Calculates P-based commands (pixel offset → angular velocity)
         ↓
RobotContainer (integrates velocity → position setpoint)
         ↓
TwoAxisPositioner (position PID control)
         ↓
Motors (track AprilTag)
```

## Phase 1: Frame-Centered Tracking

### How It Works

1. **Detect AprilTag**: PhotonVision detects tag center in pixel coordinates
2. **Calculate Error**: Difference between tag center and frame center
3. **P Command**: `angular_velocity = pixel_offset × gain`
4. **Integrate**: `new_position = current_position + velocity × dt`
5. **Execute**: Motor PID moves to new position

### Key Parameters

See `Constants.VisionConstants`:

- **`kYawTrackingP`**: Yaw gain (rad/s per pixel offset)
  - Start: 0.02
  - Higher = more aggressive tracking
  - Lower = smoother but slower response

- **`kPitchTrackingP`**: Pitch gain (rad/s per pixel offset)
  - Start: 0.02
  - Same behavior as yaw

- **`kTrackingCenterDeadzone`**: Ignore small movements (pixels)
  - Start: 15 pixels (~5-10° at typical distance)
  - Prevents jitter when tag is nearly centered

- **`kMaxYawTrackingSpeed`**: Speed limit (rad/s)
  - Start: π/4 (45°/s)
  - Prevents overshooting

- **`kMaxPitchTrackingSpeed`**: Speed limit (rad/s)
  - Start: π/6 (30°/s)
  - Prevents overshooting

## Usage

### Manual Control (Testing)
```
Press A: Stick controls pitch (up/down)
Press X: Stick controls yaw (left/right)
```

### Automatic Tracking
```
Press B: Auto-track detected AprilTag
Release B or tag lost: Stop tracking
```

## Tuning Guide

### If Tracking is Jerky
- Increase deadzone: `kTrackingCenterDeadzone`
- Decrease gain: `kYawTrackingP`, `kPitchTrackingP`

### If Tracking is Too Slow
- Increase gain: `kYawTrackingP`, `kPitchTrackingP`
- Increase max speed: `kMaxYawTrackingSpeed`, `kMaxPitchTrackingSpeed`

### If Tracking Overshoots
- Decrease gain
- Decrease max speed
- Increase motor D gain in `Configs.java` (increase damping)

## Phase 2: Coordinate-Based Tracking (Planned)

Future improvements will add:

1. **World Coordinate System**: Track tag position in 3D space
2. **Dead Reckoning**: Estimate tag position when out of frame
3. **Multi-Tag Support**: Switch between multiple AprilTags
4. **Distance Estimation**: Use tag size in frame to estimate distance
5. **Smooth Following**: Separate velocity control from position control

This will allow:
- Continuing to track tag even if it temporarily leaves frame
- Smoother, more predictable motion
- Better multi-tag scenarios (e.g., switching to nearest tag)

## Technical Notes

### NetworkTables Structure

PhotonVision publishes to:
```
/photonvision/<camera_name>/
  - hasTarget: boolean
  - targetYaw: double (degrees, horizontal offset from center)
  - targetPitch: double (degrees, vertical offset from center)
  - targetArea: double (percentage of frame)
  - targetID: double (AprilTag ID)
```

### Camera Information

- **Camera**: Arducam OV2311 (color, for this iteration)
- **Resolution**: 1280×720
- **FOV**: ~48° horizontal
- **Connection**: USB via coprocessor → Robot via PhotonVision NT

### Performance

- **Update Rate**: ~40ms (throttled from 50Hz to reduce network traffic)
- **Memory**: Minimal (cached entries, no SmartDashboard output)
- **Responsiveness**: Adequate for slow target motion

## Troubleshooting

### Tag Not Detected
- Check PhotonVision web interface (5801)
- Verify camera is connected and streaming
- Ensure good lighting on AprilTag

### Tracking Unstable
- Check motor PID gains (might be too aggressive)
- Verify mechanical alignment
- Check for bent camera mount

### Tracking Unresponsive
- Increase P gain
- Check that B button is working
- Verify AprilTag is in frame

## Future Work

- [ ] Dead reckoning when tag lost
- [ ] Multi-tag support
- [ ] Distance-based scaling
- [ ] Velocity-based tracking
- [ ] Dashboard diagnostics

# AprilTag Tracking Framework

## Overview

This framework provides automatic turret tracking of AprilTags detected by PhotonVision running on a coprocessor. The system integrates with your two-axis positioner (pitch/yaw) to track detected targets.

## Architecture

### VisionTracker Subsystem
- **Location:** `src/main/java/frc/robot/subsystems/VisionTracker.java`
- **Purpose:** Reads AprilTag detection data from PhotonVision via NetworkTables
- **Key Methods:**
  - `getYawOffset()` - Horizontal angle offset to target (degrees)
  - `getPitchOffset()` - Vertical angle offset to target (degrees)
  - `hasTarget()` - Whether a target is currently detected
  - `getTargetId()` - ID of the detected AprilTag

### RobotContainer Integration
- **B Button:** Enables automatic AprilTag tracking
- **Fallback:** Holds position if no target is detected

## How It Works

### Control Modes

**A Button - Manual Control:**
```
Stick position → Turret angle
Live angle adjustment for tuning
```

**B Button - Auto-Tracking:**
```
PhotonVision detects AprilTag
  ↓
Vision offset extracted from NetworkTables
  ↓
Proportional correction applied to turret
  ↓
Turret automatically aligns with target
```

## PhotonVision Setup

### NetworkTables Structure

Your PhotonVision coprocessor publishes target data to:
```
/photonvision/[camera_name]/
  ├── hasTarget (boolean)       - Is a target detected?
  ├── targetYaw (double)        - Horizontal offset (degrees)
  ├── targetPitch (double)      - Vertical offset (degrees)
  └── targetID (int)            - AprilTag ID (0-36)
```

### Verifying Connection

1. Open SmartDashboard
2. Go to **Network Tables** → `/photonvision/photonvision/`
3. You should see the above entries updating
4. Check dashboard under `Vision/` tab for real-time feedback

## Control Code

In `RobotContainer.configureDefaultCommands()`:

```java
if (autoTrackMode) {
    if (m_vision.hasTarget()) {
        // Vision offsets in radians
        double yawOffset = Math.toRadians(m_vision.getYawOffset());
        double pitchOffset = Math.toRadians(m_vision.getPitchOffset());
        
        // Current positions
        double currentYaw = m_positioner.getYawPosition();
        double currentPitch = m_positioner.getPitchPosition();
        
        // Apply proportional correction
        double targetYaw = currentYaw + (yawOffset * 0.1);
        double targetPitch = currentPitch + (pitchOffset * 0.1);
        
        m_positioner.setYawPosition(targetYaw);
        m_positioner.setPitchPosition(targetPitch);
    } else {
        m_positioner.stop();
    }
}
```

## Tuning the Tracker

### Proportional Gain (0.1 in current code)

**What it does:** Controls how aggressively the turret tracks targets
- **0.05** = Slow, smooth tracking (good for far targets)
- **0.1** = Moderate tracking (current default)
- **0.2** = Aggressive tracking (may oscillate)

**How to tune:**
1. Hold B button while pointing at an AprilTag
2. Observe turret response time
3. Adjust value in RobotContainer if too slow/fast
4. Recompile and deploy

### Gain Adjustment Example

```java
// In RobotContainer.java configureDefaultCommands()
// Change this line:
double targetYaw = currentYaw + (yawOffset * 0.1);  // ← Adjust 0.1

// Try values like 0.05, 0.15, 0.2 based on needs
```

## Dashboard Feedback

The system publishes real-time data under `Vision/` tab:
- **Has Target** - Boolean indicator of detection
- **Target ID** - Which AprilTag (0-36)
- **Yaw Offset (deg)** - How many degrees to turn left/right
- **Pitch Offset (deg)** - How many degrees to pitch up/down

## Advanced Features (Future)

### Possible Enhancements

1. **Distance-based gain scaling**
   - Apply different tracking gains based on distance
   - Close targets use lower gain (smoother)
   - Far targets use higher gain (faster response)

2. **Multiple target handling**
   - Track specific AprilTag IDs
   - Ignore certain tags if desired

3. **Target filtering**
   - Ignore targets with poor confidence
   - Only track targets within a certain area of view

4. **Integration with trajectory estimation**
   - Predict moving target positions
   - Lead targets for better accuracy

5. **Backup to manual mode**
   - If vision fails, release B to use manual (A) control
   - Graceful degradation

## Troubleshooting

| Issue | Likely Cause | Solution |
|-------|-------------|----------|
| "Has Target" always false | PhotonVision not running | Check coprocessor connection |
| Turret doesn't track | Camera name mismatch | Verify `"photonvision"` matches NT entry |
| Jerky tracking | Gain too high | Lower proportional gain (0.1 → 0.05) |
| Slow response | Gain too low | Raise proportional gain (0.1 → 0.15) |
| Oscillates at target | Proportional gain too high + no damping | Lower gain or add D term for damping |

## Next Steps

1. **Test on robot** - Deploy code and verify vision data appears in SmartDashboard
2. **Tune tracking gain** - Adjust 0.1 value for desired response
3. **Test different distances** - Ensure tracking works across field
4. **Integrate with game strategy** - Use tracking in autonomous/teleop sequences

## Reference Documentation

- PhotonVision: https://docs.photonvision.org/
- AprilTag Standard: https://april.eecs.umich.edu/software/apriltag/
- WPILib NetworkTables: https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html

# PID Tuning Guide via SmartDashboard

## How to Use

1. **Deploy the code to the robot**
   ```powershell
   .\gradlew deploy
   ```

2. **Open SmartDashboard** and navigate to the **Tuner** tab

3. **You'll see 6 editable fields:**
   - `Tuner/Pitch/P` - Pitch proportional gain
   - `Tuner/Pitch/I` - Pitch integral gain  
   - `Tuner/Pitch/D` - Pitch derivative gain
   - `Tuner/Yaw/P` - Yaw proportional gain
   - `Tuner/Yaw/I` - Yaw integral gain
   - `Tuner/Yaw/D` - Yaw derivative gain

4. **Adjust values while holding A button to command setpoints**
   - The motors will respond in real-time as you change P, I, D
   - Watch for oscillation, overshoot, and settle time
   - Console output will log each change

## Tuning Strategy for Yaw (or any axis with jitter)

### Phase 1: Find Minimum P
- Set `Tuner/Yaw/D = 0.4` (keep constant)
- Lower `Tuner/Yaw/P` gradually: 0.2 → 0.15 → 0.1
- Command to 90° and measure time to settle
- Find lowest P where response is acceptable (1-2 seconds)
- **Baseline:** P ≈ 0.15

### Phase 2: Tune D for Damping
- Keep P from Phase 1 constant
- Increase `Tuner/Yaw/D` gradually: 0.4 → 0.5 → 0.6 → 0.7
- Watch for oscillations to reduce smoothly
- Stop when overshoot is minimal but response isn't sluggish
- **Goal:** D ≈ 1.5-2x your final P
- **Example:** If P=0.15, try D=0.25-0.3

### Phase 3: Verify Across Range
- Test multiple setpoints: 0°, 45°, 90°, 135°, 180°
- Verify behavior is consistent
- If asymmetric, may indicate mechanical issues

### What to Monitor

**Dashboard Values:**
- `Positioner/Current Yaw (deg)` - Actual position
- `Positioner/Yaw Setpoint` - Target position
- `Positioner/Yaw Motor Current` - Load on motor
- `Positioner/Yaw Motor Voltage` - Command voltage

**Console Output:**
```
PID Tuner [Yaw] Updated: P=0.15, I=0.00, D=0.45
```

## Red Flags

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Never settles, constant oscillation | P too high, D too low | ↓ P or ↑ D |
| Sluggish response, overshoots heavily | D too high | ↓ D |
| Motor draws high current constantly | P way too high | ↓↓ P significantly |
| Jitter at setpoint even with deadzone | P/D ratio wrong | Rebalance P and D |

## Once Tuned

When you find good values, update `Configs.java`:

```java
// In TwoAxisPositioner configuration
yawConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(0.15, 0.0, 0.45)  // Your tuned values here
```

Then set deadband in `Constants.java` to help with final stability:
```java
public static final double kPositionerPositionDeadband = Math.toRadians(1.0); // ±1°
```

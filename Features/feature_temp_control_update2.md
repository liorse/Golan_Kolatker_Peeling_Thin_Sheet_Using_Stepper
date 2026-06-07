# Temperature Control Implementation

## PID Parameters (from analysis)

```
KP = 60.0
KI = 0.15
KD = 900.0
PID_BAND = 4.0°C   # Bang-bang outside, PID inside
```

## Control Algorithm

```python
def compute_heater_pwm(current_temp, setpoint, dt):
    error = setpoint - current_temp

    # 1. Safety cutoffs (skip PID entirely)
    if isnan(current_temp) or current_temp >= 120.0:
        return 0.0

    # 2. Bang-Bang outside active zone
    if error > PID_BAND:
        return 100.0   # Full power for fast warmup
    if error < -PID_BAND:
        return 0.0     # Turn off if over-temperature

    # 3. PID within the zone
    P_term = KP * error

    integral_stored += error * dt
    integral_stored = clamp(integral_stored, -50/KI, +50/KI)  # I-term contribution ±50%
    I_term = KI * integral_stored

    raw_derivative = (error - last_error) / dt
    filtered_derivative = 0.7 * filtered_derivative + 0.3 * raw_derivative  # low-pass
    D_term = KD * filtered_derivative

    last_error = error
    output = P_term + I_term + D_term
    return clamp(output, 0.0, 100.0)
```

## Design Decisions

| # | Decision |
|---|---|
| 1 | PID runs inside the temp-read `if` block, right after `lastTempC` is set, using `dt = (nowMs - lastPidRunMs) / 1000.0f` |
| 2 | PID writes `heaterDuty` directly (0–255); `h` serial command is blocked with log when `tempControlActive == true` |
| 3 | `integral_stored`, `last_error`, `filteredDerivative` reset to 0 only on false→true activation |
| 4 | Heater zeroed immediately on true→false deactivation |
| 5 | `isnan(lastTempC)` → force off; `lastTempC >= 120.0°C` → force off; both skip PID |
| 6 | I-term clamped to ±50 (integral_stored clamped to `±50/ki`) |
| 7 | Derivative low-pass filtered: `filteredDerivative = 0.7f * prev + 0.3f * raw` |
| 8 | `tempControlActive` always boots `false` regardless of EEPROM |
| 9 | First tick after activation skips computation, just records `lastPidRunMs` |

## New State Variables

```cpp
float         pidIntegral        = 0.0f;
float         pidLastError        = 0.0f;
float         pidFilteredDeriv    = 0.0f;
unsigned long lastPidRunMs        = 0;
```

All reset to 0 on activation; `lastPidRunMs = 0` triggers first-tick skip.

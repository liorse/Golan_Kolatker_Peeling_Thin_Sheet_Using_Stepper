It is time to implement the temperature control logic.

following an analysis can you build it following:

you already have the PID parameters and knowledge on how to control the power.

# Constants based on analysis
KP = 60.0
KI = 0.15
KD = 900.0

# Anti-Windup Zone
PID_BAND = 4.0  # Only enable PID within 4°C of target

def compute_heater_pwm(current_temp, setpoint, dt):
    error = setpoint - current_temp
    
    # 1. Bang-Bang Control outside the active zone
    if error > PID_BAND:
        return 100.0  # Full power for fast warmup
    if error < -PID_BAND:
        return 0.0   # Turn off completely if over-temperature
        
    # 2. Smooth PID Control within the zone
    global integral_stored, last_error
    
    P_term = KP * error
    
    # Integral with anti-windup clamping
    integral_stored += error * dt
    I_term = KI * integral_stored
    
    # Derivative term (ideally feed a filtered temperature derivative)
    derivative = (error - last_error) / dt
    D_term = KD * derivative
    
    last_error = error
    
    # Total Output
    output = P_term + I_term + D_term
    return max(0.0, min(100.0, output))  # Clamp to 0-100% PWM duty
# PID Class for Reusability
class PIDController:
    def __init__(self, kp, ki, kd, max_output=float('inf'), min_output=-float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        
        # Internal variables
        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        """Resets the PID controller's internal state."""
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        """Compute PID output based on error and time step."""
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        # PID output
        output = p_term + i_term + d_term
        
        # Clamping to max/min
        output = max(self.min_output, min(output, self.max_output))
        return output


# PID Gains (Initial Tuning)
# Position PID gains
pos_pid_x = PIDController(kp=1.0, ki=0.1, kd=0.2, max_output=5.0, min_output=-5.0)  # For x-axis
pos_pid_y = PIDController(kp=1.0, ki=0.1, kd=0.2, max_output=5.0, min_output=-5.0)  # For y-axis
pos_pid_z = PIDController(kp=1.2, ki=0.1, kd=0.3, max_output=3.0, min_output=-3.0)  # For z-axis

# Yaw PID gains
yaw_pid = PIDController(kp=1.5, ki=0.0, kd=0.2, max_output=2.0, min_output=-2.0)


# Main Controller
def controller(state, target_pos, dt):
    """
    PID Controller for UAV position and yaw.
    
    state format: [position_x, position_y, position_z, roll, pitch, yaw]
    target_pos format: (x, y, z, yaw)
    dt: time step (s)
    
    Returns:
    - velocity_x_setpoint (m/s)
    - velocity_y_setpoint (m/s)
    - velocity_z_setpoint (m/s)
    - yaw_rate_setpoint (radians/s)
    """
    # Extract state and target information
    pos_x, pos_y, pos_z, _, _, yaw = state
    target_x, target_y, target_z, target_yaw = target_pos
    
    # Calculate position errors
    error_x = target_x - pos_x
    error_y = target_y - pos_y
    error_z = target_z - pos_z
    error_yaw = target_yaw - yaw
    
    # Normalize yaw error to [-pi, pi] for smooth control
    error_yaw = (error_yaw + 3.14159) % (2 * 3.14159) - 3.14159

    # Outer loop: Position PID to generate velocity setpoints
    vel_x_setpoint = pos_pid_x.compute(error_x, dt)
    vel_y_setpoint = pos_pid_y.compute(error_y, dt)
    vel_z_setpoint = pos_pid_z.compute(error_z, dt)
    
    # Yaw PID to generate yaw rate setpoint
    yaw_rate_setpoint = yaw_pid.compute(error_yaw, dt)

    # Return velocity setpoints for the UAV
    return (vel_x_setpoint, vel_y_setpoint, vel_z_setpoint, yaw_rate_setpoint)


# Example Usage
if __name__ == "__main__":
    # Simulated state and target for testing
    state = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]  # Initial position and attitude
    target_pos = (2.0, 3.0, 1.5, 1.57)  # Target position and yaw

    dt = 0.01  # 10 ms time step

    for i in range(100):
        vel_cmd = controller(state, target_pos, dt)
        print(f"Velocity Command: {vel_cmd}")

class AttitudePDController:
    
    def __init__(self, kp, kd, base_thrust):
        """
        kp, kd: listy 3-elementowe [roll, pitch, yaw]
        base_thrust: stały ciąg (np. 1500 µs na ESC)
        """
        self.kp = kp
        self.kd = kd
        self.base_thrust = base_thrust
        # self.prev_error = [0.0, 0.0, 0.0]

    def update(self, desired_angles, current_angles, angular_velocities):
        """
        desired_angles: [roll, pitch, yaw] w stopniach lub radianach
        current_angles: [roll, pitch, yaw]
        angular_velocities: prędkości kątowe w poszczególnych osiach
        """

        errors = [d - c for d, c in zip(desired_angles, current_angles)]
        d_errors = [-w for w in angular_velocities]  # zamiast różniczki
        moments = [
            self.kp[i] * errors[i] + self.kd[i] * d_errors[i]
            for i in range(3)
        ]
        return moments, self.base_thrust


controller = AttitudePDController(
    kp=[1.2, 1.2, 1.5],  # roll, pitch, yaw
    kd=[0.3, 0.3, 0.4],
    base_thrust=1500
)


desired = [0, 0, 0]
current_angles = [5, -3, 10]
angular_velocities = [0.91, -0.2, 0.05]  # np. z żyroskopu
moments, thrust = controller.update(desired, current_angles, angular_velocities)
print("Moment roll/pitch/yaw:", moments)
print("Thrust:", thrust)


# Wejście:
# - różnice w kątach: yaw pitch i roll, i prędkości czyli pochodne

# Wyjścia:
# t1,t2,t3 - momenty obrotowe w danej osi
# T = całkowity ciąg drona 
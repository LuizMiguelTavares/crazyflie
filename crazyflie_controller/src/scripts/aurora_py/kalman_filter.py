import numpy as np
import sympy as sp

class KalmanFilter:
    def __init__(self, A, B, C, D, Rw, Rv, initial_state, initial_error_covariance, Ts):
        self.A = (np.eye(A.shape[0]) + np.array(A)*Ts)
        self.B = np.array(B)*Ts

        self.C = np.array(C)
        self.D = np.array(D)
        self.Ts = Ts
        self.Rw = np.array(Rw) # Process noise covariance
        self.Rv = np.array(Rv) # Measurement noise covariance
        self.state_estimate = np.array(initial_state) # Initial state estimate
        self.error_covariance = np.array(initial_error_covariance) # Initial error covariance

    def update(self, measurement, control_input):
        measurement = np.array(measurement)
        control_input = np.array(control_input)

        # Innovation
        innovation = measurement - self.C @ self.state_estimate - self.D @ control_input
        innovation_covariance = self.C @ self.error_covariance @ self.C.T + self.Rv

        # Correction
        kalman_gain = self.error_covariance @ self.C.T @ np.linalg.pinv(innovation_covariance)
        self.state_estimate = self.state_estimate + kalman_gain @ innovation
        self.error_covariance = (np.eye(self.A.shape[0]) - kalman_gain @ self.C) @ self.error_covariance

        # Prediction
        self.old_state = self.state_estimate
        self.state_estimate = self.A @ self.state_estimate + self.B @ control_input
        self.error_covariance = self.A @ self.error_covariance @ self.A.T + self.Rw

        self.state_estimate[2] = self.old_state[2] + self.Ts * self.old_state[5]
        self.state_estimate[5] = self.old_state[5] + self.Ts * (1/0.035 * control_input[2] - 9.81)

        return self.state_estimate, kalman_gain

class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_error_covariance, dt, m, Kvx, Kvy, Kvz, Rw, Rv):
        # Symbolic variables for state and control input
        self.x, self.y, self.z, self.x_dot_body, self.y_dot_body, self.z_dot_body = sp.symbols('x y z x_dot_body y_dot_body z_dot_body')
        self.theta, self.phi, self.psi, self.t = sp.symbols('theta phi psi t')

        # State and control input as NumPy arrays
        self.state_estimate = np.array(initial_state)
        self.error_covariance = np.array(initial_error_covariance)

        # Constants and time step
        self.dt = dt
        self.m = m
        self.g = 9.81
        self.Kvx = Kvx
        self.Kvy = Kvy
        self.Kvz = Kvz

        # Process and measurement noise covariance matrices
        self.Rw = np.array(Rw)
        self.Rv = np.array(Rv)

        # Precompute the symbolic state transition function and its Jacobian
        self._precompute_dynamics()

    def _precompute_dynamics(self):
        # Define the symbolic state transition equations
        state = sp.Matrix([self.x, self.y, self.z, self.x_dot_body, self.y_dot_body, self.z_dot_body])
        control_input = sp.Matrix([self.theta, self.phi, self.psi, self.t])

        # World frame dynamics
        x_world = sp.cos(self.psi) * self.x_dot_body + sp.sin(self.psi) * self.y_dot_body
        y_world = -sp.sin(self.psi) * self.x_dot_body + sp.cos(self.psi) * self.y_dot_body
        z_world = self.z_dot_body
        x_ddot_w = -self.Kvx / self.m * self.x_dot_body + self.t / self.m * sp.sin(self.theta)
        y_ddot_w = -self.Kvy / self.m * self.y_dot_body + self.t / self.m * sp.sin(self.phi)
        z_ddot_w = -self.Kvz / self.m * self.z_dot_body + self.t / self.m * sp.cos(self.theta) * sp.cos(self.phi) - self.g

        # State transition equations
        new_state = sp.Matrix([
            self.x + x_world * self.dt,
            self.y + y_world * self.dt,
            self.z + z_world * self.dt,
            self.x_dot_body + x_ddot_w * self.dt,
            self.y_dot_body + y_ddot_w * self.dt,
            self.z_dot_body + z_ddot_w * self.dt
        ])

        # Compute the Jacobian matrix of the state transition function
        self.jacobian_func = sp.lambdify((state, control_input), new_state.jacobian(state), 'numpy')

    def update(self, measurement, control_input):
        # Prediction
        control_input_np = np.array(control_input, dtype=float)
        jacobian_matrix = self.jacobian_func(self.state_estimate, control_input_np)

        # State transition (numerical computation)
        self.state_estimate = self._state_transition(self.state_estimate, control_input_np)
        self.error_covariance = jacobian_matrix @ self.error_covariance @ jacobian_matrix.T + self.Rw

        # Update
        H = np.eye(6)  # Measurement matrix
        innovation = measurement - H @ self.state_estimate
        innovation_covariance = H @ self.error_covariance @ H.T + self.Rv
        kalman_gain = self.error_covariance @ H.T @ np.linalg.pinv(innovation_covariance)
        self.state_estimate = self.state_estimate + kalman_gain @ innovation
        self.error_covariance = (np.eye(6) - kalman_gain @ H) @ self.error_covariance

        return self.state_estimate, kalman_gain

    def _state_transition(self, state, control_input):
        # Unpack the state and control input
        x, y, z, x_dot_body, y_dot_body, z_dot_body = state
        theta, phi, psi, t = control_input

        # Calculate the world frame dynamics
        x_world = np.cos(psi) * x_dot_body + np.sin(psi) * y_dot_body
        y_world = -np.sin(psi) * x_dot_body + np.cos(psi) * y_dot_body
        z_world = z_dot_body
        x_ddot_w = -self.Kvx / self.m * x_dot_body + t / self.m * np.sin(theta)
        y_ddot_w = -self.Kvy / self.m * y_dot_body + t / self.m * np.sin(phi)
        z_ddot_w = -self.Kvz / self.m * z_dot_body + t / self.m * np.cos(theta) * np.cos(phi) - self.g

        # State transition
        new_state = np.array([
            x + x_world * self.dt,
            y + y_world * self.dt,
            z + z_world * self.dt,
            x_dot_body + x_ddot_w * self.dt,
            y_dot_body + y_ddot_w * self.dt,
            z_dot_body + z_ddot_w * self.dt
        ])

        return new_state
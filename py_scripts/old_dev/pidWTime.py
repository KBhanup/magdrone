import numpy as np

class PIDTimedController():
    '''
        PID controller implementation with limited integration horizon
    '''

    def __init__(self, Kp, Ki, Kd, integral_horizon):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd

        self.horizon = integral_horizon

        self.errors = None

    # Reset controller
    def resetCtr(self):
        self.errors = None
    
    # Get inegral of error
    def getIntegral(self):
        i = 0
        for e in range(self.errors.shape[0] - 1):
            e_i  = (self.errors[e, 0] + self.errors[e + 1, 0]) / 2.0
            dt_i = self.errors[e + 1, 1] - self.errors[e, 1]
            i += e_i * dt_i

        return i

    # Update the error value
    def updateError(self, error, t):
        if self.errors is None:
            self.errors = np.array([[error, t]])
        else:
            self.errors = np.append(self.errors, [[error, t]], axis = 0)
        
        if self.errors.shape[0] > 1:
            while self.errors[-1, 1] - self.errors[0, 1] > self.horizon:
                self.errors = np.delete(self.errors, 0, 0)

    # Get command from controller
    def getCommand(self):
        if not self.errors is None:
            # Proportional
            cmd = self.kp * (self.errors[-1, 0])

            if self.errors.shape[0] > 1:
                # Derivative
                cmd += self.kd * ((self.errors[-1, 0] - self.errors[-2, 0]) / (self.errors[-1, 1] - self.errors[-2, 1]))
                # Integral
                cmd += self.ki * self.getIntegral()

            return cmd
        else:
            return 0.0

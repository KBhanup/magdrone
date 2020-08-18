import numpy as np

class PIDcontroller():
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
        return np.sum(self.errors)

    # Update the error value
    def updateError(self, error):
        if self.errors is None:
            self.errors = np.array([error])
        else:
            self.errors = np.append(self.errors, [error])

        if len(self.errors) > self.horizon:
            self.errors = np.delete(self.errors, 0)

    # Get command from controller
    def getCommand(self):
        if not self.errors is None:
            # Proportional
            cmd = self.kp * (self.errors[-1])

            if len(self.errors) > 1:
                # Derivative
                cmd += self.kd * (self.errors[-1] - self.errors[-2])
                # Integral
                cmd += self.ki * self.getIntegral()

            return cmd
        else:
            return 0.0

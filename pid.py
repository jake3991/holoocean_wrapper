
class Control:
    """A class to handle some very basic control of the ROV
    """

    def __init__(self):

        self.kp = 2.0 # some constants for a PID loop
        self.ki = 1.0
        self.kd = 1.0
    
    def control_depth(self, current_depth : float, set_point : float) -> float:
        """Set the vertical thruster command using proportional control

        Args:
            current_depth (float): the current depth of the ROV
            set_point (float): the desired depth of the ROV

        Returns:
            float: the vertical thruster command
        """

        error = -1 * (current_depth - set_point) * self.kp

        return error


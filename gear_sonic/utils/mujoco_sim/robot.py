"""Legacy robot config container (deprecated -- prefer robot_model package)."""


# TODO: This class and associated configs should be deleted and instead use the robot_model package
class Robot:
    def __init__(self, config):
        self.ROBOT_TYPE = config["ROBOT_TYPE"]
        self.MOTOR2JOINT = config["MOTOR2JOINT"]
        self.JOINT2MOTOR = config["JOINT2MOTOR"]
        self.UNITREE_LEGGED_CONST = config["UNITREE_LEGGED_CONST"]
        self.MOTOR_KP = config["MOTOR_KP"]
        self.MOTOR_KD = config["MOTOR_KD"]
        self.WeakMotorJointIndex = config["WeakMotorJointIndex"]
        self.NUM_MOTORS = config["NUM_MOTORS"]
        self.NUM_JOINTS = config["NUM_JOINTS"]
        self.DEFAULT_DOF_ANGLES = config["DEFAULT_DOF_ANGLES"]
        self.DEFAULT_MOTOR_ANGLES = config["DEFAULT_MOTOR_ANGLES"]
        self.USE_SENSOR = config["USE_SENSOR"]
        self.MOTOR_EFFORT_LIMIT_LIST = config["motor_effort_limit_list"]

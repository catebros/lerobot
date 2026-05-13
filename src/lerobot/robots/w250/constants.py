import math

W250_JOINT_NAMES = [
    "waist",
    "shoulder",
    "elbow",
    "forearm_roll",
    "wrist_angle",
    "wrist_rotate",
]


W250_JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "waist":        (-math.pi, math.pi),
    "shoulder":     (-1.88, 1.99),
    "elbow":        (-2.15, 1.60),
    "forearm_roll": (-math.pi, math.pi),
    "wrist_angle":  (-1.745, 2.15),
    "wrist_rotate": (-math.pi, math.pi),
}

W250_HOME_POSITION = {
    "waist.pos": 0.0,
    "shoulder.pos": 0.0,
    "elbow.pos": 0.0,
    "forearm_roll.pos": 0.0,
    "wrist_angle.pos": 0.0,
    "wrist_rotate.pos": 0.0,
    "gripper.pos": 0.0,
}

W250_REST_POSITION = {
    "waist.pos": 0.5,
    "shoulder.pos": -0.6,
    "elbow.pos": 0.4,
    "forearm_roll.pos": 0.0,
    "wrist_angle.pos": 0.73,
    "wrist_rotate.pos": 0.0,
    "gripper.pos": 1.0,
}

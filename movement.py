from enum import Enum
from turtle import forward

class Movement(Enum):
    tracking = 1        # 延循線方向前進
    slow_tracking = 2   # 延循線方向慢速前進
    braking = 3         # 煞車
    hover = 4           # 懸停
    first_forward = 5   # 給一個初始力道往原先循跡方向飛
    forward = 6         # 往原先循跡方向飛
    last_forward = 7    # 給一個初始力道往目前循跡方向飛
    up = 8              # 上升
    # down = 9          # 下降
    land = 9            # 降落

class State(Enum):
    tracking = 1
    alert = 2
    avoid = 3
    landing = 4

class Task(Enum):
    start = 1
    hover = 2
    up = 3
    forward_toforward = 4
    forward_todown   = 5
    # down = 5

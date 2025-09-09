
# ur5_motion/ur5_motion/robot_description.py (Pydantic v2 + runtime validation)
from __future__ import annotations
from typing import List, Tuple
import numpy as np

from pydantic import BaseModel, Field, field_validator, ValidationError

class JointLimit(BaseModel):
    min: float
    max: float

    @field_validator("max")
    @classmethod
    def check_order(cls, v, info):
        minv = info.data.get("min")
        if minv is not None and v <= minv:
            raise ValueError("max must be greater than min")
        return v

class RobotDescription(BaseModel):
    joint_names: List[str] = Field(..., min_length=6, max_length=6)
    a: object
    alpha: object
    d: object
    joint_limits: List[JointLimit] = Field(..., min_length=6, max_length=6)
    tool_T: object

    # --- validators ---
    @field_validator("a", "alpha", "d", mode="before")
    @classmethod
    def to_numpy_1d(cls, v):
        arr = np.array(v, dtype=float).reshape(-1)
        if arr.shape != (6,):
            raise ValueError("Expected shape (6,)")
        return arr

    @field_validator("tool_T", mode="before")
    @classmethod
    def to_numpy_4x4(cls, v):
        arr = np.array(v, dtype=float)
        if arr.shape != (4,4):
            raise ValueError("tool_T must be 4x4")
        if not np.allclose(arr[3,:], [0,0,0,1]):
            raise ValueError("tool_T last row must be [0,0,0,1]")
        return arr

def ur5_description() -> RobotDescription:
    joint_names = [
        'shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
        'wrist_1_joint','wrist_2_joint','wrist_3_joint'
    ]
    # UR5e DH parameters (in meters)
    a     = [0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0]
    alpha = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
    d     = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
    
    joint_limits = [
        JointLimit(min=-2*np.pi, max=2*np.pi),
        JointLimit(min=-2*np.pi, max=2*np.pi),
        JointLimit(min=-2*np.pi, max=2*np.pi),
        JointLimit(min=-2*np.pi, max=2*np.pi),
        JointLimit(min=-2*np.pi, max=2*np.pi),
        JointLimit(min=-2*np.pi, max=2*np.pi),
    ]

    tool_T = np.eye(4) # Identity matrix for the tool
    
    return RobotDescription(
        joint_names=joint_names,
        a=a,
        alpha=alpha,
        d=d,
        joint_limits=joint_limits,
        tool_T=tool_T
    )

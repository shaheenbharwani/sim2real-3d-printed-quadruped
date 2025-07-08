# Isaac Lab Training Configuration
# This file configures the quadruped robot training in Isaac Lab simulation

import math
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp

# Robot Configuration
MY_QUADRUPED_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        # Update this path to point to your USD file
        usd_path="./simulation/assets/quad5.usd",
        activate_contact_sensors=True,
    ),
    # Joint configuration optimized for walking
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, -0.5),
        rot=(0.7071, 0.7071, 0.0, 0.0),
        joint_pos={
            "front_left_hip": 0.0, "front_right_hip": 0.0,
            "rear_left_hip": 0.0, "rear_right_hip": 0.0,
            "front_left_knee": -1.571, "front_right_knee": 1.571,
            "rear_left_knee": -1.571, "rear_right_knee": 1.571,
            "front_left_ankle": 1.571, "front_right_ankle": -1.571,
            "rear_left_ankle": 1.571, "rear_right_ankle": -1.571,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={"body": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=120.0, damping=20.0)},
)

# Training Environment Configuration
@configclass
class MyQuadrupedEnvCfg(ManagerBasedRLEnvCfg):
    """Complete training environment for quadruped locomotion."""
    
    # Scene with robot and terrain
    scene = InteractiveSceneCfg(
        num_envs=50, env_spacing=100,
        terrain=TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane"),
        robot=MY_QUADRUPED_CFG,
        light=AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DistantLightCfg()),
    )
    
    # Velocity commands
    commands = {
        "base_velocity": {
            "resampling_time_range": (10.0, 10.0),
            "ranges": {"lin_vel_x": (0.1, 0.4), "lin_vel_y": (-0.2, 0.2), "ang_vel_z": (-0.3, 0.3)},
        }
    }
    
    # 72D observation space (matches deployment)
    observations = {
        "policy": {
            "base_lin_vel": {"func": "base_lin_vel"},
            "base_ang_vel": {"func": "base_ang_vel"},
            "projected_gravity": {"func": "projected_gravity"},
            "velocity_commands": {"func": "generated_commands"},
            "joint_pos_rel": {"func": "joint_pos_rel"},
            "joint_vel_rel": {"func": "joint_vel_rel"},
            "actions": {"func": "last_action"},
            "feet_forces": {"func": "body_incoming_wrench", "scale": 0.1},
        }
    }
    
    # Reward structure for walking
    rewards = {
        "base_position_tracking": {"func": "track_lin_vel_xy_exp", "weight": 25.0},
        "track_ang_vel_z_exp": {"func": "track_ang_vel_z_exp", "weight": 4.0},
        "joint_deviation_l1": {"func": "joint_deviation_l1", "weight": -0.2},
        "joint_pos_limits": {"func": "joint_pos_limits", "weight": -2.0},
        "flat_orientation_l2": {"func": "flat_orientation_l2", "weight": -0.2},
    }
    
    def __post_init__(self):
        self.decimation = 2
        self.episode_length_s = 20.0
        self.sim.dt = 1 / 100.0  # 100Hz simulation

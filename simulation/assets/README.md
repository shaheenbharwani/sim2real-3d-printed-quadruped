# Isaac Lab Assets

## Robot Model Files

### USD File Location
The trained robot model is: `C:/Users/Shaheen Bharwani/Documents/SCI-Arc/Thesis/Original_Quad_URDF_Export/quad5.usd`

**Note**: This 7MB USD file contains the complete robot model with 165 mesh components. To use:
1. Copy `quad5.usd` to `simulation/assets/` 
2. Update path in `my_quadruped_config.py` if needed

### Training Results
- **Episodes**: 999 (approximately 2 hours)
- **Model Export**: `/mnt/d/IsaacLab/logs/rsl_rl/my_quadruped/2025-06-16_22-35-31/exported/policy.pt`
- **Observation Space**: 72 dimensions
- **Action Space**: 12 joint positions

### Joint Configuration
Robot uses `rear_*` joint naming (not `back_*`) to match URDF:
- 4 legs × 3 joints = 12 DOF total
- Hip, knee, ankle per leg
- Position control with stiffness=120.0, damping=20.0

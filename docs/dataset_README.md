---
task_categories:
- robotics
---

# LAFAN1 Retargeting Dataset

<video controls autoplay src="https://cdn-uploads.huggingface.co/production/uploads/67639932ad38702e6c8d16d9/V7InyG4CAh5NhUXILTK9b.mp4"></video>

To make the motion of humanoid robots more natural, we retargeted [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) motion capture data to [Unitree](https://www.unitree.com/)'s humanoid robots, supporting three models: [H1, H1_2](https://www.unitree.com/h1), and [G1](https://www.unitree.com/g1). This retargeting was achieved through numerical optimization based on [Interaction Mesh](https://ieeexplore.ieee.org/document/6651585) and IK, considering end-effector pose constraints, as well as joint position and velocity constraints, to prevent foot slippage. It is important to note that the retargeting only accounted for kinematic constraints and did not include dynamic constraints or actuator limitations. As a result, the robot cannot perfectly execute the retargeted trajectories.

# How to visualize robot trajectories?

```shell
# Step 1: Set up a Conda virtual environment
conda create -n retarget python=3.10
conda activate retarget

# Step 2: Install dependencies
conda install pinocchio -c conda-forge
pip install numpy rerun-sdk trimesh

# Step 3: Run the script
python rerun_visualize.py
# run the script with parameters:
# python rerun_visualize.py --file_name dance1_subject2 --robot_type [g1|h1|h1_2]
```

# Dataset Collection Pipeline

![image/png](https://cdn-uploads.huggingface.co/production/uploads/67639932ad38702e6c8d16d9/_tAr3zwPotJGaUqxe2u4p.png)

This database stores the retargeted trajectories in CSV format. Each row in the CSV file corresponds to the original motion capture data for each frame, recording the configurations of all joints in the humanoid robot in the following order:

```txt
The Order of Configuration
G1: (30 FPS)
    root_joint(XYZQXQYQZQW)
    left_hip_pitch_joint
    left_hip_roll_joint
    left_hip_yaw_joint
    left_knee_joint
    left_ankle_pitch_joint
    left_ankle_roll_joint
    right_hip_pitch_joint
    right_hip_roll_joint
    right_hip_yaw_joint
    right_knee_joint
    right_ankle_pitch_joint
    right_ankle_roll_joint
    waist_yaw_joint
    waist_roll_joint
    waist_pitch_joint
    left_shoulder_pitch_joint
    left_shoulder_roll_joint
    left_shoulder_yaw_joint
    left_elbow_joint
    left_wrist_roll_joint
    left_wrist_pitch_joint
    left_wrist_yaw_joint
    right_shoulder_pitch_joint
    right_shoulder_roll_joint
    right_shoulder_yaw_joint
    right_elbow_joint
    right_wrist_roll_joint
    right_wrist_pitch_joint
    right_wrist_yaw_joint
H1_2: (30 FPS)
    root_joint(XYZQXQYQZQW)
    left_hip_yaw_joint
    left_hip_pitch_joint
    left_hip_roll_joint
    left_knee_joint
    left_ankle_pitch_joint
    left_ankle_roll_joint
    right_hip_yaw_joint
    right_hip_pitch_joint
    right_hip_roll_joint
    right_knee_joint
    right_ankle_pitch_joint
    right_ankle_roll_joint
    torso_joint
    left_shoulder_pitch_joint
    left_shoulder_roll_joint
    left_shoulder_yaw_joint
    left_elbow_joint
    left_wrist_roll_joint
    left_wrist_pitch_joint
    left_wrist_yaw_joint
    right_shoulder_pitch_joint
    right_shoulder_roll_joint
    right_shoulder_yaw_joint
    right_elbow_joint
    right_wrist_roll_joint
    right_wrist_pitch_joint
    right_wrist_yaw_joint
H1: (30 FPS)
    root_joint(XYZQXQYQZQW)
    left_hip_yaw_joint
    left_hip_roll_joint
    left_hip_pitch_joint
    left_knee_joint
    left_ankle_joint
    right_hip_yaw_joint
    right_hip_roll_joint
    right_hip_pitch_joint
    right_knee_joint
    right_ankle_joint
    torso_joint
    left_shoulder_pitch_joint
    left_shoulder_roll_joint
    left_shoulder_yaw_joint
    left_elbow_joint
    right_shoulder_pitch_joint
    right_shoulder_roll_joint
    right_shoulder_yaw_joint
    right_elbow_joint
```

[LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) is licensed under Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International Public License (unlike the code, which is licensed under MIT).

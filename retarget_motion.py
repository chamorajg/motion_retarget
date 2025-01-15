import argparse
import glob
import os
from typing import Dict, List, Tuple

import numpy as np
import pinocchio as pin
import torch
import yaml
from tqdm import tqdm


class RetargetingConfig:
    def __init__(self, config_path: str):
        """Load retargeting configuration from YAML file"""
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        self.source_robot = config["source_robot"]
        self.target_robot = config["target_robot"]
        self.height_offset = config.get("height_offset", 0.0)
        self.joint_mapping = config["joint_pairs"]


class MotionRetargeting:
    def __init__(
        self, config: RetargetingConfig, source_urdf: str, target_urdf: str
    ):
        self.config = config
        self.source_model, self.source_data = self.load_robot_model(source_urdf)
        self.target_model, self.target_data = self.load_robot_model(target_urdf)

        # Get joint IDs for mapping
        self.source_joint_ids = [
            self.source_model.getJointId(name)
            for name in self.config.joint_mapping.keys()
        ]
        self.target_joint_ids = [
            self.target_model.getJointId(name)
            for name in self.config.joint_mapping.values()
        ]

    @staticmethod
    def load_robot_model(urdf_path: str) -> Tuple[pin.Model, pin.Data]:
        """Load robot model from URDF"""
        robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            os.path.dirname(urdf_path),
            pin.JointModelFreeFlyer(),
        )
        return robot.model, robot.data

    @staticmethod
    def load_motion_data(file_name):
        motion = np.loadtxt(file_name, delimiter=",")
        return torch.from_numpy(motion).float()

    @staticmethod
    def save_motion_data(optimized_angles, output_file, joint_names=None):
        """Save optimized angles as CSV without header"""
        if joint_names is not None:
            header = ",".join(joint_names)
            # Save raw data
            np.savetxt(
                output_file,
                optimized_angles,
                fmt="%.6f",  # 6 decimal precision
                delimiter=",",
                comments="",  # No comments/header
                header=header,
            )
        else:
            np.savetxt(
                output_file,
                optimized_angles,
                fmt="%.6f",  # 6 decimal precision
                delimiter=",",
                comments="",  # No comments/header
            )

    @staticmethod
    def clamp_to_joint_limits(model, q):
        """
        Clamp configuration to joint limits
        Args:
            model: Pinocchio model
            q: Configuration to clamp
        Returns:
            Clamped configuration
        """
        # Get joint limits (excluding freeflyer)
        upper_limits = model.upperPositionLimit[7:]
        lower_limits = model.lowerPositionLimit[7:]

        # Keep freeflyer as is
        q_clamped = q.copy()

        # Clamp joint angles to limits
        q_clamped[7:] = np.clip(q[7:], lower_limits, upper_limits)

        return q_clamped

    def retarget_motion(
        self,
        motion_data: torch.Tensor,
        max_iterations: int = 100,
        damping: float = 1e-12,
        eps: float = 1e-6,
        dt: float = 1e-1,
    ) -> np.ndarray:
        """Retarget motion from source to target robot"""
        num_frames = len(motion_data)
        optimized_angles = np.zeros((num_frames, self.target_model.nq))

        # Initialize first frame
        q_prev = pin.neutral(self.target_model)
        q_prev[:7] = motion_data[0][:7].numpy()
        q_prev[2] -= self.config.height_offset
        q_prev[7:] = np.random.uniform(-1, 1, self.target_model.nq - 7)

        with tqdm(
            range(num_frames),
            desc="Retargeting frames",
            position=1,
            leave=False,
        ) as pbar:
            for frame in pbar:
                # Get source poses relative to root
                pin.forwardKinematics(
                    self.source_model,
                    self.source_data,
                    motion_data[frame].numpy(),
                )
                pin.updateFramePlacements(self.source_model, self.source_data)
                root_pose_source = self.source_data.oMi[1]  # Root joint pose
                target_poses = [
                    root_pose_source.inverse() * self.source_data.oMi[joint_id]
                    for joint_id in self.source_joint_ids
                ]

                # Initialize from previous frame
                q = q_prev.copy()
                q[:7] = motion_data[frame][:7].numpy()
                q[2] -= self.config.height_offset

                # IK optimization loop
                for _ in range(max_iterations):
                    pin.forwardKinematics(
                        self.target_model, self.target_data, q
                    )
                    pin.updateFramePlacements(
                        self.target_model, self.target_data
                    )

                    # Get target poses relative to root
                    root_pose_target = self.target_data.oMi[1]
                    current_poses = [
                        root_pose_target.inverse()
                        * self.target_data.oMi[joint_id]
                        for joint_id in self.target_joint_ids
                    ]

                    # Compute pose errors
                    errors = []
                    for target, current in zip(target_poses, current_poses):
                        err_se3 = pin.log6(current.inverse() * target)
                        errors.append(err_se3)

                    errors_flat = np.concatenate(errors)
                    if np.linalg.norm(errors_flat) < eps:
                        break

                    # Get full Jacobian (6 DOF per joint)
                    J = pin.computeJointJacobians(
                        self.target_model, self.target_data, q
                    )
                    J = np.vstack(
                        [
                            pin.getJointJacobian(
                                self.target_model,
                                self.target_data,
                                joint_id,
                                pin.ReferenceFrame.LOCAL,
                            )
                            for joint_id in self.target_joint_ids
                        ]
                    )

                    # Damped least squares
                    JJt = J @ J.T
                    lambda2 = damping * np.eye(JJt.shape[0])
                    v = np.linalg.solve(JJt + lambda2, errors_flat)
                    dq = J.T @ v

                    # Update configuration
                    q = pin.integrate(self.target_model, q, dt * dq)
                    q = self.clamp_to_joint_limits(self.target_model, q)
                    q[:7] = motion_data[frame][:7].numpy()
                    q[2] -= self.config.height_offset

                # Store optimized angles for this frame
                optimized_angles[frame] = q
                q_prev = q.copy()
                pbar.set_postfix(frame=f"{frame}/{num_frames}")

        return optimized_angles


def main():
    parser = argparse.ArgumentParser(
        description="Motion retargeting between robots"
    )
    parser.add_argument("files", nargs="+", help="Motion files to process")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Path to joint mapping YAML config",
    )
    parser.add_argument(
        "--source-urdf",
        type=str,
        required=True,
        help="Path to source robot URDF",
    )
    parser.add_argument(
        "--target-urdf",
        type=str,
        required=True,
        help="Path to target robot URDF",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="retargeted_motions",
        help="Output directory for retargeted motions",
    )
    args = parser.parse_args()

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    # Initialize retargeting
    config = RetargetingConfig(args.config)
    retargeting = MotionRetargeting(config, args.source_urdf, args.target_urdf)

    # Process files
    files = []
    for pattern in args.files:
        files.extend(glob.glob(pattern))

    with tqdm(files, desc="Processing files", position=0) as pbar:
        for file_path in pbar:
            try:
                motion_data = retargeting.load_motion_data(file_path)
                retargeted_motion = retargeting.retarget_motion(motion_data)

                output_file = os.path.join(
                    args.output_dir,
                    f"{os.path.basename(file_path)}",
                )
                retargeting.save_motion_data(retargeted_motion, output_file)

            except Exception as e:
                print(f"Error processing {file_path}: {e}")


if __name__ == "__main__":
    main()

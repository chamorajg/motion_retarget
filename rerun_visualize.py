import argparse
import os
import time
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import pinocchio as pin
import rerun as rr
import trimesh
import yaml


@dataclass
class RobotConfig:
    """Robot configuration data class"""

    urdf_path: str
    urdf_root: str
    base_height: float
    scale: float
    t_pose: np.ndarray


class RerunURDF:
    def __init__(self, config: RobotConfig, name: str):
        """Initialize robot visualization

        Args:
            config: Robot configuration
            name: Name identifier for the robot
        """
        self.name = name
        self.scale = config.scale

        # Load robot model
        self.robot = pin.RobotWrapper.BuildFromURDF(
            config.urdf_path, config.urdf_root, pin.JointModelFreeFlyer()
        )

        # Set T-pose configuration
        self.Tpose = np.array(config.t_pose).astype(np.float32)

        # Initialize visualization
        self.link2mesh = self.get_link2mesh()
        self.load_visual_mesh()
        self.update()

    def get_link2mesh(self):
        link2mesh = {}
        for visual in self.robot.visual_model.geometryObjects:
            mesh = trimesh.load_mesh(visual.meshPath)
            name = visual.name[:-2]
            mesh.visual = trimesh.visual.ColorVisuals()
            mesh.visual.vertex_colors = visual.meshColor
            link2mesh[name] = mesh
        return link2mesh

    def load_visual_mesh(self):
        self.robot.framesForwardKinematics(pin.neutral(self.robot.model))
        for visual in self.robot.visual_model.geometryObjects:
            frame_name = visual.name[:-2]
            mesh = self.link2mesh[frame_name]

            frame_id = self.robot.model.getFrameId(frame_name)
            parent_joint_id = self.robot.model.frames[frame_id].parentJoint
            parent_joint_name = self.robot.model.names[parent_joint_id]
            frame_tf = self.robot.data.oMf[frame_id]
            joint_tf = self.robot.data.oMi[parent_joint_id]
            rr.log(
                f"urdf_{self.name}/{parent_joint_name}",
                rr.Transform3D(
                    translation=joint_tf.translation * self.scale,
                    mat3x3=joint_tf.rotation,
                    axis_length=0.01 * self.scale,
                ),
            )

            relative_tf = frame_tf * joint_tf.inverse()
            mesh.apply_transform(relative_tf.homogeneous)
            rr.log(
                f"urdf_{self.name}/{parent_joint_name}/{frame_name}",
                rr.Mesh3D(
                    vertex_positions=mesh.vertices,
                    triangle_indices=mesh.faces,
                    vertex_normals=mesh.vertex_normals,
                    vertex_colors=mesh.visual.vertex_colors,
                    albedo_texture=None,
                    vertex_texcoords=None,
                ),
                timeless=True,
            )

    def update(self, configuration=None):
        self.robot.framesForwardKinematics(
            self.Tpose if configuration is None else configuration
        )
        for visual in self.robot.visual_model.geometryObjects:
            frame_name = visual.name[:-2]
            frame_id = self.robot.model.getFrameId(frame_name)
            parent_joint_id = self.robot.model.frames[frame_id].parentJoint
            parent_joint_name = self.robot.model.names[parent_joint_id]
            joint_tf = self.robot.data.oMi[parent_joint_id]
            rr.log(
                f"urdf_{self.name}/{parent_joint_name}",
                rr.Transform3D(
                    translation=joint_tf.translation * self.scale,
                    mat3x3=joint_tf.rotation,
                    axis_length=0.01 * self.scale,
                ),
            )


def load_robot_config(robot_type: str) -> RobotConfig:
    """Load robot configuration from YAML file"""
    config_path = os.path.join("robot_configs", f"{robot_type}.yaml")
    if not os.path.exists(config_path):
        raise ValueError(f"No configuration found for robot type: {robot_type}")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    return RobotConfig(**config)


def visualize_motion(
    motion_file: str,
    robot_type: str,
    frame_delay: float = 0.03,
    window_title: str = "Motion Visualization",
    coordinate_frame: str = "RIGHT_HAND_Z_UP",
) -> None:
    """Visualize motion data for a specific robot

    Args:
        motion_file: Path to motion CSV file
        robot_type: Type of robot to visualize
        frame_delay: Delay between frames in seconds
        window_title: Title for visualization window
        coordinate_frame: Coordinate frame convention
    """
    # Initialize rerun
    rr.init(window_title, spawn=True)
    rr.log("", getattr(rr.ViewCoordinates, coordinate_frame), timeless=True)
    rr.set_time_sequence("frame_nr", 0)

    # Load robot configuration and motion data
    config = load_robot_config(robot_type)
    data = np.genfromtxt(motion_file, delimiter=",")

    # Create visualization
    rerun_urdf = RerunURDF(config, robot_type)

    # Animate frames
    for frame_nr in range(data.shape[0]):
        rr.set_time_sequence("frame_nr", frame_nr)
        configuration = data[frame_nr, :]
        rerun_urdf.update(configuration)

        if frame_delay > 0:
            time.sleep(frame_delay)


def main():
    parser = argparse.ArgumentParser(description="Visualize robot motion data")
    parser.add_argument(
        "--motion-file",
        "-m",
        type=str,
        required=True,
        help="Path to motion CSV file",
    )
    parser.add_argument(
        "--robot-type",
        "-r",
        type=str,
        required=True,
        help="Robot type (must have corresponding config in robot_configs/)",
    )
    parser.add_argument(
        "--frame-delay",
        "-d",
        type=float,
        default=0.0,
        help="Delay between frames in seconds",
    )
    parser.add_argument(
        "--window-title",
        "-t",
        type=str,
        default="Motion Visualization",
        help="Window title for visualization",
    )
    parser.add_argument(
        "--coordinate-frame",
        "-c",
        type=str,
        default="RIGHT_HAND_Z_UP",
        choices=[
            "RIGHT_HAND_Y_UP",
            "RIGHT_HAND_Z_UP",
            "LEFT_HAND_Y_UP",
            "LEFT_HAND_Z_UP",
        ],
        help="Coordinate frame convention",
    )

    args = parser.parse_args()

    try:
        visualize_motion(
            args.motion_file,
            args.robot_type,
            args.frame_delay,
            args.window_title,
            args.coordinate_frame,
        )
    except Exception as e:
        print(f"Error during visualization: {str(e)}")


if __name__ == "__main__":
    main()

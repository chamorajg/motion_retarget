import argparse
import os
from typing import List, Optional, Tuple

import pinocchio as pin


def get_joint_names(urdf_path: str, include_limits: bool = True) -> List[Tuple]:
    """Extract joint names and properties from URDF file.

    Args:
        urdf_path: Path to the URDF file
        include_limits: Whether to include joint limits in output

    Returns:
        List of tuples containing joint information
    """
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    # Load robot model
    robot = pin.RobotWrapper.BuildFromURDF(
        urdf_path, os.path.dirname(urdf_path), pin.JointModelFreeFlyer()
    )

    # Get joint names (excluding universe and freeflyer joints)
    joint_names = []
    for joint_id in range(2, robot.model.njoints):
        joint = robot.model.joints[joint_id]
        if joint.nv > 0:  # Only get actuated joints
            joint_placement = robot.model.jointPlacements[joint_id]
            joint_axis = joint_placement.rotation[:, 2]

            joint_info = [
                robot.model.names[joint_id],
                joint.shortname(),
                joint_axis,
            ]

            if include_limits:
                joint_info.extend(
                    [
                        robot.model.upperPositionLimit[joint_id - 1],
                        robot.model.lowerPositionLimit[joint_id - 1],
                    ]
                )

            joint_names.append(tuple(joint_info))

    return joint_names


def print_joint_info(
    joint_names: List[Tuple], robot_name: str, verbose: bool = False
) -> None:
    """Print joint information in a formatted way.

    Args:
        joint_names: List of joint information tuples
        robot_name: Name of the robot for display
        verbose: Whether to print detailed information including axis and limits
    """
    print(f"\n{robot_name} Joints: {len(joint_names)}")
    for joint_info in joint_names:
        if verbose:
            name, type_, axis = joint_info[:3]
            limits = joint_info[3:] if len(joint_info) > 3 else None
            print(f"Joint: {name:<30} Type: {type_:<15} Axis: {axis}")
            if limits:
                print(
                    f"    Limits: Upper={limits[0]:.2f}, Lower={limits[1]:.2f}"
                )
        else:
            name, type_ = joint_info[:2]
            print(f"Joint: {name:<30} Type: {type_:<15}")


def main():
    parser = argparse.ArgumentParser(
        description="Extract joint information from URDF files"
    )
    parser.add_argument(
        "--urdf", "-u", type=str, nargs="+", help="Path(s) to URDF file(s)"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Print detailed joint information",
    )
    parser.add_argument(
        "--no-limits",
        action="store_true",
        help="Exclude joint limits from output",
    )

    args = parser.parse_args()

    # Default URDFs if none provided
    default_urdfs = {
        "G1": "robot_description/g1/g1_29dof_rev_1_0.urdf",
        "H1": "robot_description/h1/h1.urdf",
        "H1-2": "robot_description/h1_2/h1_2_wo_hand.urdf",
        "Stompy": "robot_description/stompypro/robot_test.urdf",
        "GPR": "robot_description/gpr/robot.urdf",
    }

    urdf_files = args.urdf if args.urdf else default_urdfs.items()

    for name, urdf_path in (
        urdf_files
        if isinstance(urdf_files, dict)
        else [(f"Robot_{i}", p) for i, p in enumerate(urdf_files)]
    ):
        try:
            joint_names = get_joint_names(urdf_path, not args.no_limits)
            print_joint_info(joint_names, name, args.verbose)
        except Exception as e:
            print(f"Error processing {urdf_path}: {str(e)}")


if __name__ == "__main__":
    main()

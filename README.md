### Documentations

[dataset_README.md](docs/dataset_README.md) - Information about the dataset
[dataset_LICENSE](docs/LICENSE) - Information about the dataset LICENSE


### Usage

1. Extracting joint names from URDF files
```bash
# Default behavior with built-in URDFs
python extract_joint_names.py
```
```bash
# Custom URDF with verbose output
python extract_joint_names.py -u path/to/urdf/file.urdf -v
```
```bash
# Multiple URDFs without joint limits
python extract_joint_names.py -u robot1.urdf robot2.urdf --no-limits
```


2. Retargeting motion files.
```bash
python retarget_motion.py \
    --config joint_mappings/g1_stompypro.yaml \
    --source-urdf robot_description/g1/g1_29dof_rev_1_0.urdf \
    --target-urdf robot_description/stompypro/robot_test.urdf \
    --output-dir retargeted_motions \
    "motion_files/*.csv"
```


3. Visualizing retargeted motion files.
```bash
# Basic usage
python rerun_visualize.py -m motions/dance1.csv -r g1
```

```bash
# Custom visualization settings
python rerun_visualize.py \
    --motion-file motions/dance1.csv \
    --robot-type stompypro \
    --frame-delay 0.05 \
    --window-title "Stompy Dance" \
    --coordinate-frame LEFT_HAND_Z_UP
```
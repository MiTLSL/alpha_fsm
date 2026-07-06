from __future__ import annotations


def load_sim_parameters(node) -> None:
    from pathlib import Path

    from ament_index_python.packages import get_package_share_directory
    from fsm_core.ros2_helpers import declare_parameters_from_dict, load_yaml

    config_dir = Path(get_package_share_directory("fsm_config")) / "params"
    node.config.update(declare_parameters_from_dict(node, load_yaml(config_dir / "sim.yaml")))

import os
from typing import Any, Dict


def get_project_root() -> str:
    """Get the absolute path to the project root directory."""
    # This file is in spot/utils/, so go up 2 levels to get to project root
    current_file = os.path.abspath(__file__)
    utils_dir = os.path.dirname(current_file)
    spot_dir = os.path.dirname(utils_dir)
    project_root = os.path.dirname(spot_dir)
    return project_root


def resolve_config_paths(cfg: Any) -> Any:
    """
    Resolve relative paths in a configuration object to absolute paths.
    
    This function looks for common path attributes and converts relative paths
    to absolute paths relative to the project root.
    
    Args:
        cfg: Configuration object (typically from Hydra)
        
    Returns:
        Modified configuration object with resolved paths
    """
    project_root = get_project_root()
    
    # Common path attributes to resolve
    path_attrs = [
        'dataset_root', 'test_dataset_root', 'log_dir', 'checkpoint_dir',
        'weights_folder', 'rpdiff_descriptions_path', 'demo_folder'
    ]
    
    for attr in path_attrs:
        if hasattr(cfg, attr) and getattr(cfg, attr):
            path_value = getattr(cfg, attr)
            # Only resolve if it's a relative path
            if path_value and not os.path.isabs(path_value):
                resolved_path = os.path.join(project_root, path_value)
                setattr(cfg, attr, resolved_path)
    
    return cfg


def resolve_path(path: str) -> str:
    """
    Resolve a single relative path to an absolute path.
    
    Args:
        path: Path string (can be relative or absolute)
        
    Returns:
        Absolute path
    """
    if os.path.isabs(path):
        return path
    
    project_root = get_project_root()
    return os.path.join(project_root, path)

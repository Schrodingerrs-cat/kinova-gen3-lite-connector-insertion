import os

import matplotlib
import mpl_toolkits

# pip matplotlib shadows the system one but the system mpl_toolkits (older) wins: point it at the pip copy before anything imports pyplot, else '3d' is not registered
mpl_toolkits.__path__.insert(0, os.path.join(os.path.dirname(os.path.dirname(matplotlib.__file__)), "mpl_toolkits"))

from .cloud import cloud_from_depth
from .gripper import grasp_held

__all__ = ["cloud_from_depth", "grasp_held"]

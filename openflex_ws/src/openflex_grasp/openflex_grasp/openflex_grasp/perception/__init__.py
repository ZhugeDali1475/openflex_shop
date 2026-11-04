"""Pure perception components used by the ROS and offline entry points.

The package initializer intentionally avoids importing GPU runtimes. Callers import
the required submodule so RGB-D, point-cloud and cache tests stay independent of torch.
"""

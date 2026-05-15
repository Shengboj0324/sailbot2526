"""Path planning package compatibility helpers."""
import sys

# The ROS package setup installs this code as ``path_planning.path_planning``,
# while local source-checkout tests often import it as plain ``path_planning``.
# Register both names so control modules and tests can use either style.
sys.modules.setdefault("path_planning.path_planning", sys.modules[__name__])

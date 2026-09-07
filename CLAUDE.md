## Coding Principles

Single Responsibility Principle (SRP) - Each class or function should have a single responsiblity, which leads
to better reusability, reduced complexity, and easier maintenance.

Don't Repeat Yourself (DRY) - No copy-pasted code; As bugs found in the copy-pasted code is very difficult
to ensure are fixed in all locations. Constructing shared helpers to share functinoality or updating previous
code to support both (for example, upgrading a conversion fucntion to support a new conversion) allows us to
fulfill the DRY principle and also fulfills the SRP princple as the function still has one responsibility (image
Conversion).

## Coding style

Python:
- Google-style docstrings. One line for simple functions; a full docstring with `Args`/`Returns`
  for larger ones. At most one paragraph of prose in a docstring (not counting `Args`/`Returns`
  themselves) except in genuinely unusual cases.
- Type hints on every parameter, return value, and non-obvious variable.
- No global variables anywhere in this library — they should be part of a class instead.
- Inline comments are one line ~95% of the time; two-plus lines only in extreme cases.
- Imports listed alphabetically (e.g. `import numpy`; `import robotdataprocess`). This means one
  flat list sorted case-insensitively by module path — not grouped into stdlib/third-party/local
  blocks (no isort-style grouping). A relative import (`from .foo import bar`) sorts by its bare
  module name (`foo`), ignoring the leading dot. Example:
  ```python
  from .air_museum import load_AirMuseum_data
  from MeronomyGraph.params.data_params import DataParams
  from pathlib import Path
  from robotdatapy.data.pose_data import PoseData
  from typing import Callable
  ```
- Type hints on non-obvious variables only apply to a line that also does something (an actual
  assignment or expression) — never add a bare `name: Type` declaration line with nothing else on
  it just to satisfy this rule. If a variable is assigned conditionally across branches, annotate
  the first assignment (or skip the annotation if it isn't genuinely needed for clarity).
- No file-level top-of-file comment block — put that content in the relevant class/function
  docstring instead. This applies to CLI-launcher-style files too: give them a `__main__` guard and
  put the file's explanation in its docstring, not a header comment.
- Every class attribute that gets set anywhere in the class must be declared up front in the class
  body with a type annotation — even though Python allows adding attributes on the fly elsewhere,
  don't rely on that; it makes classes hard to reason about at a glance. If a new value is set on
  the class in any of its functions, it needs to be added here too. Ideally, every attribute is
  initialized (or set to `None`) in `__init__`. Example:

  ```python
  @typechecked
  class OdometryData(PathData):
      """
      Odometry data extending PathData with a child frame ID and ROS message caching.
      Supports loading from ROS2 bags, CSV files, and TXT files, and exporting
      to CSV or ROS messages (Odometry, Path, and maplab OdometryWithImuBiases).

      Attributes:
          child_frame_id: The frame whose pose is described by this odometry (e.g. ``"base_link"``).
          poses: Cached rosbags PoseStamped messages (rebuilt after any mutation).
          poses_rclpy: Cached rclpy/rospy PoseStamped messages (rebuilt after any mutation).
      """

      # Define odometry-specific data attributes
      child_frame_id: str
      poses: list  # Saved nav_msgs/msg/Pose for rosbags
      poses_rclpy: list  # Saved nav_msgs/msg/Pose for rclpy

      def __init__(self, frame_id: str, child_frame_id: str, timestamps: Union[np.ndarray, list],
                   positions: Union[np.ndarray, list], orientations: Union[np.ndarray, list], frame: CoordinateFrame):

          # Copy initial values into attributes
          super().__init__(frame_id, timestamps, positions, orientations, frame)
          self.child_frame_id: str = child_frame_id
          self.poses: list = []
          self.poses_rclpy: list = []

          # Check to ensure that all arrays have same length
          if len(self.timestamps) != len(self.positions) or len(self.positions) != len(self.orientations):
              raise ValueError("Lengths of timestamp, position, and orientation arrays are not equal!")
  ```

- Avoid one-liner functions — if a function's
  entire body is one line, the call site could just use that line directly instead, which makes it
  harder to see what's actually happening. A function earns its name by representing a genuinely
  complicated low-level implementation at a high-level of understanding; a one-liner doesn't do
  that. Don't write new functions like this, and remove existing ones from the library where found.

# ur2_description
Simple UR-2 arm package

This package containts a problem for motion planning: a 5-DoF double-arm robot (4 rotations and 1 translation) 
that should move around one or two cylinder(s). One of the arm can be easily removed by the user commenting 
lines in the URDF file.

The package contains:

  - URDF/SRDF/class files describing the objects,

  - Some Python scripts going along with HPP software (github.com/humanoid-path-planner) for motion planning,

The problem can be vizualised with HPP-gepetto-viewer (github.com/humanoid-path-planner) or with RViz (must create .launch files).

To install the package with cmake, simply:

  - Create a 'build' directory in the source package,

  - in the created /build, configure the package - particularly the 'install path variable' - and install it with 'ccmake ..' and 'make install'.


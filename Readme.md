ik.hpp
================


`ik.hpp` is an minimal Inverse Kinematics Library without any dependency except C++ STL.

## Features

1. FABRIK Inverse Kinematics for 2D/3D bone chains
2. Joint angle constraints
3. Force-based soft IK (`step`) and influence blending
4. Chain-scoped solve (`solveChain` / `stepChain`) that pins an arbitrary chain root
5. 3D pole vector (magnet / hint) for bend direction
6. Tip orientation override after solving


## Compare

https://github.com/stack-of-tasks/pinocchio

动力学库(我知道的)有RBDL、KDL、Pinocchio，Frost，Drake等， Ikfast,Trac-ik
# Mediapipe to MANO Mesh

From mediapipe hand joint, via inverse kinematics, create MANO hand mesh. Visualize the mesh with Open3D.

![demo](./demo.gif)

\* Hands are flipped as the camera is mirrored. This is supposed to work with a right hand without a camera flip.

## Installation

If you don't have python3.8,

```
brew install python@3.8
```

```
python3.8 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

```

\* If you are in MacOS M processors, you may need python3.8 or lower due to the issue in Open3D. It may require open3d version 0.14.1 and this is available python3.8 or lower.

## Usage

```
python main.py
```

## Credit

[CalciferZh/Minimal-IK](https://github.com/CalciferZh/Minimal-IK)

[SMPL MANO](https://mano.is.tue.mpg.de/)

[Mediapipe](https://pypi.org/project/mediapipe/)

# Mediapipe to MANO Mesh

From mediapipe hand joint, via inverse kinematics, create MANO hand mesh. Visualize the mesh with Open3D.

![demo](./demo.gif)

\* Hands are flipped as the camera is mirrored. This is supposed to work with a right hand without a camera flip.

## Installation

```
python3.8 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

```

\* Due to the issue in Open3D, you may need python3.8 or lower.

## Usage

```
python main.py
```

## Credit

[CalciferZh/Minimal-IK](https://github.com/CalciferZh/Minimal-IK)

[SMPL MANO](https://mano.is.tue.mpg.de/)

[Mediapipe](https://pypi.org/project/mediapipe/)

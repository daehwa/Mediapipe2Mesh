# Mediapipe to MANO Mesh

From mediapipe hand joint, via inverse kinematics, create MANO hand mesh. Visualize the mesh with Open3D.

![demo](./demo.gif)
(Hands are flipped as camera is mirrored. This supposed to work with right hand without camera flip.)

## Installation

```
python3.8 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

```

## Usage

```
python main.py
```

## Credit

[CalciferZh/Minimal-IK](https://github.com/CalciferZh/Minimal-IK)

[SMPL MANO](https://mano.is.tue.mpg.de/)

[Mediapipe](https://pypi.org/project/mediapipe/)

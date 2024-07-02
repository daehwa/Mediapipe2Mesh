import cv2
import numpy as np

## ------------------- ##
## Inverse Kinematics  ##
## ------------------- ##
from keypoint2mano import Keypoints2Mano
from inverse_kinematics.armatures import *
from inverse_kinematics.models import *
k2m = Keypoints2Mano(model_path='./MANO_RIGHT.npz')

## ------------------- ##
##      Visualizer     ##
## ------------------- ##
import open3d as o3d

mesh = o3d.geometry.TriangleMesh()
verts = k2m.get_vertices()
faces = k2m.get_faces()
mesh.vertices = o3d.utility.Vector3dVector(verts)
mesh.triangles = o3d.utility.Vector3iVector(faces)
mesh.compute_vertex_normals()
mesh.paint_uniform_color([0.9, 0.8, 0.7])

vis = o3d.visualization.Visualizer()
vis.create_window(window_name='EtherPose Viewer')
vis.add_geometry(mesh)

## ------------------- ##
##      Mediapipe      ##
## ------------------- ##
import mediapipe as mp

# Initialize MediaPipe hands module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, 
                        model_complexity=1,
                        max_num_hands=1,
                        min_detection_confidence=0.5,
                        min_tracking_confidence=0.5)

# Initialize MediaPipe drawing module
mp_drawing = mp.solutions.drawing_utils

# Open video stream (0 for the default camera, or replace with video file path)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to RGB
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the frame to find hand landmarks
    results = hands.process(image_rgb)

    # Draw the hand annotations on the frame
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_world_landmarks:
            hand_landmarks_array = np.array([[lmk.x, lmk.y, lmk.z] for lmk in hand_landmarks.landmark])
            pose_hand = k2m.get_mano_params(hand_landmarks_array)
            verts = k2m.get_vertices()
            faces = k2m.get_faces()

            mesh.vertices = o3d.utility.Vector3dVector(verts)
            mesh.triangles = o3d.utility.Vector3iVector(faces)
            mesh.compute_vertex_normals()
            vis.update_geometry(mesh)

        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    vis.poll_events()
    vis.update_renderer()

    # Display the frame
    cv2.imshow('MediaPipe Hands', frame)

    if cv2.waitKey(5) == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
hands.close()

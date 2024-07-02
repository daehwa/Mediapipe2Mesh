import sys
import numpy as np
import argparse
from inverse_kinematics.solver import *
from inverse_kinematics.armatures import *
from inverse_kinematics.models import *
import inverse_kinematics.config
from scipy.spatial.transform import Rotation as R
import glob

n_pose = 17

class Keypoints2Mano():
    def __init__(self,model_path='./MANO_RIGHT.npz'):
        mesh = KinematicModel(model_path, MANOArmature, scale=1000)
        self.wrapper = KinematicPCAWrapper(mesh, n_pose=n_pose)
        self.solver = Solver(max_iter=3,verbose=True)
        self.mesh = mesh

    def get_mano_params(self, keypoints):
        keypoints = self.to_wrist_coordinate(keypoints)
        keypoints = self.mediapipe2mano_joints(keypoints)
        keypoints = self.set_default_rotation(keypoints)
        params_est = self.solver.solve(self.wrapper, keypoints, u=35, v=10)
        shape_est, pose_pca_est, pose_glb_est = self.wrapper.decode(params_est)
        self.set_mesh_params(pose_pca_est,
                             np.zeros(pose_glb_est.shape),
                             np.zeros(shape_est.shape))
        return pose_pca_est

    def to_wrist_coordinate(self,hand_data):
        def normalize(x):
            res = x / np.linalg.norm(x)
            return res
        v05 = hand_data[5]-hand_data[0]
        v017 = hand_data[17]-hand_data[0]
        normal = normalize(np.cross(v017,v05))
        x_axis = normalize(v05)
        y_axis = normalize(normal)
        z_axis = normalize(np.cross(x_axis,y_axis))
        for i in range(len(hand_data)):
            v = np.copy(hand_data[i])
            hand_data[i, 0] = np.dot(v,x_axis)
            hand_data[i, 1] = np.dot(v,y_axis)
            hand_data[i, 2] = np.dot(v,z_axis)
        hand_data[:, 2] *= -1
        return hand_data

    def set_mesh_params(self,pose_pca,pose_glb,shape):
        self.mesh.set_params(pose_pca=pose_pca, pose_glb=pose_glb, shape=shape)

    def get_vertices(self):
        return self.mesh.verts

    def get_faces(self):
        return self.mesh.faces

    def mediapipe2mano_joints(self,keypoints):
        mapping = [0,13,14,15,20,1,2,3,16,4,5,6,17,10,11,12,19,7,8,9,18]
        mano_keypoints = np.zeros((21,3))
        for i, m in enumerate(mapping):
            mano_keypoints[m] = keypoints[i]
        return mano_keypoints

    def set_default_rotation(self,keypoints):
        keypoints = keypoints*1.85*500
        r_mat = R.from_euler('xyz', [180,190,0], degrees=True).as_matrix()
        origin = np.array([95.66993092,6.38342886,6.18630528])
        for i in range(keypoints.shape[0]):
            k = np.array([keypoints[i]]).T
            keypoints[i] = np.matmul(r_mat,k).T[0]
        keypoints = keypoints - (keypoints[0]-origin)
        return keypoints
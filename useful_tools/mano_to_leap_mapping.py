import numpy as np
from typing import Optional, Union
import torch

from manotorch.anchorlayer import AnchorLayer
from manotorch.axislayer import AxisLayerFK
from manotorch.manolayer import ManoLayer, MANOOutput

'''
This converts MANO to LEAP Hand by simple direct joint-to-joint angle mapping.

This is a simple retargeting scheme, I HIGHLY recommend you tune/visualize the outputs to make it useful for your particular purpose and tune the offsets/scaling a bit.

This is based on this repository, see here for more details:
https://github.com/lixiny/manotorch
Follow the instructions from that repo to install manotorch.
This is different to using mano params directly, which is the rotation of fingers w/ respect to the previous joint, including more than just joint angles but also how the joints are aligned with respect to the wrist.

In our case, we care about ee.  ee is the Euler angles of the rotation of the joints in the human hand, anatomically aligned.
The order of ee is Twist(abnormal for most humans), Spread (side to side/abduction adduction), Bend (normal for all joints).   And the joint order is MCP, PIP, DIP from Index, Middle, Ring, Pinky and then Thumb.
T_g_a is the joint positions of the anatomically correct hand....we don't use the positions but the joint angles directly.
'''

class ManoRetargeter:
    def __init__(
        self,
        device: Optional[Union[str, torch.device]] = None):
        self.mano_layer = ManoLayer(
            rot_mode="axisang",
            use_pca=False,
            side="right",
            center_idx=None,
            mano_assets_root="/home/kshaw/manotorch/assets/mano",  #change this to your user
            flat_hand_mean=False,
        )
        self.axis_layer = AxisLayerFK(mano_assets_root="/home/kshaw/manotorch/assets/mano") #change this to your user         
    def mano_retarget(self, joint_pose, shape_params):
        mano_results: MANOOutput = self.mano_layer(joint_pose, shape_params)
        T_g_p = mano_results.transforms_abs  # (B, 16, 4, 4)
        T_g_a, R, ee = self.axis_layer(T_g_p)  # ee (B, 16, 3)  this is anatomy aligned euler angle
        ee = ee.flatten().tolist()
        output = self._get_poses(ee)
        return output
    #  transform order of right hand
        #         15-14-13-\
        #                   \
        #    3-- 2 -- 1 -----0   
        #   6 -- 5 -- 4 ----/
        #   12 - 11 - 10 --/
        #    9-- 8 -- 7 --/
        '''
        This takes the euler angles from the axis_layer outputs and maps them directly to the 16 joint angles in the LEAP Hand by extracting the right component of the euler.
        '''
    def _get_poses(self, finger_joints):
        finger_joints = np.reshape(finger_joints, (16, 3)) 
 
        finger_mcp_id = [1, 4, 10]#,7]  ##this is not a bug, 7,8,9 is pinky which we ignore
        finger_pip_id = [2, 5, 11]#, 8]
        finger_dip_id = [3, 6, 12]#, 9]

        ee_mcps = finger_joints[finger_mcp_id]  # shape (4, 3)
        ee_pips = finger_joints[finger_pip_id]  # shape (4, 3)
        ee_dips = finger_joints[finger_dip_id]  # shape (4, 3)

        joint_mcp_side = -ee_mcps[:,1] #shape (4)
        joint_mcp_forward = ee_mcps[:,2] #shape (4)
        joint_pip = ee_pips[:,2] #shape (4)
        joint_dip = ee_dips[:,2] #shape (4)

        thumb_cmc_side = finger_joints[13:14,1] #shape (1)
        thumb_cmc_forward = finger_joints[13:14,2] #shape (1)
        thumb_mcp = finger_joints[14:15,2]  #shape (1)
        thumb_ip = finger_joints[15:,2]  #shape (1)
        output = []
        for i in range(0,3):
            output +=  [joint_mcp_side[i],joint_mcp_forward[i], joint_pip[i], joint_dip[i]]
        output += [thumb_cmc_side[0], thumb_cmc_forward[0], thumb_mcp[0], thumb_ip[0]]
        return output

if __name__ == "__main__":
    mano_retargeter = ManoRetargeter()
    mano_grasp = np.zeros(58)  ##Replace this with the 58 dimensional MANO you want
    joint_pose = mano_grasp[0:48] ##First 48 is the joint params
    shape_params = mano_grasp[48:58] ##last 10 is the shape params
    #You can also do this in a batch, but I don't in this example.
    leap_hand_output = mano_retargeter.mano_retarget(joint_pose, shape_params)
    print(leap_hand_output)

import pybullet as p
import pybullet_data
import numpy as np

def view_pose(client, T):
    length = 0.1
    xaxis = np.array([length, 0, 0, 1])
    yaxis = np.array([0, length, 0, 1])
    zaxis = np.array([0, 0, length, 1])
    T_axis = np.array([xaxis, yaxis, zaxis]).T
    axes = T @ T_axis
    orig = T[:3,-1]
    xaxis = axes[:-1,0]
    yaxis = axes[:-1,1]
    zaxis = axes[:-1,2]
    x = client.addUserDebugLine(orig,xaxis, lineColorRGB=[1,0,0], lineWidth=5)
    y = client.addUserDebugLine(orig,yaxis, lineColorRGB=[0,1,0], lineWidth=5)
    z = client.addUserDebugLine(orig,zaxis, lineColorRGB=[0,0,1], lineWidth=5)
    pose_id = [x, y, z]
    return pose_id

def clear(client):
    for i in range(100):
        client.removeUserDebugItem(i)

class Panda:
    def __init__(self, client):
        """ Init panda class
        """
        self.client = client
        self.robot = self.client.loadURDF("./urdf/panda.urdf", useFixedBase=True)
        # Simulation Configuration
        pos, ori = [0, 0, 0], [0, 0, 0, 1]
        self.client.resetBasePositionAndOrientation(self.robot, pos, ori)
        #create a constraint to keep the fingers centered
        c = self.client.createConstraint(self.robot,
                        9,
                        self.robot,
                        10,
                        jointType=self.client.JOINT_GEAR,
                        jointAxis=[1, 0, 0],
                        parentFramePosition=[0, 0, 0],
                        childFramePosition=[0, 0, 0])
        self.client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

        # Set robot information
        self._all_joints = range(self.client.getNumJoints(self.robot))
        self._arm_joints = [i for i in range(7)]
        self._finger_joints = [9,10]
        self._joint_info = self.get_joint_info()
        self._movable_joints = self._arm_joints + self._finger_joints
        
    def get_joint_attribute_names(self):
        return ["joint_index","joint_name","joint_type",
             "q_index", "u_index", "flags", 
             "joint_damping", "joint_friction","joint_lower_limit",
             "joint_upper_limit","joint_max_force","joint_max_velocity",
             "link_name","joint_axis","parent_frame_pos","parent_frame_orn","parent_index"]

    def get_joint_info(self):
        result = {}
        attribute_names = self.get_joint_attribute_names()
        for i in self._all_joints:
            values = self.client.getJointInfo(self.robot, i)
            result[i] = {name:value for name, value in zip(attribute_names, values)}
        return result

    def get_states(self):
        result = {}
        state_names = ["position", "velocity", "wrench", "effort"]
        joint_states = self.client.getJointStates(self.robot, self._movable_joints)
        for i, name in enumerate(state_names):
            result[name] = [states[i] for states in joint_states]
        return result
    
    def get_link_pose(self, link_index):
        result = self.client.getLinkState(self.robot, link_index)
        pos, ori = np.array(result[0]), np.array(result[1])
        R = np.array(self.client.getMatrixFromQuaternion(ori)).reshape((3,3))
        return np.block([[R,pos[:,None]],[np.zeros(3),1]])

    def get_body_jacobian(self):
        states = self.get_states()
        n = len(self._movable_joints)
        trans, rot = self.client.calculateJacobian(bodyUniqueId=self.robot,
                                      linkIndex=11,
                                      localPosition=[0,0,0],
                                      objPositions=states["position"],
                                      objVelocities=np.zeros(n).tolist(),
                                      objAccelerations=np.zeros(n).tolist())
        return np.vstack([trans, rot])[:,:-2] #remove finger joint part

#    def inverse_kinematics(self, )

if __name__ == "__main__":
    uid = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # for loading plane
    
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    
    # load objects
    # load plane
    plane_id = p.loadURDF("plane.urdf")
    panda = Panda(p)
    a = panda.get_states()
    
    T = panda.get_link_pose(11)
    jac = panda.get_body_jacobian()
    print(3)
    view_pose(p, T)
    print("hey")

    while(1):
        pass
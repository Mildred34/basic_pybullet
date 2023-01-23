#!/usr/bin/env python3
import os,sys
import time
from ast import Dict, List
import gym
import numpy as np
import math
import pybullet as p
import pybullet_data # you can provide your own data or data package from pybullet
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
from generator.ressources.arm import Arm
from generator.ressources.plane import Plane
from generator.ressources.table import Table
from generator.ressources.objects.random_object import Goal
# from generator.grasps.quality import Contacts_Generator # HERE
import matplotlib.pyplot as plt
from typing import List, Tuple
from functions import DEBUG_JOINTS,DEBUG_POS, GUI, GRAVITY, ARM_MODEL, DEBUG_LEVEL, LOG_PATH, path_starting_from_code, QualityMode, axe
from logs import _log_handling as log

class GraspingGeneratorEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    logger = None

    def __init__(self):
        """
        Parameters : 
        For Gym : (A Definir WIP)
            Action Space :
                - Cartesian positions of the end-effector
                - Joint variable for every fingers

            Observation Space :
                - Joint value
                - Cartesian positions and Orientation of the end-effector
                - Object Cartesian position
                - Object Orientation

        - Client : id of the simulator client
        - Robot : id of the robot that will be summoned into the sim
        - gui : If you wanna see the simulation, or just a rendering at the end
        """

        if( GraspingGeneratorEnv.logger is None):
            self.init_log()
        
        self.action_space = gym.spaces.box.Box(
            low=np.array([-1]*4, dtype=np.float32),
            high=np.array([1]*4, dtype=np.float32))

        self.observation_space = gym.spaces.box.Box(
            low=np.array([-10, -10, -1, -1, -5, -5, -10, -10], dtype=np.float32),
            high=np.array([10, 10, 1, 1, 5, 5, 10, 10], dtype=np.float32))
        
        self.action = [0,0,0]

        # Debug parameters
        self.debug = {"joints":False,"pos": False}

        if(GUI()):
            GraspingGeneratorEnv.logger.debug("Graphique mode activated")
            self.gui = True
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI,1)
            p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0.4,0.4,0.7])
        else:
            self.gui = False
            self.client = p.connect(p.DIRECT)

        # engine Parameters
        p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
        p.setPhysicsEngineParameter(solverResidualThreshold=0.001,numSolverIterations=300)
        p.setPhysicsEngineParameter(collisionFilterMode = 1, enableConeFriction = 1, contactSlop = 0.2)
        

        # Reduce length of episodes for RL algorithms
        p.setTimeStep(1./240., self.client)
        # To see things clearly

        self.arm = None
        self.obj = None
        self.done = False
        self.rendered_img = None
        self.render_rot_matrix = None

        # Quality test mode
        self.axe = axe.X
        self.reset()

        if(self.gui):

            if(DEBUG_JOINTS()):
                GraspingGeneratorEnv.logger.debug("Joints mode activated")
                self.debug["joints"] = True
                self.angle = {}

                # Contrôle des articulations manuel en mode débug
                for joint_id in range(self.arm.numJoints):
                    if(p.getJointInfo(self.arm.robotId, joint_id)[2] != p.JOINT_FIXED):
                        joint_info = p.getJointInfo(self.arm.robotId, joint_id)
                        joint_type = joint_info[2]
                        joint_limits = {'lower': joint_info[8], 'upper': joint_info[9]}

                        if joint_type == p.JOINT_SPHERICAL:
                            self.angle[joint_id] =[p.addUserDebugParameter("%s pitch" % joint_info[1].decode('UTF-8'), joint_limits["lower"], joint_limits["upper"], 0),
                                                p.addUserDebugParameter("%s yaw" % joint_info[1].decode('UTF-8'), joint_limits["lower"], joint_limits["upper"], 0)] # No roll
                        else:
                            self.angle[joint_id] = p.addUserDebugParameter(p.getJointInfo(self.arm.robotId, joint_id)[1].decode('UTF-8'), joint_limits["lower"], joint_limits["upper"])

            elif(DEBUG_POS()):
                GraspingGeneratorEnv.logger.debug("Position mode activated")
                self.debug["pos"] = True
                self.action[0] = p.addUserDebugParameter('posX', -1, 1, 0.5)
                self.action[1] = p.addUserDebugParameter('posY', -1, 1, 0)
                self.action[2] = p.addUserDebugParameter('posZ', 0.5, 1.5, 1.0)


    def init_log(self):
         
        path_log = path_starting_from_code(0) + LOG_PATH()

        #Initialisation du mode debug ou release
        # Choix du mode DEBUG ou RELEASE
        if(DEBUG_LEVEL() == log.DEBUG_LEVEL.DEBUG_SOFT):
            #log.config["filename_debug"] = path_log + "debug"
            log.config["loggername"] = "env"
            GraspingGeneratorEnv.logger = log.factory.create('DEBUG',**log.config)
        else:
            #log.config["filename_release"] = path_log + "release"
            log.config["loggername"] = "env"
            GraspingGeneratorEnv.logger = log.factory.create('RELEASE',**log.config)

    def step(self, action : List = [0.6,0,0.8,0,0,0,1]) -> Tuple[np.ndarray,int,bool,Dict]:
        """
            Feed action to the arm and get observation of arm's state
            The action is a list composed of a position and an orientation
            (Orientation is optional)
            Orientation is in quaternion
        """

        user_param = {}
        user_config = []
        res = None

        # Récupération de l'orientation/position du robot
        robot_ob = self.arm.get_observation_quaternion(self.arm.endeffectorId)

        # Smooth simulation rendering to avoid jumping
        if(self.gui):
            p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)

        if(self.debug['joints']):
            for joint_id in range(self.arm.numJoints):

                joint_info = p.getJointInfo(self.arm.robotId, joint_id)
                joint_type = joint_info[2]

                if(joint_type != p.JOINT_FIXED):
                    if(joint_type != p.JOINT_SPHERICAL):
                        user_param["Joint_"+str(joint_id)] = p.readUserDebugParameter(self.angle[joint_id])
                    else:
                        angles_spherical = []

                        for id in self.angle[joint_id]:
                            angles_spherical.append(p.readUserDebugParameter(id))

                        user_param["Joint_"+str(joint_id)] = angles_spherical


            res = self.arm.apply_action(user_config,**user_param)

        elif(self.debug["pos"] and not self.debug['joints']):
            user_config.append(p.readUserDebugParameter(self.action[0]))
            user_config.append(p.readUserDebugParameter(self.action[1]))
            user_config.append(p.readUserDebugParameter(self.action[2]))
            res = self.arm.apply_action(user_config,**user_param)

            if(not res):
                GraspingGeneratorEnv.logger.warning("Inverse Kinematics not found")

        elif(not(self.debug["pos"] or self.debug["joints"])):
            # Récupération de l'orientation/position de l'objet
            # On attend que le bras se place bien avant de passer à l'étape suivante
            # self.Quality_test.etude()

            # Si on change d'axe d'étude, on fait un reset
            # Ou on change d'objet
            # if(Contacts_Generator.m_reset_request):
            #     self.reset()
            #     Contacts_Generator.m_reset_request = False
            pass

        # show axis for end-effector and object
        if(self.gui):
            self.arm.axiscreator(linkId = -1)
            self.obj.axiscreator()

            for finger in self.arm._fingers:
                finger.axiscreator()

        p.stepSimulation()

        ob = np.array(robot_ob, dtype=np.float32)

        return ob, 0, self.done, dict()

    def seed(self, seed : int = 0) -> List : 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def reset(self) -> np.ndarray:
        p.resetSimulation(self.client)

        if(GRAVITY()):
            p.setGravity(0, 0, -9.81)

        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything

        # Reload the plane, robot and table
        Plane(self.client)
        # Table(self.client)
        
        self.arm = Arm(self.client,ARM_MODEL())


        # Set the goal to a random target
        # WIP Générer une position aléatoire dans le domaine atteignable du bras
        x = 0.5 # 0.4
        y = 0.35 # 0.4
        z = 0.0
             
        self.goal = (x, y, z)
        self.done = False

        # Visual element of the goal
        if(self.obj is None):
            self.obj = Goal(self.client, self.goal, self.axe)
            GraspingGeneratorEnv.logger.info("Objet en cours d'étude:{}".format(self.obj._name))
        else:
            self.obj = Goal(self.client, self.goal, self.axe)


        GraspingGeneratorEnv.logger.info("Axe en cours d'étude:{}".format(self.axe))

        # Get observation to return
        rob_ob = self.arm.get_observation_quaternion(0) # ori and pos of robot base

        return np.array(rob_ob + self.goal, dtype=np.float32)

    def render(self, mode='human'):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))

        # Base information
        robot_id, client_id = self.arm.get_ids()
        proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=1,
                                                   nearVal=0.01, farVal=100)
        pos, ori = [list(l) for l in
                    p.getBasePositionAndOrientation(robot_id, client_id)]
        pos[2] = 0.6

        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        view_matrix = p.computeViewMatrix(pos, pos + camera_vec, up_vec)

        # Display image
        frame = p.getCameraImage(100, 100, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (100, 100, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.00001)

    def close(self):
        p.disconnect(self.client)

def main():
    val = (
        __file__.replace(os.path.dirname(__file__), "")[1:]
        + " is meant to be imported not executed"
    )
    print(f"\033[91m {val}\033[00m")


if __name__ == "__main__":
    main()

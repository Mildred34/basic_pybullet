#!/usr/bin/env python3
# from ctypes import Union
import pybullet as p
import os, sys
import math
import pybullet_data # you can provide your own data or data package from pybullet
from typing import List, Dict, Tuple, Union
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
from functions import DEBUG_JOINTS, DEBUG_POS, GUI, path_starting_from_code, LOG_PATH, DEBUG_LEVEL,ConvertEulerAngle2Quaternion, get_fingers_dict, get_arm_dict
from logs import _log_handling as log
from generator.ressources.model import Model

class _Fingers(object):
    def __init__(self,model, model_id, arm_model, finger_name):
        self.model_id = model_id
        self.arm_model = arm_model
            
        self._name = finger_name
        self._id = int(self._name.strip("finger_"))
        self.finger_config = get_fingers_dict(self.arm_model)[self._name]

        # Joints and links of the finger
        self.lid = self.get_links_ids()
        self.jid = self.get_joints_ids()

        self.joints = model.get_joints(self.jid)
        self.update_joints_pose()

        self.links = model.get_links(self.lid)

        # If there is interpenetration between the fingers and the object 
        # (hands is not in a good orientation for that finger to close)
        # we lock it into open position
        self.locked = False

        # Distance from the object
        self.distance = 10000


    def __str__(self) -> str:
        representation = "{} with: \n".format(self._name)

        for joint in self.joints:
            representation += " {} at {} \n".format(joint.name,p.getJointState(self.model_id, joint.jid)[0])

        return representation


    def add_segments(self, segment):
        pass

    def add_joint(self,joint):
        pass

    def lock(self):
        self.locked = True

    def unlock(self):
        self.locked = False

    def get_joints_ids(self):
        jointsId = []
        numJoints = p.getNumJoints(self.model_id)
        joints_name = self.finger_config["joints"]["names"].split(",")

        for j in range(numJoints):
            if p.getJointInfo(self.model_id, j)[1].decode('UTF-8') in joints_name:
                jointsId.append(j)

        if(len(jointsId) < len(joints_name)):
            Arm.logger.error("Some joints were not found")
            sys.exit()

        return jointsId

    def get_links_ids(self):
        linksId = []
        numJoints = p.getNumJoints(self.model_id)
        links_name = self.finger_config["links"].split(",")

        for j in range(numJoints):
            if p.getJointInfo(self.model_id, j)[12].decode('UTF-8') in links_name:
                linksId.append(j)

        if(len(linksId) < len(links_name)):
            Arm.logger.error("Some fingers were not found")
            sys.exit()

        return linksId


    def update_joints_pose(self):
        open_pose = self.finger_config["joints"]["open"].split(",")
        close_pose = self.finger_config["joints"]["close"].split(",")

        for i,joint in enumerate(self.joints):
            joint.update_open_pose(open_pose[i])
            joint.update_close_pose(close_pose[i])

    def close(self):
        """
        Close the finger
        """
        for joint in self.joints:
            joint.set_position(joint.close_pose,joint.limits["force"])


    def open(self):
        """
        Open the finger
        """
        for joint in self.joints:
            joint.set_position_no_force(joint.open_pose)

    def IsJointsOpened(self) -> bool:
        res = True

        for joint in self.joints:
            current_pos = p.getJointState(self.model_id,joint.jid)[0]

            if((current_pos - joint.open_pose) > 0.1):
                res = False
                break

        return res


    def adjust(self,joint_pose:dict):
        for joint in self.joints:
            
            if(joint.close_pose == 0 and joint.open_pose == 0):
                joint.set_position_no_force(joint.open_pose)
            else:
                if(joint.name in joint_pose.keys()):
                    pos = max(joint.limits["lower"], min(joint_pose[joint.name], joint.limits["upper"]))
                    # Modifier la force pour pas être en dur
                    joint.set_position_no_force(pos)#set_position_no_force(pos) or #set_position(pos,20)#joint.limits["force"])


    def axiscreator(self, linkId = -1):
        linkId = self.lid[0]
        # print(f'axis creator at bodyId = {self.robotId} and linkId = {linkId} as XYZ->RGB')
        x_axis = p.addUserDebugLine(lineFromXYZ          = [0, 0, 0] ,
                                    lineToXYZ            = [0.1, 0, 0],
                                    lineColorRGB         = [1, 0, 0] ,
                                    lineWidth            = 0.1 ,
                                    lifeTime             = 0 ,
                                    parentObjectUniqueId = self.model_id ,
                                    parentLinkIndex      = linkId )

        y_axis = p.addUserDebugLine(lineFromXYZ          = [0, 0, 0],
                                    lineToXYZ            = [0, 0.1, 0],
                                    lineColorRGB         = [0, 1, 0],
                                    lineWidth            = 0.1,
                                    lifeTime             = 0,
                                    parentObjectUniqueId = self.model_id,
                                    parentLinkIndex      = linkId)

        z_axis = p.addUserDebugLine(lineFromXYZ          = [0, 0, 0]  ,
                                    lineToXYZ            = [0, 0, 0.1],
                                    lineColorRGB         = [0, 0, 1]  ,
                                    lineWidth            = 0.1        ,
                                    lifeTime             = 0          ,
                                    parentObjectUniqueId = self.model_id     ,
                                    parentLinkIndex      = linkId     )
        return [x_axis, y_axis, z_axis]

class Arm:
    logger = None

    def __init__(self, client : int, arm_model : str):

        if(Arm.logger == None):
            self.init_log()

        self.clientId = client
        # f_name = os.path.join(os.path.dirname(__file__), 'simplecar.urdf')

        # Position et orientation initiale du robot
        self.start_pos = [0,0,0]
        self.start_orn = p.getQuaternionFromEuler([0,0,0]) # Orientation must be in quaternion
        self.arm_model = arm_model

        # Si on veut contraindre les pos dans un volume donné
        # self._workspace = {'lower': np.array([-1., -1., -1]),
        #                    'upper': np.array([1., 1., 1.])}
        
        # Add path
        self.robotId = None
        self.arm_config = get_arm_dict(self.arm_model)
        self.path = os.path.dirname(__file__) + self.arm_config["path"]
        self.endeffectorName = self.arm_config["endeffector"]

        #Chargement du modèle
        self._model = Model(self.path,self.start_pos,self.start_orn)
        self.robotId = self._model.model_id

        ## Configuration robot
        self._fingers = None
        self._fingersJointsiD = []
        self._joints = None
        self.numJoints = None
        self.endeffectorId = None
        self.NonFixedJoints = []
        self.palmlinkname = self.arm_config["palm"]["link"]
        self._palmId = None

        # Go to home pose
        #self.go_home()

        # Other parameters
        self.maxIter = 100 # Iteration on inverse kinematics calculation

        # Optional parameters for IK, read from urdf
        #lower limits for null space
        self.ik_lower_limits = []
        #upper limits for null space
        self.ik_upper_limits = []
        #joint ranges for null space
        self.ik_joint_ranges = []
        #restposes for null space
        self.ik_rest_poses = []
        #joint damping coefficents
        self.ik_jd = [] # Damped Least Squares IK method

        self.arm_configuration()

    def __str__(self) -> str:
        representation = "Arm {} with: \n".format(self.arm_model)

        for finger in self._fingers:
            representation += str(finger)

        return representation

    def init_log(self):
         
        path_log = path_starting_from_code(0) + LOG_PATH()

        #Initialisation du mode debug ou release
        # Choix du mode DEBUG ou RELEASE
        if(DEBUG_LEVEL() == log.DEBUG_LEVEL.DEBUG_SOFT):
            #log.config["filename_debug"] = path_log + "debug"
            log.config["loggername"] = "arm"
            Arm.logger = log.factory.create('DEBUG',**log.config)
        else:
            #log.config["filename_release"] = path_log + "release"
            log.config["loggername"] = "arm"
            Arm.logger = log.factory.create('RELEASE',**log.config)

    def get_ids(self) -> Tuple[int,int]:
        return self.robotId, self.clientId


    def get_handId(self) -> int:
        Id = 0

        for j in range(self.numJoints):
            Arm.logger.debug("Link name:{}".format(p.getJointInfo(self.robotId, j, physicsClientId=self.clientId)[12].decode('UTF-8')))

            # Retourne le nom du segment en byte
            if p.getJointInfo(self.robotId, j, physicsClientId=self.clientId)[12].decode('UTF-8') == self.endeffectorName:
                Id = j

        if(Id == 0):
            Arm.logger.error("End effector not found")
            sys.exit()

        return Id

    def get_palmId(self) -> int:
        Id = 0

        for j in range(self.numJoints):
            Arm.logger.debug("Link name:{}".format(p.getJointInfo(self.robotId, j, physicsClientId=self.clientId)[12].decode('UTF-8')))

            # Retourne le nom du segment en byte
            if p.getJointInfo(self.robotId, j, physicsClientId=self.clientId)[12].decode('UTF-8') == self.palmlinkname:
                Id = j

        if(Id == 0):
            Arm.logger.error("End effector not found")
            sys.exit()

        return Id

    def get_nonfixedjoints(self):
        for j in range(self.numJoints):
            if p.getJointInfo(self.robotId, j, physicsClientId=self.clientId)[2] != p.JOINT_FIXED:
                self.NonFixedJoints.append(j)

    def get_fingers(self):

        fingers = []
        i = 1
        fingers_config = get_fingers_dict(self.arm_model)


        for finger in fingers_config.keys() :
            fingers.append(_Fingers(self._model,self.robotId,self.arm_model,finger))
            i+=1

        try:
            assert(len(fingers) > 0)

        except AssertionError:
            Arm.logger.error("Fingers were not found")
            sys.exit(1) # 0 no error 1 error

        return fingers
    
    def is_there2many_fingerslocked(self):
        """
        Compte le nombre de doigt bloqué en position ouverte
        Si - de
        """
        # nombre de doigt en contact
        nb_fingers = len(self._fingers)
        nb_fingers_locked = 0
        res = False

        for finger in self._fingers:
            if(finger.locked):
                nb_fingers_locked += 1

        if((nb_fingers - nb_fingers_locked) < 2):
            res = True

        return res

    def unlockallfingers(self):
        for finger in self._fingers:

            if(finger.locked):
                Arm.logger.info("{} has been unlocked".format(finger._name))
                finger.locked = False

    def get_fingersJoints(self):
        jid = []

        for finger in self._fingers:
            jid.append(finger.jid)

        return jid

    def add_damping2joints(self):
        """
        Currently damping coefficients are fixed to 0.1.
        """
        for j in range(self.numJoints):
            if p.getJointInfo(self.robotId, j, physicsClientId=self.clientId)[2] != p.JOINT_FIXED:
                self.ik_jd.append(0.1)

    def arm_configuration(self):
        self._joints = self._model.joints
        self.numJoints = p.getNumJoints(self.robotId)
        self.endeffectorId = self.get_handId()
        self._fingers = self.get_fingers()
        self._fingersJointsiD = self.get_fingersJoints()
        self._palmId = self.get_palmId()

        # Get non fixed joints
        self.get_nonfixedjoints()

        self.add_damping2joints()

        # Updates ik parameters
        self.update_ik_parameters(self.robotId, self.endeffectorId, False)

    def update_ik_parameters(self, body : int, target_joint : int, half_range : bool = False):
        
        # Configure IK optional parameters from joint info read from urdf file
        for jointId in range(self.numJoints):
            joint = self._joints[jointId]
            jointType = p.getJointInfo(body, jointId, physicsClientId=self.clientId)[2]

            if jointType != p.JOINT_FIXED:
                self.ik_lower_limits.append(joint.limits["lower"])
                self.ik_upper_limits.append(joint.limits["upper"])

                if not half_range:
                    self.ik_joint_ranges.append(joint.ranges)
                else:
                    self.ik_joint_ranges.append(joint.ranges/2)

                self.ik_rest_poses.append(joint.rest_pose)

                # self.ik_rest_poses[key].append((upper_limit + lower_limit)/2.0)


    def axiscreator(self, linkId = -1):
        linkId = self.endeffectorId
        # print(f'axis creator at bodyId = {self.robotId} and linkId = {linkId} as XYZ->RGB')
        x_axis = p.addUserDebugLine(lineFromXYZ          = [0, 0, 0] ,
                                    lineToXYZ            = [0.1, 0, 0],
                                    lineColorRGB         = [1, 0, 0] ,
                                    lineWidth            = 0.1 ,
                                    lifeTime             = 0 ,
                                    parentObjectUniqueId = self.robotId ,
                                    parentLinkIndex      = linkId )

        y_axis = p.addUserDebugLine(lineFromXYZ          = [0, 0, 0],
                                    lineToXYZ            = [0, 0.1, 0],
                                    lineColorRGB         = [0, 1, 0],
                                    lineWidth            = 0.1,
                                    lifeTime             = 0,
                                    parentObjectUniqueId = self.robotId,
                                    parentLinkIndex      = linkId)

        z_axis = p.addUserDebugLine(lineFromXYZ          = [0, 0, 0]  ,
                                    lineToXYZ            = [0, 0, 0.1],
                                    lineColorRGB         = [0, 0, 1]  ,
                                    lineWidth            = 0.1        ,
                                    lifeTime             = 0          ,
                                    parentObjectUniqueId = self.robotId     ,
                                    parentLinkIndex      = linkId     )
        return [x_axis, y_axis, z_axis]


    def ik(self, body : int, target_joint : int, target_pos : List,threshold : int, target_orient : List = None, 
    max_iterations : int = 1000, half_range : bool = False
     ):
        closeEnough = False # Find a solution close enough
        iter = 0 
        distpos = 1e30 # distance to minimize
        distori = 1e30
        key = '%d_%d' % (body, target_joint)
        target_joint_positions = [0 for i in range(self.numJoints)]

        # Calcul des positions articulaires pour atteindre la pos ciblée
        # Prend en compte les limites articulaires définies dans l'urdf
        while (not closeEnough and iter < max_iterations):
            if target_orient is not None:
                ik_joint_poses = p.calculateInverseKinematics(body, target_joint, targetPosition=target_pos, 
                targetOrientation=target_orient, lowerLimits=self.ik_lower_limits, upperLimits=self.ik_upper_limits, jointRanges=self.ik_joint_ranges, 
                restPoses=self.ik_rest_poses, maxNumIterations=max_iterations, physicsClientId=self.clientId)
            else:
                ik_joint_poses = p.calculateInverseKinematics(body, target_joint, targetPosition=target_pos, 
                lowerLimits=self.ik_lower_limits, upperLimits=self.ik_upper_limits, jointRanges=self.ik_joint_ranges, 
                restPoses=self.ik_rest_poses, maxNumIterations=max_iterations, physicsClientId=self.clientId)

            j = 0
            k = 0 # Number of spherical joint

            for i in range(self.numJoints): # Faire attention aux articulations:  "fixed base and fixed joints are skipped"
                joint_type = p.getJointInfo(self.robotId, i, physicsClientId=self.clientId)[2]

                if joint_type != p.JOINT_FIXED:
                    if joint_type != p.JOINT_SPHERICAL:
                        target_joint_positions[i] = ik_joint_poses[i-j+k*2]
                        p.resetJointState(self.robotId, i, target_joint_positions[i])
                    else:
                        target_joint_positions[i] = [ik_joint_poses[i-j],ik_joint_poses[i-j+1], ik_joint_poses[i-j+2]]
                        k += 1
                else :
                    j += 1
                    target_joint_positions[i] = 0

            ls = p.getLinkState(self.robotId, self.endeffectorId)
            newPos = ls[4]
            newOri = ls[5]

            # On compare la distance
            diffpos = [target_pos[0] - newPos[0], target_pos[1] - newPos[1], target_pos[2] - newPos[2]]
            distpos = math.sqrt((diffpos[0] * diffpos[0] + diffpos[1] * diffpos[1] + diffpos[2] * diffpos[2]))

            # Et l'orientation
            if type(target_orient)==list and len(target_orient) > 0:
                target_ang = p.getEulerFromQuaternion(target_orient)
                new_ang = p.getEulerFromQuaternion(newOri)
                diffori = [target_ang[0] - new_ang[0], target_ang[1] - new_ang[1], target_ang[2] - new_ang[2]]
                distori = math.sqrt((diffori[0] * diffori[0] + diffori[1] * diffori[1] + diffori[2] * diffori[2]))

            
            
            closeEnough = (distpos < threshold) # and (distori < 1*math.pi/180)
            iter = iter + 1

        if(closeEnough == False): # DEBUG
            Arm.logger.debug("No Ik solution found!!!")
        
        # print(j_names)
        # print(ik_joint_poses)
        # exit()

        return target_joint_positions, closeEnough


    def accurateCalculateInverseKinematics(self, targetPos : List, target_orient : List = None, threshold : float = 0.01, maxIter : int = 100) -> List:
        """
        Calcule la cinématique inversé pour atteindre la position ciblé nommé targetPos.
        Une loop avec un nombre d'itération max se fait afin d'atteindre au plus proche la cible.

        ---- WIP ----
            - Prise en compte des collisions
            - Eviter de s'approcher des positions singulières (quand on s'en rapproche, ca fait nimp par la suite)

        """
        closeEnough = False
        iter = 0
        distpos = 1e30
        distori = 1e30
        target_joint_positions = [0 for i in range(self.numJoints)]
        while (not closeEnough and iter < maxIter):
            ik_joint_poses = p.calculateInverseKinematics(self.robotId, self.endeffectorId, targetPosition=targetPos, 
                targetOrientation=target_orient)

            j = 0
            k = 0 # Number of spherical joint


            for i in range(self.numJoints): # Faire attention aux articulations:  "fixed base and fixed joints are skipped"
                joint_type = p.getJointInfo(self.robotId, i, physicsClientId=self.clientId)[2]

                if joint_type != p.JOINT_FIXED:
                    if joint_type != p.JOINT_SPHERICAL:
                        target_joint_positions[i] = ik_joint_poses[i-j+k*2]
                        p.resetJointState(self.robotId, i, target_joint_positions[i])
                    else:
                        target_joint_positions[i] = [ik_joint_poses[i-j],ik_joint_poses[i-j+1], ik_joint_poses[i-j+2]]
                        p.resetJointStateMultiDof(self.robotId, i, target_joint_positions[i])
                        k += 1
                else :
                    j += 1
                    target_joint_positions[i] = 0

            ls = p.getLinkState(self.robotId, self.endeffectorId)
            newPos = ls[4]
            newOri = ls[5]

            # On compare la distance
            diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
            distpos = math.sqrt((diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]))

            # Et l'orientation
            if type(target_orient)==list and len(target_orient) > 0:
                target_ang = p.getEulerFromQuaternion(target_orient)
                new_ang = p.getEulerFromQuaternion(newOri)
                diff = [target_ang[0] - new_ang[0], target_ang[1] - new_ang[1], target_ang[2] - new_ang[2]]
                distori = math.sqrt((diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]))
                
            closeEnough = (distpos < threshold) # and (distori < 1*math.pi/180)
            
            iter = iter + 1
        #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))

        if(closeEnough == False):
            Arm.logger.debug("No Ik solution found!!!")
        
            
        return target_joint_positions, closeEnough

    def setJointAngleById(self, model, joint_id, joint_type, desired_angle ):

        if joint_type == p.JOINT_SPHERICAL:
            
            if(len(desired_angle) < 3):
                desired_angle.insert(0,0)

            desired_angle = ConvertEulerAngle2Quaternion(*desired_angle)
            p.setJointMotorControlMultiDof(bodyUniqueId=model,jointIndex=joint_id, controlMode=p.POSITION_CONTROL, targetPosition=desired_angle)

        elif joint_type in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]:
            p.setJointMotorControl2( bodyIndex=model, jointIndex=joint_id, controlMode=p.POSITION_CONTROL, targetPosition=desired_angle)


    def apply_action(self, position : List = None,**dic) -> bool:
        """
        Go to position ; step by step
        """

        res = False

        if(GUI() and DEBUG_JOINTS() and not DEBUG_POS()):

            try:
                assert(len(dic) > 0)

            except AssertionError:
                Arm.logger.error("No input given")
                sys.exit(-1)

            for joint_id in range(self.numJoints):

                joint_info = p.getJointInfo(self.robotId, joint_id)
                joint_type = joint_info[2]

                if(joint_type != p.JOINT_FIXED):

                    if("Joint_"+str(joint_id) not in dic.keys()):
                        Arm.logger.error("Joint not found in the position dictionnary!")
                        sys.exit()

                    self.setJointAngleById(self.robotId, joint_id, joint_type, dic["Joint_"+str(joint_id)])

            res = True
        else:
            if(position != None):


                target_joint_positions, close_enough = self.accurateCalculateInverseKinematics(targetPos = position[:3],target_orient = position[3:],
                      threshold = 0.01, maxIter = self.maxIter)

                # target_joint_positions, close_enough = self.ik(self.robotId, self.endeffectorId, target_pos = position[:3],threshold = 0.01, target_orient = position[3:], 
                #     max_iterations = self.maxIter, half_range = False
                #     )

                if(close_enough):
                    res = True
                    for joint_id in self.NonFixedJoints:
                        if(joint_id not in self._fingersJointsiD):
                            joint_info = p.getJointInfo(self.robotId, joint_id)
                            joint_type = joint_info[2]

                            self.setJointAngleById(self.robotId, joint_id, joint_type, target_joint_positions[joint_id])

                            # p.setJointMotorControl2(
                            #                 bodyIndex      = self.robotId,
                            #                 jointIndex     = i,
                            #                 controlMode    = p.POSITION_CONTROL,
                            #                 targetPosition = target_joint_positions[i],
                            #                 targetVelocity = 0,
                            #                 force          = 1, # 500
                            #                 positionGain   = 1, # 1
                            #                 velocityGain   = 0.1)
                else:
                    #      val = "IK didn't find a solution close enough"
                    #      print(f"\033[91m {val}\033[00m")
                    pass


                if( not DEBUG_POS()):
                    # Close the grip # plus utilliser
                    if((dic["grip"])) and (dic["joint"] is None):
                        self.close_grip()

                    elif((dic["joint"] is not None) and 
                            (dic["grip"] != 0)):
                        # self.close_grip()
                        self.adjust_grip(dic["joint"])

                    # Open the grip
                    else:
                        self.open_grip()



        return res


    def get_observation_euler(self,link_index) -> tuple:
        """
        Get the position and orientation of any link in the arm in the simulations
        (COM frame not urdf frame)
        """
        result = p.getLinkState(bodyUniqueId = self.robotId, 
                                  linkIndex = link_index,
                                  physicsClientId = self.clientId)

        if(result is None):
            Arm.logger.error("None type found after observation!!!")

        pos = result[0]
        ang = result[1]

        ang = p.getEulerFromQuaternion(ang)
        ori = (math.cos(ang[2]), math.sin(ang[2])) # rotation z --> vecteur directeur projeté sur le plan (x,y)

        return pos+ang

    def get_observation_quaternion(self,link_index) -> tuple:
        """
        Get the position and orientation of any link in the arm in the simulations
        (COM frame not urdf frame)
        """
        result = p.getLinkState(bodyUniqueId = self.robotId, 
                                  linkIndex = link_index,
                                  physicsClientId = self.clientId)

        if(result is None):
            Arm.logger.error("None type found after observation!!!")

        pos = result[0]
        ang = result[1]

        return pos+ang

    def go_home(self):
        """
        DEPRECATED
        """
        rest_poses = [0,0.413,0,-0.231,-0.281,1.571,-1.571,0.08,0.08]
        
        for i in range(7):
            p.resetJointState(self.robotId,i, rest_poses[i])

    def check_openfingers(self):
        res = True

        for finger in self._fingers:
            res = finger.IsJointsOpened()
            if res == False:
                break

        return res


    def open_grip(self):
        for finger in self._fingers:
            finger.open()

    def adjust_grip(self, joint_pos : dict):
        for finger in self._fingers:
            if not finger.locked : 
                finger.adjust(joint_pos[finger._name])
            else:
                finger.open()
            

    def close_grip(self):
        for finger in self._fingers:
            if not finger.locked : 
                finger.close()
            else:
                finger.open()

            # finger.set_position(finger.limits["upper"])


def main():
    val = (
        __file__.replace(os.path.dirname(__file__), "")[1:]
        + " is meant to be imported not executed"
    )
    print(f"\033[91m {val}\033[00m")


if __name__ == "__main__":
    main()










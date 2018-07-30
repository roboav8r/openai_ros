import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import sawyer_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

timestep_limit_per_episode = 10000 # Can be any Value

register(
        id='ShadowTcGetBall-v0',
        entry_point='openai_ros:ShadowTcGetBallEnv',
        timestep_limit=timestep_limit_per_episode,
    )

class ShadowTcGetBallEnv(shadow_tc_env.SawyerEnv):
    def __init__(self):
        """
        Make sawyer learn how pick up a ball
        """
        
        # We execute this one before because there are some functions that this
        # TaskEnv uses that use variables from the parent class, like the effort limit fetch.
        super(ShadowTcGetBallEnv, self).__init__()
        
        # Here we will add any init functions prior to starting the MyRobotEnv
        
        
        # Only variable needed to be set here

        rospy.logdebug("Start ShadowTcGetBallEnv INIT...")
        number_actions = rospy.get_param('/sawyer/n_actions')
        self.action_space = spaces.Discrete(number_actions)
        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)
        
        
        self.movement_delta =rospy.get_param("/shadow_tc/movement_delta")
        
        self.work_space_x_max = rospy.get_param("/shadow_tc/work_space/x_max")
        self.work_space_x_min = rospy.get_param("/shadow_tc/work_space/x_min")
        self.work_space_y_max = rospy.get_param("/shadow_tc/work_space/y_max")
        self.work_space_y_min = rospy.get_param("/shadow_tc/work_space/y_min")
        self.work_space_z_max = rospy.get_param("/shadow_tc/work_space/z_max")
        self.work_space_z_min = rospy.get_param("/shadow_tc/work_space/z_min")
        
        self.max_effort = rospy.get_param("/shadow_tc/max_effort")
        
        self.dec_obs = rospy.get_param("/shadow_tc/number_decimals_precision_obs")
        
        self.acceptable_distance_to_ball = rospy.get_param("/shadow_tc/acceptable_distance_to_ball")
        
        self.tcp_z_position_min = rospy.get_param("/shadow_tc/tcp_z_position_min")
        
        # We place the Maximum and minimum values of observations
        # TODO: Fill when get_observations is done.
        
        
        # We fetch the limits of the joinst to get the effort and angle limits
        self.joint_limits = self.init_joint_limits()
        
        high = numpy.array([self.work_space_x_max,
                            self.work_space_y_max,
                            self.work_space_z_max,
                            1])
                                        
        low = numpy.array([ self.work_space_x_min,
                            self.work_space_y_min,
                            self.work_space_z_min,
                            0])

        
        self.observation_space = spaces.Box(low, high)
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards
        
        self.done_reward =rospy.get_param("/shadow_tc/done_reward")
        self.closer_to_block_reward = rospy.get_param("/shadow_tc/closer_to_block_reward")

        self.cumulated_steps = 0.0

        
        
        rospy.logdebug("END shadow_tcGetBallEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the UR5 arm to the initial position and the objects to the original position.
        """

        # We set the angles to zero of the limb
        self.reset_scene()
        # Un check just in case was left activated
        self.get_finguers_colision(False)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """

        # For Info Purposes
        self.cumulated_reward = 0.0
        
        
        self.ball_pose = self.get_ball_pose()
        tcp_pose = self.get_tip_pose()
        self.previous_distance_from_ball = self.get_distance_from_point(self.ball_pose.position, tcp_pose.position)
        
        
        

    def _set_action(self, action):
        """
        It sets the joints of shadow_tc based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """
        
        rospy.logdebug("Start Set Action ==>"+str(action))
       
        
        increment_vector = Vector3() 
        action_id="move"
        
        if action == 0: # Increase X
            increment_vector.x = self.movement_delta
        elif action == 1: # Decrease X
            increment_vector.x = -1*self.movement_delta
        elif action == 2: # Increase Y
            increment_vector.x = self.movement_delta
        elif action == 3: # Decrease Y
            increment_vector.x = -1*self.movement_delta
        elif action == 4: # Increase Z
            increment_vector.x = self.movement_delta
        elif action == 5: # Decrease Z
            increment_vector.x = -1*self.movement_delta
        elif action == 6: # Open Claw
            action_id = "open"
        elif action == 7: # Close Claw
           action_id = "close"
        

        if action_id == "move":
            # We tell shadow_tc the action to perform
            # We dont change the RPY, therefore it will always be zero
            self.move_tip(  x=increment_vector.x,
                            y=increment_vector.y,
                            z=increment_vector.z)
        elif  action_id == "open":
            self.open_hand()
        elif action_id == "close":
            self.close_hand()
            
        
        rospy.logdebug("END Set Action ==>"+str(action)+",action_id="+str(action_id)+",IncrementVector===>"+str(increment_vector))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        shadow_tcEnv API DOCS.
        :return: observation
        """
        rospy.logdebug("Start Get Observation ==>")
        
        tcp_pose = self.get_tip_pose()
        
        # We dont add it to the observations because is not part of the robot
        self.ball_pose = self.get_ball_pose()
        
        # We activate the Finguer collision detection
        self.finguer_collided = self.get_finguers_colision(True)
        self.get_finguers_colision(False)
        
        observation = [ round(ball_pose.position.x,self.dec_obs),
                        round(ball_pose.position.y,self.dec_obs),
                        round(ball_pose.position.z,self.dec_obs),
                        int(self.finguer_collided)
                        ]

        return observation
        

    def _is_done(self, observations):
        """
        We consider the episode done if:
        1) The shadow_tc TCP is outside the workspace.
        2) The TCP to block distance is lower than a threshold ( it got to the place )
           and the the collisions in the figuers are true.
        """
        tcp_pos = Vector3()
        tcp_pos.x = observations[0]
        tcp_pos.y = observations[1]
        tcp_pos.z = observations[2]
        finguers_collided = observations[3]
        
        is_inside_workspace = self.is_inside_workspace(tcp_pos)
        
        
        has_reached_the_ball = self.reached_ball(  tcp_pos,
                                                    self.ball_pose.position,
                                                    self.acceptable_distance_to_ball,
                                                    finguers_collided)
        
        done = has_reached_the_ball or not(is_inside_workspace)
        
        rospy.logdebug("#### IS DONE ? ####")
        rospy.logdebug("Not is_inside_workspace ?="+str(not(is_inside_workspace)))
        rospy.logdebug("has_reached_the_ball ?="+str(has_reached_the_ball))
        rospy.logdebug("done ?="+str(done))
        rospy.logdebug("#### #### ####")
        
        return done

    def _compute_reward(self, observations, done):
        """
        We Base the rewards in if its done or not and we base it on
        if the distance to the block has increased or not.
        :return:
        """

        tcp_pos = Vector3()
        tcp_pos.x = observations[0]
        tcp_pos.y = observations[1]
        tcp_pos.z = observations[2]
        
        self.distance_from_ball = self.get_distance_from_point(self.ball_pose.position, tcp_pos)
        
        distance_difference =  distance_from_ball - self.previous_distance_from_ball


        if not done:
            
            # If there has been a decrease in the distance to the desired point, we reward it
            if distance_difference < 0.0:
                rospy.logdebug("DECREASE IN DISTANCE GOOD")
                reward = self.closer_to_block_reward
            else:
                rospy.logerr("ENCREASE IN DISTANCE BAD")
                #reward = -1*self.closer_to_block_reward
                reward = 0.0

        else:
            

        
            if self.reached_ball(tf_tcp_to_block_vector,self.acceptable_distance_to_cube,self.translation_tcp_world[2], self.tcp_z_position_min):
                reward = self.done_reward
            else:
                reward = -1*self.done_reward


        self.previous_distance_from_ball = distance_block_to_tcp


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward


    # Internal TaskEnv Methods
    def is_arm_stuck(self, joints_efforts_dict):
        """
        Checks if the efforts in the arm joints exceed certain theshhold
        We will only check the joints_0,1,2,3,4,5,6
        """
        is_arm_stuck = False
        
        for joint_name in self.joint_limits.joint_names:
            if joint_name in joints_efforts_dict:
                
                effort_value = joints_efforts_dict[joint_name]
                index = self.joint_limits.joint_names.index(joint_name)
                effort_limit = self.joint_limits.effort[index]
                
                rospy.logdebug("Joint Effort ==>Name="+str(joint_name)+",Effort="+str(effort_value)+",Limit="+str(effort_limit))

                if abs(effort_value) > effort_limit:
                    is_arm_stuck = True
                    rospy.logerr("Joint Effort TOO MUCH ==>"+str(joint_name)+","+str(effort_value))
                    break
                else:
                    rospy.logdebug("Joint Effort is ok==>"+str(joint_name)+","+str(effort_value))
            else:
                rospy.logdebug("Joint Name is not in the effort dict==>"+str(joint_name))
        
        return is_arm_stuck
    
    
    def reached_ball(self,tcp_position, ball_position, minimum_distance, finguers_collided):
        """
        Return true if the distance from TCP position to the ball position is 
        lower than the minimum_distance.
        """
        
        distance_from_ball = self.get_distance_from_point(tcp_position, ball_position)
        
        distance_to_ball_ok = distance_from_ball < minimum_distance
        
        reached_ball_b = distance_to_ball_ok and finguers_collided
        
        rospy.logdebug("###### REACHED BLOCK ? ######")
        rospy.logdebug("distance_from_ball==>"+str(distance_from_ball))
        rospy.logdebug("distance_to_ball_ok==>"+str(distance_to_ball_ok))
        rospy.logdebug("reached_ball_b==>"+str(reached_ball_b))
        rospy.logdebug("finguers_collided==>"+str(finguers_collided))
        rospy.logdebug("############")
        
        return reached_ball_b
    
    def get_distance_from_desired_point(self, current_position):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        distance = self.get_distance_from_point(current_position,
                                                self.desired_point)
    
        return distance
        
    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((pstart.x, pstart.y, pstart.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))
    
        distance = numpy.linalg.norm(a - b)
    
        return distance
    
    def get_magnitud_tf_tcp_to_block(self, translation_vector):
        """
        Given a Vector3 Object, get the magnitud
        :param p_end:
        :return:
        """
        a = numpy.array((   translation_vector.x,
                            translation_vector.y,
                            translation_vector.z))
        
        distance = numpy.linalg.norm(a)
    
        return distance
        
    def get_orientation_euler(self, quaternion_vector):
        # We convert from quaternions to euler
        orientation_list = [quaternion_vector.x,
                            quaternion_vector.y,
                            quaternion_vector.z,
                            quaternion_vector.w]
    
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw
        
    def is_inside_workspace(self,current_position):
        """
        Check if the shadow_tc is inside the Workspace defined
        """
        is_inside = False

        rospy.logdebug("##### INSIDE WORK SPACE? #######")
        rospy.logdebug("XYZ current_position"+str(current_position))
        rospy.logdebug("work_space_x_max"+str(self.work_space_x_max)+",work_space_x_min="+str(self.work_space_x_min))
        rospy.logdebug("work_space_y_max"+str(self.work_space_y_max)+",work_space_y_min="+str(self.work_space_y_min))
        rospy.logdebug("work_space_z_max"+str(self.work_space_z_max)+",work_space_z_min="+str(self.work_space_z_min))
        rospy.logdebug("############")

        if current_position.x > self.work_space_x_min and current_position.x <= self.work_space_x_max:
            if current_position.y > self.work_space_y_min and current_position.y <= self.work_space_y_max:
                if current_position.z > self.work_space_z_min and current_position.z <= self.work_space_z_max:
                    is_inside = True
        
        return is_inside
        
    

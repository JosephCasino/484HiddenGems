import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode
import matplotlib.pyplot as plt
from tqdm import tqdm

import random

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):

            # (Default) The whole map
            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)


            ## first quadrant
            # x = np.random.uniform(world.width/2, world.width)
            # y = np.random.uniform(world.height/2, world.height)

            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))

        ###############

        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):
        if x1 is None: # If the robot recieved no sensor measurement, the weights are in uniform distribution.
            return 1./len(self.particles)
        else:
            tmp1 = np.array(x1)
            tmp2 = np.array(x2)
            return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))


    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """

        ## TODO #####
        for particle in self.particles:
            particle.weight = self.weight_gaussian_kernel(readings_robot, particle.read_sensor())
        
        weight_list = np.zeros(self.num_particles)
        for i in range(self.num_particles):
            weight_list[i] = self.particles[i].weight 

        weight_list = weight_list / np.sum(weight_list)

        for i in range(self.num_particles):
            self.particles[i].weight  = weight_list[i] 


        ###############
        # pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()

        ## TODO #####

        # get the weights and normalize them
        weight_list = np.zeros(self.num_particles)
        for i in range(self.num_particles):
            weight_list[i] = self.particles[i].weight 

        # weight_list = weight_list / np.sum(weight_list)

        # calculate the cumulative sum of the weights
        cumu_sum = np.cumsum(weight_list)
        
        # print(f'should be 1: {cumu_sum[-1]}')

        # resampling
        for i in range(self.num_particles):
            rand_num = random.uniform(cumu_sum[0],cumu_sum[-1])
            idx = 0
            for j in range(self.num_particles):
                if cumu_sum[j] > rand_num: 
                    idx = j
                    break
            particles_new.append(Particle(x = self.particles[idx].x, y = self.particles[idx].y , maze = self.world , heading = self.particles[idx].heading, 
                                          weight = self.particles[idx].weight, sensor_limit = self.sensor_limit, noisy=True))
        ###############

        self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
            You can either use ode function or vehicle_dynamics function provided above
        """
        ## TODO #####
        for particle in self.particles:
            for control in self.control:

                dx, dy, dtheta = vehicle_dynamics(t = None, vars=[particle.x, particle.y, particle.heading], vr = control[0] , delta=control[1])
                particle.x += dx * 0.01
                particle.y += dy * 0.01
                particle.heading += dtheta * 0.01
        
        self.control = []
        ###############
        # pass


    def runFilter(self, distance_error, orientation_error):
        """
        Description:
            Run PF localization
        """

        # distance_error = []
        # orientation_error = []

        count = 0 
        for i in tqdm(range(800)):
            ## TODO: (i) Implement Section 3.2.2. (ii) Display robot and particles on map. (iii) Compute and save position/heading error to plot. #####

            # print('test')
            # print(f'control_list: {self.control}')

            self.world.clear_objects()

            self.particleMotionModel()
            self.updateWeight(self.bob.read_sensor())


            self.world.show_particles(self.particles, show_frequency = 10)
            x_e, y_e, h_e = self.world.show_estimated_location(self.particles)
            self.world.show_robot(self.bob)

            distance_error.append(np.sqrt(pow(self.bob.x - x_e, 2) + pow(self.bob.y - y_e, 2)))
            orientation_error.append(np.abs(self.bob.heading - h_e))



            self.resampleParticle()
            count += 1




            ###############

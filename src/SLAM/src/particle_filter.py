import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode
import math

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
        # particles_new = list()
        for i in range(num_particles):
            x = np.random.uniform(0, world.width)  
            y = np.random.uniform(0, world.height)
            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))
            # particles_new.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))
        self.particles = particles  
        # self.particles_new = particles_new        # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        self.control_cur_count_state = 0
        self.control_pub_count_state = 0
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)
        self.control_pub_count_state += 1
        # print(self.control)

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

    def weight_gaussian_kernel(self,x1, x2, std = 12500): #12500
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
        particle_distance_inf = []
        car_distance_info = []
        car_distance_info = self.bob.read_sensor()
        weight = []
        ## TODO #####
        for i in range (len(self.particles)):
            particle_distance_inf = self.particles[i].read_sensor()
            self.particles[i].weight = self.weight_gaussian_kernel(particle_distance_inf,car_distance_info)
            # print("distance:" , car_distance_info, "\n",particle_distance_inf, " \nweights: ", self.particles[i].weight)
            weight.append(self.particles[i].weight)

        
        return weight
        ###############

    def resampleParticle(self,weight):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        # particles_new = list()

        ## TODO #####
        cum_weight = np.cumsum(weight)
        # print(cum_weight[-1])

        total_weight = (cum_weight[-1])
        # np.amax(weight)
        # print(total_weight, cum_weight[-1])
        cum_weight = cum_weight/(cum_weight[-1])
        # print(cum_weight)
        a = np.random.rand(1,len(cum_weight))
        # print("a is: ", a[0][0])
        particles_new = list()
        for i in range (len(cum_weight)):
            indices = np.where(cum_weight>a[0][i])[0][0]
            self.particles[i].weight = self.particles[i].weight/total_weight
            particles_new.append(Particle(x = self.particles[indices].x, y = self.particles[indices].y, maze = self.world, sensor_limit = self.sensor_limit, noisy = True))
            particles_new[-1].heading = self.particles[indices].heading
        ###############


        # for j in range(10):
        #     print(j)
        #     print("new:", particles_new[j].sensor_limit)
        #     print("part:", self.particles[j].sensor_limit)
        
        self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to 
            the control input from actual robot 
        """
        ## TODO #####
        loops = self.control_pub_count_state - self.control_cur_count_state
        
        # print(loops)
        for i  in range(loops):
            for j in range(len(self.particles)):
                vars = [self.particles[j].x, self.particles[j].y, self.particles[j].heading]
                if self.control:
                    vr = self.control[i + self.control_cur_count_state][0]               ## problem 2 add all control instead of the last one 
                    delta = self.control[i + self.control_cur_count_state][1]
                else:
                    vr = 0
                    delta = 0
                t = .01
                vehicle_dyn = vehicle_dynamics(t, vars, vr, delta)

                # no ode package lols
                # print(self.particles[j].x, self.particles[j].y, vehicle_dyn[0])
                self.particles[j].x = self.particles[j].x + t*vehicle_dyn[0]
                self.particles[j].y = self.particles[j].y + t*vehicle_dyn[1]
                self.particles[j].heading = (((self.particles[j].heading + (t*vehicle_dyn[2]))*180/np.pi)%360)*(np.pi/180.0)
                # print("theta: ", (particle.heading + (t*vehipycle_dyn[2])) % (2.0*math.pi))
            # print(self.particles[-1].heading,self.particles[-1].heading + (t*vehicle_dyn[2]))
        self.control_cur_count_state = self.control_pub_count_state


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        # iterations = 0
        # file2write = open("Sensorlim_" + str(self.sensor_limit) + "_particlesnum_" + str(self.num_particles),'w')
        while True:
            ## TODO #####
            # Finish this function to have the particle filter running
            self.particleMotionModel()
            # Read sensor msg
            reading = self.getModelState()
            weight = self.updateWeight(reading)
            self.resampleParticle(weight)
            # Display robot and particles on map 
            # print("size of particles list: ", len(self.particles))
            self.world.show_particles(particles = self.particles, show_frequency = 10)
            self.world.show_robot(robot = self.bob)
            # print("size of particles list: ", len(self.particles))
            [est_x,est_y, est_orientation] = self.world.show_estimated_location(particles = self.particles)
            self.world.clear_objects()

            # file2write.write(str(iterations))
            # file2write.write(" ")
            # file2write.write(str(self.bob.x))
            # file2write.write(" ")
            # file2write.write(str(self.bob.y))
            # file2write.write(" ")
            # file2write.write(str(self.bob.heading))
            # file2write.write(" ")
            # file2write.write(str(est_x))
            # file2write.write(" ")
            # file2write.write(str(est_y))
            # file2write.write(" ")
            # file2write.write(str(est_orientation))
            # file2write.write("\n")
            # iterations += 1

        # file2write.close()

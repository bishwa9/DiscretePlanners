import numpy
import pylab as pl
import random
import math
import time
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.boundary_limits = [self.lower_limits, self.upper_limits]

        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):
        n = 1
        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        grid_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        num = self.discrete_env.dimension*2
        dimension_to_inc = 0
        increments = [n, -1*n] #basically get increment by +n then -n in each dimension
        for i in range(self.discrete_env.dimension): #iterate through each dimension (4-connected)
            for j in range(len(increments)): #apply the each inc on each dimension
                neighbor = list(grid_coord)
                neighbor[i] += increments[j] #ith joint with jth increment

                neighbor_id = self.discrete_env.GridCoordToNodeId(neighbor)
                successors.append(neighbor_id)
        return successors

        # successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        # grid_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        # successors = [self.discrete_env.GridCoordToNodeId([grid_coord[0] + x, grid_coord[1] + y])
        #               for x, y in [[-1, 0], [0, 1], [1, 0], [0, -1]]
        #               if -1 < grid_coord[0] + x < self.discrete_env.num_cells[0] and -1 < grid_coord[1] + y < self.discrete_env.num_cells[1]]
        # return successors
    
    def ComputeConfigDistance(self, start_config, end_config):
        ConfigDist = numpy.linalg.norm(end_config - start_config)
        return ConfigDist
    
    
    def ComputeDistance(self, start_id, end_id):

        # dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        cost = numpy.linalg.norm(numpy.array(end_config) - numpy.array(start_config)) / 2.0
        # Euclidean Distance (Direction)
        #cost = numpy.sqrt(sum([(x1 - x2)**2 for x1, x2 in zip(start_grid_coord, end_grid_coord)]))
        return cost

    def ComputeHeuristicCost(self, start_id, goal_id):
        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        # start_grid_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        # end_grid_coord = self.discrete_env.NodeIdToGridCoord(goal_id)

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        cost = numpy.linalg.norm(numpy.array(end_config) - numpy.array(start_config))

        # Euclidean Distance (Direction)
        #cost = numpy.sqrt(sum([(x1 - x2)**2 for x1, x2 in zip(start_grid_coord, end_grid_coord)]))
        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

        
    def checkCollision(self, start_config, end_config):
        no_samples = 10
        T = self.robot.GetTransform()
        for xpp, ypp in zip(numpy.linspace(start_config[0], end_config[0], no_samples), numpy.linspace(start_config[1], end_config[1], no_samples)):
            T_new = T
            T_new[0][3] = xpp
            T_new[1][3] = ypp
            self.robot.SetTransform(T_new)
            if self.robot.GetEnv().CheckCollision(self.robot, self.table):
                return True
        return False

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        while True:
          config = [(random.random() - 0.5)*(u - l) + 0.5*(u + l) for u, l in zip(upper_limits,lower_limits)]
          robot_pose = numpy.array([[ 0, 0, 0, config[0]], 
                                  [0, 0,  0, config[1]], 
                                  [ 0, 0,  0, 0], 
                                  [ 0, 0,  0, 1]])
          with self.robot.GetEnv():
            self.robot.SetTransform(robot_pose)
          c3 = self.robot.GetEnv().CheckCollision(self.robot) #condition 3: robot is in collision
          if not c3: break          
        return numpy.array(config)
    def InLimits(self,config):
        lower_limits, upper_limits = self.boundary_limits
        return sum([not (lower_limits[i] <= config[i] <= upper_limits[i]) for i in range(len(lower_limits))]) == 0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        while True:
          config = [(random.random() - 0.5)*(u - l) + 0.5*(u + l) for u, l in zip(upper_limits,lower_limits)]
          robot_pose = numpy.array([[ 0, 0, 0, config[0]], 
                                  [0, 0,  0, config[1]], 
                                  [ 0, 0,  0, 0], 
                                  [ 0, 0,  0, 1]])
          with self.robot.GetEnv():
            self.robot.SetTransform(robot_pose)
          c3 = self.robot.GetEnv().CheckCollision(self.robot) #condition 3: robot is in collision
          if not c3: break          
        return numpy.array(config)

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        stepSize = 0.01
        numOfInterp = int(self.ComputeDistance(start_config, end_config)/stepSize)
        
        x = numpy.linspace(start_config[0], end_config[0], numOfInterp)
        yinterp = numpy.linspace(start_config[1], end_config[1], numOfInterp)

        for i in range(numOfInterp):
            
            check = self.CollisionChecker([x[i], yinterp[i]])

            if check == False:
                extend_config = numpy.array([x[i], yinterp[i]])
            else:
                break

        if i == 1 or i == 0:
            return None
        else :
            return extend_config
    
    def Extend_max(self, start_config, end_config, max_extend):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        
        plan_step_size = 0.05
        unit_vec = end_config - start_config #[x - y for x, y in zip(end_config,start_config)]
        dist = numpy.linalg.norm(unit_vec)
        unit_vec = [i / dist for i in unit_vec]
        count = 0
        curr_config = start_config
        c3 = False
        while True:
          #print count
          c1 = count*plan_step_size < dist  #condition 1: end_config is not  reached
          c2 =  count*plan_step_size < max_extend #condition 2: within boundaries

          if  not c1: return end_config
          if not c2: return numpy.array(curr_config) - plan_step_size*numpy.array(unit_vec)
          c3 = not self.CollisionChecker(curr_config) #condition 3: robot is not in collision
          if not c3: break
          count += 1
          curr_config = [ x + count*plan_step_size*y for x,y in zip(start_config, unit_vec)]
        #print curr_config, start_config  
        if all([i == j for i,j in zip(curr_config,start_config)]): return None
        curr_config = [ x - plan_step_size*y for x,y in zip(start_config, unit_vec)]
        if all([i == j for i,j in zip(curr_config,start_config)]): return None
        return numpy.array(curr_config)

    def CollisionChecker(self, config):
        config_pose = numpy.array([[ 1, 0,  0, config[0]], 
                                   [ 0, 1,  0, config[1]], 
                                   [ 0, 0,  1, 0], 
                                   [ 0, 0,  0, 1]])

        with self.robot.GetEnv():
            self.robot.SetTransform(config_pose)

        check = self.robot.GetEnv().CheckCollision(self.robot)
        return check

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        tstart = time.time()

        while ( (time.time() - tstart) < timeout ):


            x1, x2 = sorted(random.sample(range(len(path)), 2))


            #x1 = int(numpy.random.uniform(0,len(path)-1))
            #x2 = int(numpy.random.uniform(x1+1,len(path)-1))
            start_config = path[x1]
            end_config = path[x2]

            #print (x1,x2)
            #print len(path)
            #print path

            extend_config = self.Extend(start_config, end_config)

            if extend_config != None:
                if all(extend_config ==  end_config):
                    
                    #print "successful connection"
                    new_path = []
                    for i in range(0,x1+1):
                        new_path.append(path[i])
                    for i in range(x2,len(path)):
                        new_path.append(path[i])
                    path = new_path
 
      
        return path

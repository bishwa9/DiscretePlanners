import numpy
from DiscreteEnvironment import DiscreteEnvironment
import time

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):
        n = 1
        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        grid_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        increments = [n, -1*n] #basically get increment by +n then -n in each dimension
        for i in range(self.discrete_env.dimension): #iterate through each dimension (4-connected)
            for j in range(len(increments)): #apply the each inc on each dimension
                neighbor = list(grid_coord)
                neighbor[i] += increments[j] #ith joint with jth increment
                neighbor_id = self.discrete_env.GridCoordToNodeId(neighbor)
                successors.append(neighbor_id)
        return successors

    def ComputeDistance(self, start_id, end_id):

        # dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        cost = numpy.linalg.norm(numpy.array(end_config) - numpy.array(start_config))
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

    def checkCollision(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        start_config = numpy.array(start_config)
        end_config = numpy.array(end_config)
        d = 0
        delta_d = 0.05
        xy_ = numpy.ones((1,len(self.robot.GetActiveDOFIndices())))
        while d <= 1:
            # print d
            xy = [numpy.add( (1-d)*start_config, d*end_config )]
            # print d*end_config
            xy_ = numpy.append(xy_, xy, axis=0)
            d += delta_d
            if( self.collision_pt( xy[0] ) ):
                return True
        return False

    def collision_pt(self, dof_values):
        # ensure pt is not colliding with robot or table
        with self.robot.GetEnv():
            self.robot.SetDOFValues(dof_values, self.robot.GetActiveDOFIndices(), checklimits=True)
        return self.robot.GetEnv().CheckCollision(self.robot,self.table) or self.robot.GetEnv().CheckCollision(self.robot,self.robot)

    def ComputeConfigDistance(self, start_config, end_config):
        ConfigDist = numpy.linalg.norm(end_config - start_config)
        return ConfigDist

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        #
        # TODO: Generate and return a random configuration
        #

        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        
        while True:
           
            config = numpy.random.uniform(lower_limits, upper_limits)
            
            check = self.CollisionChecker(config)

            if check == False:
                break
        
        return numpy.array(config)

    def CollisionChecker(self, config):

        with self.robot.GetEnv():
            self.robot.SetDOFValues(config, self.robot.GetActiveDOFIndices())

        # collision check: when there is collision, return true
        check_1 = self.robot.GetEnv().CheckCollision(self.robot) 

        # self collision check: when self collision, return true
        check_2 = self.robot.CheckSelfCollision()

        # we want both to be false
        check = check_1 or check_2
        
        return check

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        
        stepSize = 0.1
        numOfInterp = int(self.ComputeDistance(start_config, end_config)/stepSize)
        #print "numOfInterp", numOfInterp
        interp_config = numpy.zeros(len(start_config))
        i=0
        
        for i in range(numOfInterp):
            for j in range(len(start_config)):
                interp_config[j] = numpy.linspace(start_config[j], end_config[j], numOfInterp)[i]

            check = self.CollisionChecker(interp_config)

            if check == False:
                extend_config = numpy.array(interp_config)
            else:
                break

        if i == 1 or i == 0:
            #print "can't extend"
            return None
        else:
            #print "---- extend ----, i-->", i 
            
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
        
    def ShortenPath(self, path, timeout=5.0):
       
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
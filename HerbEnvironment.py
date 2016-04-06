import numpy
from DiscreteEnvironment import DiscreteEnvironment

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
        num = self.discrete_env.dimension*2
        dimension_to_inc = 0
        increments = [n, -1*n] #basically get increment by +n then -n in each dimension
        for i in range(self.discrete_env.dimension): #iterate through each dimension (4-connected)
            for j in range(len(increments)): #apply the each inc on each dimension
                neighbor = list(grid_coord)
                neighbor[i] += increments[j] #ith joint with jth increment
                if neighbor[i] < self.lower_limits[i]:
                    neighbor[i] = self.lower_limits[i]

                if neighbor[i] > self.upper_limits[i]:
                    neighbor[i] = self.upper_limits[i]

                neighbor_id = self.discrete_env.GridCoordToNodeId(neighbor)
                successors.append(neighbor_id)
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_grid_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        end_grid_coord = self.discrete_env.NodeIdToGridCoord(end_id)

        # Manhattan Distance
        dist = sum([abs(x1 - x2)] for x1, x2 in zip(start_grid_coord, end_grid_coord))
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        start_grid_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        end_grid_coord = self.discrete_env.NodeIdToGridCoord(goal_id)

        # Euclidean Distance (Direction)
        cost = sum([(x1 - x2)**2 for x1, x2 in zip(start_grid_coord, end_grid_coord)])
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
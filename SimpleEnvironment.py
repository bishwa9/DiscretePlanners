import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
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
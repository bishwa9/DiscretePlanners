import heapq
import numpy

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):

        plan = []
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.InitializePlot(goal_config)

        start_config = int(self.planning_env.discrete_env.ConfigurationToNodeId(start_config))
        goal_config = int(self.planning_env.discrete_env.ConfigurationToNodeId(goal_config))

        # Reference
        # http://www.eecs.yorku.ca/course_archive/2006-07/W/2011/Notes/BFS_part2.pdf

        q = []
        index = 0

        no_vertices = reduce(lambda x, y: x * y, self.planning_env.discrete_env.num_cells, 1)

        flag = [False]*no_vertices
        pred = [-1]*no_vertices


        #heapq.heappush(q, (self.hueristic(current, neighbor), index, neighbor))
        #heapq.heappop(q)[-1]

        heapq.heappush(q, (0, index, start_config))
        index += 1
        flag[start_config] = True

        while len(q) > 0:
            current = heapq.heappop(q)[-1]
            #print "Current Node = ", current, "Neighbours = ", map(int, self.planning_env.GetSuccessors(current))
            for neighbor in map(int, self.planning_env.GetSuccessors(current)):
                # neighbor = self.planning_env.discrete_env.NodeIdToGridCoord(neighbor)
                # Get the current transform
                T = self.planning_env.robot.GetTransform()
                start_config = self.planning_env.discrete_env.NodeIdToConfiguration(current)
                end_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbor)
                # print "Neighbour = ", neighbor, "Configuration = ", end_config
                if not flag[neighbor] and not self.checkCollision(start_config, end_config):
                    flag[neighbor] = True
                    pred[neighbor] = current
                    # Plotting function
                    if self.visualize:
                        self.planning_env.PlotEdge(start_config, end_config)

                    heapq.heappush(q, (self.hueristic(self.planning_env.discrete_env.NodeIdToConfiguration(goal_config),
                                                      self.planning_env.discrete_env.NodeIdToConfiguration(neighbor)),
                                                      index, neighbor))
                    index += 1
                    if neighbor == goal_config:
                        root = neighbor
                        while root != -1:
                            plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(root))
                            print root
                            root = pred[root]
                        print "Final Path is = ",  plan[::-1]
                        return plan[::-1]

    def checkCollision(self, start_config, end_config):
        no_samples = 10
        T = self.planning_env.robot.GetTransform()
        for xpp, ypp in zip(numpy.linspace(start_config[0], end_config[0], no_samples), numpy.linspace(start_config[1], end_config[1], no_samples)):
            T_new = T
            T_new[0][3] = xpp
            T_new[1][3] = ypp
            self.planning_env.robot.SetTransform(T_new)
            if self.planning_env.robot.GetEnv().CheckCollision(self.planning_env.robot, self.planning_env.table):
                print "In collision"
                return True
        return False

    def hueristic(self, start_config, end_config):
        dist = sum([abs(x1 - x2) for x1, x2 in zip(start_config, end_config)])
        return dist
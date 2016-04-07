import heapq
import numpy
import time

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
        # print start_config
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.InitializePlot(goal_config)

        start_config_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(start_config))
        goal_config_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(goal_config))
        begin_config_id = start_config_id
        # Reference
        # http://www.eecs.yorku.ca/course_archive/2006-07/W/2011/Notes/BFS_part2.pdf

        q = []
        index = 0

        no_vertices = reduce(lambda x, y: x * y, self.planning_env.discrete_env.num_cells, 1)

        #visited = [False]*no_vertices  #visited
        parent = dict() #[-1]*no_vertices     #parent
        dist_from_start = dict()


        #heapq.heappush(q, (self.hueristic(current, neighbor), index, neighbor))
        #heapq.heappop(q)[-1]
        heuristic_ = self.planning_env.ComputeHeuristicCost(goal_config_id,start_config_id)
        heapq.heappush(q, (0, index, start_config_id))
        index += 1
        parent[start_config_id] = -1
        dist_from_start[start_config_id] = 0

        while len(q) > 0:
            current = heapq.heappop(q)[-1]
            dist_till_current = dist_from_start[current]
            #print "Current Node = ", current, "Neighbours = ", map(int, self.planning_env.GetSuccessors(current))
            for neighbor_id in map(int, self.planning_env.GetSuccessors(current)):
                #print neighbor_id
                # neighbor = self.planning_env.discrete_env.NodeIdToGridCoord(neighbor)
                # Get the current transform
                T = self.planning_env.robot.GetTransform()
                start_config = self.planning_env.discrete_env.NodeIdToConfiguration(current)
                end_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbor_id)
                # print start_config
                # time.sleep(10)
                #print "Neighbour = ", neighbor_id, "Configuration = ", end_config
                if neighbor_id not in parent and not self.planning_env.checkCollision(start_config, end_config):
                    parent[neighbor_id] = current
                    # Plotting function
                    if self.visualize:
                        self.planning_env.PlotEdge(start_config, end_config)
                    heuristic_ = self.planning_env.ComputeHeuristicCost(goal_config_id,neighbor_id)
                    cost_till = self.planning_env.ComputeDistance(current, neighbor_id) + dist_till_current
                    dist_from_start[neighbor_id] = cost_till
                    heapq.heappush(q, (heuristic_+cost_till/10.0, index, neighbor_id))
                    index += 1
                    #print len(q)
                    #print heuristic_
                    if neighbor_id == goal_config_id:
                        #print "Done"
                        root = neighbor_id
                        path_len = 0
                        #print root
                        while root != -1:
                            plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(root))
                            path_len += self.planning_env.ComputeDistance(root, parent[root])
                            root = parent[root]
                        #print "Final Path is = ",  plan[::-1]
                        print "Path Length", path_len
                        return plan[::-1]
        print "Done nothing"

    def hueristic(self, start_config, end_config):
        dist = sum([abs(x1 - x2) for x1, x2 in zip(start_config, end_config)])
        return dist
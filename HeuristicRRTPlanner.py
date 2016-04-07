import numpy
from RRTTree import RRTTree


class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
#         plan.append(start_config)
#         plan.append(goal_config)
        
        #REFERENCE: https://www.personalrobotics.ri.cmu.edu/files/courses/papers/Urmson03-hrrt.pdf
        #BUILD RRT
        #
        #while goal config isn't part of T:
        #
        #   (rand config, rand near) <- SELECT NODE(T)
        #   Extend(T, rand config, near config)
        #
        #Return T
        #
        #SELECT NODE(T)
        #
        #while r > quality measure
        #
        #   rand config <- RANDOM STATE()
        #   rand near <- NEAREST NEIGHBOR(rand config, T)
        #   quality measure <- 1 - (vertex cost - optimal cost)/(max cost - optimal cost)
        #   quality measure <- min(quality measure, T.prob_floor)
        #   r <- RANDOM VALUE
        #
        #Return (rand config, near config)
        #EXTEND(T, rand config, near config)
        #
        #if (NEW STATE(rand config, near config, new config, )) then
        #
        #   T.add_vertex(new config)
        #   T.add_edge(near config, new config, u new)
        #   CALCULATE COST(new config)
        #   T.max_cost <-max(cost new config, T.max_cost)
        #set probability of randomly sampling the goal config
        ## choose a new target config (p = 0.2 by default)
        ## 20% to be the goal config, 80% to be a random config 
        

        self.planning_env.SetGoalParameters(goal_config) 

        start_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(start_config))
        goal_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(goal_config))
        
        new_config = start_config
        new_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(new_config))
        #print new_id

        optimal_cost = self.planning_env.ComputeHeuristicCost(goal_id,start_id)
        max_cost = optimal_cost
        prob_floor = 0.2

        print "Growing the tree"

        while(self.planning_env.ComputeDistance(new_id, goal_id) > epsilon):
            
            r = numpy.random.rand()

            quality_measure = 0
            quality_measure = min(quality_measure, prob_floor)

            while(r>quality_measure): #SELECT NODE
                ## find the node in the tree that is nearest to the target config
                ## tree.vertices[vid]: the nearest node 
                ## vid: id of the node 
                target_prob = numpy.random.uniform(0, 1)

                if target_prob < self.planning_env.p:
                    target_config = goal_config
                else:
                    target_config = self.planning_env.GenerateRandomConfiguration()
                                    
                target_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(target_config))
                vertex_cost = self.planning_env.ComputeHeuristicCost(goal_id,target_id)
                vid, tree.vertices[vid] = tree.GetNearestVertex(target_config)
                quality_measure = 1 - (vertex_cost - optimal_cost)/(max_cost - optimal_cost)
                quality_measure = min(quality_measure, prob_floor)
                r = numpy.random.rand()

            ## extend the path between nearest node to the target config
            #extend_config = self.planning_env.Extend(tree.vertices[vid], target_config)
        
            extend_config = self.planning_env.Extend_max(tree.vertices[vid], target_config, 1)


            ## add the extend config to the tree
            if extend_config != None:
                # append extend config into the tree
                eid = tree.AddVertex(extend_config)

                # append the id mapping into the dictionary
                tree.AddEdge(vid, eid)

                extend_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(extend_config))
                new_cost = self.planning_env.ComputeHeuristicCost(goal_id,extend_id)
                max_cost = max(new_cost, max_cost)

                # update plot #
                if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                    self.planning_env.PlotEdge(tree.vertices[vid], extend_config)
  
            new_config = tree.vertices[-1]
            new_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(new_config))
            new_cost = self.planning_env.ComputeHeuristicCost(new_id,target_id)
            max_cost = max(new_cost,max_cost)
            print "... add new nodes" 

        # num of vertices
        self.numOfVert = len(tree.vertices)

        print "path found"

        # tree --> plan
        while eid != 0:
            plan.append(tree.vertices[eid])
            eid = tree.edges[eid]

        plan.append(start_config)
        
        plan.reverse()

        return plan


        
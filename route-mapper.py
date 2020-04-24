###################################################################
# Route Planner maps the best possible route between two intersections
# It uses a combination of Djikstra graph algorithm and A* algorithm
###################################################################
import math


class RoutePlanner:
    def __init__(self, m, start=None, goal=None):
        self.map = m
        self.start = start
        self.goal = goal
        # only init following params if map, start and goal are not Nonw
        if self.start is not None and self.goal is not None and self.map is not None:
            self.g_val_dict = dict()
            self.f_val_dict = dict()
            self.explored = set()
            self.frontier = set()
            self.best_origin_dict = {self.start: self.start}
            self.path = {(self.start, self.start): [self.start]}

    def init_g_val_dict(self):
        # g_val_dict maps node:actual cost of reaching that node from the start
        for intersection in self.map.intersections.keys():
            if intersection == self.start:
                # getting from start to start should have cost = 0
                self.g_val_dict[intersection] = 0
            else:
                self.g_val_dict[intersection] = float('inf')

    def init_f_val_dict(self):
        # f_val_dict maps node:total cost of going from start to goal via that node, based on
        # actual cost g and heuristic value H
        for intersection in self.map.intersections.keys():
            if intersection == self.start:
                # getting from start to start should have cost = 0
                self.f_val_dict[intersection] = self.get_euclidean_dist(self.start, self.goal)
            else:
                self.f_val_dict[intersection] = float('inf')

    def get_euclidean_dist(self, node1, node2):
        # simple straight line distance between x1,y1 and x2,y2
        return math.sqrt((self.map.intersections[node1][0] - self.map.intersections[node2][0]) ** 2
                         + (self.map.intersections[node1][1] - self.map.intersections[node2][1]) ** 2)

    def get_g_val_node(self, node):
        return self.g_val_dict[node]

    def get_f_val_node(self, node):
        # f_val = g_val + H where g_val cost of path from start to node,
        #         and  H is heuristic distance from that node to goal
        return self.get_g_val_node(node) + self.get_euclidean_dist(node, self.goal)

    def get_neighbour_nodes(self, node):
        # Returns neighbours for the given node
        return self.map.roads[node]

    def get_closest_node_from_frontier(self):
        # check frontier and return next to visit node depending on min f_val value
        # sort the f_val dict items based on values
        pres_node, min_f_val = sorted(self.f_val_dict.items(), key=lambda x: x[1])[0]
        return pres_node

    def reverse_back_to_start(self):
        # after goal is reached, formulate the final path
        # by tracking all best origin nodes back till start
        current = self.goal
        final_path = []
        while current != self.start:
            final_path.append(current)
            current = self.best_origin_dict[current]
        # append the start
        final_path.append(self.start)
        # reverse it to depict path from start --> goal
        final_path.reverse()
        return final_path

    def search_route(self):
        # This function follows A* algo to move from start node towards goal node, examining each node in frontier
        # deciding on actual distance from start + heuristic distance from the goal

        # put start node in frontier set
        self.frontier.add(self.start)

        # loop until either all nodes from frontier set are exhausted or the goal is reached
        while len(self.frontier) > 0:
            # based of f_val get the next node in frontier to be examined
            present_node = self.get_closest_node_from_frontier()

            if present_node == self.goal:
                # reached goal node, reverse back to the start
                return self.reverse_back_to_start()
            else:
                # remove it from frontier, add it to explored and reset its f_val
                self.frontier.remove(present_node)
                self.f_val_dict[present_node] = float('inf')
                self.explored.add(present_node)

            # search all the neighbours of the currently examined node
            for neighbour in self.get_neighbour_nodes(present_node):
                # do not go to the already explored
                if neighbour in self.explored:
                    continue

                # add the neighbour to frontier
                if neighbour not in self.frontier:
                    self.frontier.add(neighbour)

                # decision to choose best neighbour node to move to =
                # current dist from start till present node + heuristic dist to neighbour
                best_distance_to_neighbour = self.get_g_val_node(present_node) \
                                         + self.get_euclidean_dist(present_node, neighbour)

                if best_distance_to_neighbour <= self.get_g_val_node(neighbour):
                    # if we are in right direction
                    # update this as neighbour's new g_val, f_val and best_origin
                    self.g_val_dict[neighbour] = best_distance_to_neighbour
                    self.f_val_dict[neighbour] = self.get_f_val_node(neighbour)
                    self.best_origin_dict[neighbour] = present_node
                else:
                    # else simply continue to examine next neighbour
                    continue

        print(f'No path could be found between {self.start} and {self.goal}')
        return []

    def get_best_route(self):
        # This function checks if start, goal already exist in path dictionary
        # then simply return that or else find the best route
        if (self.start, self.goal) in self.path:
            return self.path[(self.start, self.goal)]
        else:
            best_path = self.search_route()
            if len(best_path) != 0:
                self.path[(self.start, self.goal)] = best_path
            return best_path


class Map:
    def __init__(self):
        self.intersections = {}
        self.roads = []

    def set_fields(self, intersect, roads):
        self.roads = roads
        self.intersections = intersect


roads = [[7, 6, 5],
 [4, 3, 2],
 [4, 3, 1],
 [5, 4, 1, 2],
 [1, 2, 3],
 [7, 0, 3],
 [0],
 [0, 5],
 [9],
 [8]]

intersections = {0: [0.7798606835438107, 0.6922727646627362],
 1: [0.7647837074641568, 0.3252670836724646],
 2: [0.7155217893995438, 0.20026498027300055],
 3: [0.7076566826610747, 0.3278339270610988],
 4: [0.8325506249953353, 0.02310946309985762],
 5: [0.49016747075266875, 0.5464878695400415],
 6: [0.8820353070895344, 0.6791919587749445],
 7: [0.46247219371675075, 0.6258061621642713],
 8: [0.11622158839385677, 0.11236327488812581],
 9: [0.1285377678230034, 0.3285840695698353]}

M = Map()
M.set_fields(intersections, roads)

def shortest_path(M,start,goal):
    print("shortest path called")
    RouteObj = RoutePlanner(M,start,goal)
    RouteObj.init_f_val_dict()
    RouteObj.init_g_val_dict()
    return RouteObj.get_best_route()

print(shortest_path(M, 1, 0))

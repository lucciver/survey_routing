"""
Knit Anemia Route Optimisation
Version : 4
Description : 
    implemented the time window constraints for each villages
Shorcomings :
    while formulator defines its routes , if there is a time constraints , then there will be a lot of wastage.
Errors :
    NONE
"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


###########################
# Problem Data Definition #
###########################
class Vehicle():
    """Stores the property of a vehicle"""
    def __init__(self):
        """Initializes the vehicle properties"""
        self._capacity = 12000

    @property
    def capacity(self):
        """Gets vehicle capacity"""
        return self._capacity

class DataProblem():
    """Stores the data for the problem"""
    def __init__(self):
        """Initializes the data for the problem"""
        self._vehicle = Vehicle()
        #self._num_vehicles = 2

        self._depot = 0 

        self._req_vehicles = 0
        ###              0  1      2    3     4     5     6     7     8    9     10    11    12
        self._demands = [0, 1000, 3000, 3000, 2000, 8000, 1130, 3000, 560, 4000, 2500, 8000, 1000]

        self._distances = [[  0, 2451,  713, 1018, 1631, 1374, 2408,  213, 2571,  875, 1420, 2145, 1972], # New York
                          [2451,    0, 1745, 1524,  831, 1240,  959, 2596,  403, 1589, 1374,  357,  579], # Los Angeles
                          [ 713, 1745,    0,  355,  920,  803, 1737,  851, 1858,  262,  940, 1453, 1260], # Chicago
                          [1018, 1524,  355,    0,  700,  862, 1395, 1123, 1584,  466, 1056, 1280,  987], # Minneapolis
                          [1631,  831,  920,  700,    0,  663, 1021, 1769,  949,  796,  879,  586,  371], # Denver
                          [1374, 1240,  803,  862,  663,    0, 1681, 1551, 1765,  547,  225,  887,  999], # Dallas
                          [2408,  959, 1737, 1395, 1021, 1681,    0, 2493,  678, 1724, 1891, 1114,  701], # Seattle
                          [ 213, 2596,  851, 1123, 1769, 1551, 2493,    0, 2699, 1038, 1605, 2300, 2099], # Boston
                          [2571,  403, 1858, 1584,  949, 1765,  678, 2699,    0, 1744, 1645,  653,  600], # San Francisco
                          [ 875, 1589,  262,  466,  796,  547, 1724, 1038, 1744,    0,  679, 1272, 1162], # St. Louis
                          [1420, 1374,  940, 1056,  879,  225, 1891, 1605, 1645,  679,    0, 1017, 1200], # Houston
                          [2145,  357, 1453, 1280,  586,  887, 1114, 2300,  653, 1272, 1017,    0,  504], # Phoenix
                          [1972,  579, 1260,  987,  371,  999,  701, 2099,  600, 1162, 1200,  504,   0]] # Salt Lake City

        self._time_windows = \
            [(0, 12000),
             (4000, 12000), (6000, 12000), # 1, 2
             (0, 12000), (0, 6000), # 3, 4
             (0, 12000), (8000, 12000), # 5, 6
             (4000, 12000), (6000, 12000), # 7, 8
             (0, 12000), (6000, 12000), # 9, 10
             (0, 12000), (0, 12000)] # 11, 12

        
    @property        
    def vehicle(self):
        """Gets a vehicle"""
        return self._vehicle

    @property
    def demands(self):
        """Gets demands at each location"""
        return self._demands

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self.demands)

    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return len(self.demands)-1

    @property
    def req_vehicles(self):
        """Gets number of vehicles"""
        return self._req_vehicles

    @property       
    def depot(self):
        """Gets depot location index"""
        return self._depot

    @property
    def distances(self):
        """Gets distance between each pair of locations"""
        return self._distances

    @property
    def time_windows(self):
        """Gets the time window of a particular location"""
        return self._time_windows

    def formulator(self):
        print('formulator\n')
        for x in xrange(0, len(self._demands)):
            if ((self._demands[x] + 2*self._distances[self._depot][x]) > self._vehicle.capacity):
                while ((self._demands[x] + 2*self._distances[self._depot][x]) > self._vehicle.capacity) :
                    self._req_vehicles += 1
                    self._demands[x] -= (self._vehicle.capacity - 2*self._distances[self._depot][x])
                    print('Route of Vehilce {}\n'.format(self._req_vehicles))
                    print('{0} Load({1}) -> {2} Load({3}) -> {4} Load({5})\n'.format(self._depot, 0,  x, self._vehicle.capacity - 2*self._distances[self._depot][x], self._depot, self._vehicle.capacity))


#######################
# Problem Constraints #
#######################

class CreateDistanceEvaluator(object): # pylint: disable=too-few-public-methods
    """Creates callback to return distance between points."""
    def __init__(self, data):
        """Initializes the distance matrix."""
        self._distances = data.distances

    def distance_evaluator(self, from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return self._distances[from_node][to_node]

class CreateDemandEvaluator(object): # pylint: disable=too-few-public-methods
    """Creates callback to get demands at each location."""
    def __init__(self, data):
        """Initializes the demand array."""
        self._demands = data.demands

    def demand_evaluator(self, from_node, to_node):
        """Returns the demand of the current node"""
        del to_node
        return self._demands[from_node]

class CreateTimeEvaluator(object):
    """Creates callback to get total times between locations."""
    @staticmethod
    def service_time(data, node):
        """Gets the service time for the specified location."""
        return data.demands[node]

    @staticmethod
    def travel_time(data, from_node, to_node):
        """Gets the travel times between two locations."""
        if from_node == to_node:
            travel_time = 0
        else:
            travel_time = data.distances[from_node][to_node]
        return travel_time

    def __init__(self, data):
        """Initializes the total time matrix."""
        self._total_time = {}
        # precompute total time to have time callback in O(1)
        for from_node in xrange(data.num_locations):
            self._total_time[from_node] = {}
            for to_node in xrange(data.num_locations):
                if from_node == to_node:
                    self._total_time[from_node][to_node] = 0
                else:
                    self._total_time[from_node][to_node] = int(
                        self.service_time(data, from_node) +
                        self.travel_time(data, from_node, to_node))

    def time_evaluator(self, from_node, to_node):
        """Returns the total time between the two nodes"""
        return self._total_time[from_node][to_node]

#start of differance
def add_capacity_constraints(routing, data, time_evaluator):
    """Adds capacity constraint"""
    capacity = "Capacity"
    routing.AddDimension(
        time_evaluator,
        data.vehicle.capacity, # null capacity slack
        data.vehicle.capacity, # vehicle maximum capacity
        True, # start cumul to zero
        capacity)


def implement_time_constraint(routing, data, time_evaluator):
    capacity = "Capacity"
    time_dimension = routing.GetDimensionOrDie(capacity)
    for location_idx, time_window in enumerate(data.time_windows):
        time_dimension.CumulVar(location_idx).SetRange(time_window[0], time_window[1])

###########
# Printer #
###########
class ConsolePrinter():
    """Print solution to console"""
    def __init__(self, data, routing, assignment):
        """Initializes the printer"""
        self._data = data
        self._routing = routing
        self._assignment = assignment

    @property
    def data(self):
        """Gets problem data"""
        return self._data

    @property
    def routing(self):
        """Gets routing model"""
        return self._routing

    @property
    def assignment(self):
        """Gets routing model"""
        return self._assignment

    def print(self):
        """Prints assignment on console"""
        # Inspect solution.
        print('after optimization\n')
        capacity_dimension = self.routing.GetDimensionOrDie('Capacity')
        total_time = 0
        req_vehicles = self.data.req_vehicles
        for vehicle_id in xrange(self.data.num_vehicles):
            index = self.routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {0}:\n'.format(req_vehicles+1)
            route_time = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing.IndexToNode(index)
                next_node_index = self.routing.IndexToNode(
                    self.assignment.Value(self.routing.NextVar(index)))
                #route_time += self.data.distances[node_index][next_node_index]
                time_var = capacity_dimension.CumulVar(index)
                route_time = self.assignment.Value(time_var)
                plan_output += ' {0} Load({1}) ->'.format(node_index, route_time)
                index = self.assignment.Value(self.routing.NextVar(index))

            node_index = self.routing.IndexToNode(index)
            time_var = capacity_dimension.CumulVar(index)
            route_time = self.assignment.Value(time_var)
            plan_output += ' {0} Load({1})\n'.format(node_index, route_time)
            if (route_time > 0) :
                req_vehicles += 1
                print(plan_output)
        print('No Of Required Vehicles are {0}'.format(req_vehicles))

########
# Main #
########
def main():
    """Entry point of the program"""
    # Instantiate the data problem.  
    if_time_windows = input("")
    data = DataProblem()
    data.formulator()
    # Create Routing Model
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    # Define weight of each edge
    time_evaluator = CreateTimeEvaluator(data).time_evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(time_evaluator)
    # Add Capacity constraint
    add_capacity_constraints(routing, data, time_evaluator)
    #if the time constraint is to be implemented or not
    implement_time_constraint(routing, data, time_evaluator)
    # Add Time Window constraint
    #time_evaluator = CreateTimeEvaluator(data).time_evaluator
    #add_time_window_constraints(routing, data, time_evaluator)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    printer = ConsolePrinter(data, routing, assignment)
    printer.print()

if __name__ == '__main__':
    main()
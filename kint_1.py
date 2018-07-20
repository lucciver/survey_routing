"""
Knit Anemia Route Optimisation
Version : 2
Description : 
    formulator dedictes vehicles to specific villags and reduces the demands inti optimizable demands
    considers the time from village to village and time to be spend in each village and displays the optimal route to be followed
Shorcomings :
    data is hardcoded and not read form excel sheet
Errors :
    NONE
"""


from __future__ import print_function
from six.moves import xrange
import sys
sys.path.append("F:\\projects\\ameen\\ortools")
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import csv


###########################
# Problem Data Definition #
###########################
class Vehicle():
    """Stores the property of a vehicle"""
    def __init__(self):
        """Initializes the vehicle properties"""
        vehicle_capacity = input()
        self._capacity = int(vehicle_capacity)


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

        #read the demands as a list of lists
        with open('test_demands1.csv', 'rU') as f:
            list = [i[5] for i in csv.reader(f)]
        list.pop(0)
        self._demands = [int(i) for i in list]

        with open('travel_times.csv', 'rU') as f:
            list = [i for i in csv.reader(f)]
        list.pop(0)
        #loop for the size of list
        self._distances = [[int(j) for j in i] for i in list]
         

        
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

    def printDemands(self):
        for i in range(len(self._demands)): 
            print('{0} , '.format(self._demands[i]))

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

#capacity constraint getting added 
def add_capacity_constraints(routing, data, time_evaluator):
    """Adds capacity constraint"""
    capacity = "Capacity"
    routing.AddDimension(
        time_evaluator,
        0, # null capacity slack
        data.vehicle.capacity, # vehicle maximum capacity
        True, # start cumul to zero
        capacity)

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
    data = DataProblem()
    print('hello')
    data.formulator()
    print('\n')
    data.printDemands()
    # Create Routing Model
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    # Define weight of each edge
    time_evaluator = CreateTimeEvaluator(data).time_evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(time_evaluator)
    # Add Capacity constraint
    add_capacity_constraints(routing, data, time_evaluator)
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
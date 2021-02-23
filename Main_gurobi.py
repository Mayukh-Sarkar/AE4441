# -*- coding: utf-8 -*-
"""
Created on Sun Dec 13 23:10:05 2020

@author: M.Dheer M.Sarkar N.Pauly
"""
# Import the necessary packages
import gurobipy as gp  # don't forget to download gurobi with academic license
import numpy as np
import pandas as pd
from matplotlib import pyplot
import warnings
import math as mth
import xlwt
from xlwt import Workbook
import os.path
import sys
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
# from mpl_toolkits.basemap import basemap

# Class definition
class DroneScanningOpt:
    def __init__(self, area_shape, amount_of_drones_used, location_charging_points, minimum_charge,
                 name, logging_obj, scanning_time, scanning_consumption, plotting=True, area_length=3, area_width=3,
                 resolution=1):
        # Define instance variables
        self.name = name  # Name of the excel sheet in which it saves
        self.wb = logging_obj  # Name of the object so everything is logged in the same file
        self.area_shape = area_shape  # Which shape is chosen by the user
        self.area_width = area_width  # Defines the width in case a rectangle is chosen
        self.area_length = area_length  # Defines the length in case a rectangle is chosen
        self.resolution = resolution  # Resolution of the points in case a rectangle is chosen
        self.drone_amount = amount_of_drones_used  # Amount of the drones used to run the optimization
        self.gm = gp.Model('Modelname')  # Name of the gurobi model
        self.area_coord = []  # List in list, [node number, x_location node, y_location node]
        self.i = 0  # i is the name of the node
        self.x_vars = {}  # Dictionary with the links, [node i, node j, drone k]
        self.result = []  # List with the output of the optimization
        self.q = {}  # Dictionary with charges, [node i, drone k]
        self.time = {}  # Dictionary with the time, [node i, drone k]
        self.M = 1000  # Big M value, slack
        self.value_list = []  # List in list which contains the possible nodes it can reach from another node
        self.new_value_list = []  # value_list but rearranged
        self.amount_of_links = 0  # Counter which counts the total amount of links (including all the drones)
        self.obj_time = 0  # Represents the time component of the objective function
        self.obj_dist = 0  # Represents the distance component of the objective function
        self.obj_charge = 0  # Represents the charge component of the objective function
        self.obj_function = 0  # Represents the objective function
        self.location_charg_point = location_charging_points  # List in list with the location of the charging points
        self.amount_of_charg_stat = len(location_charging_points)  # Amount of charging points used
        self.charge_scan = scanning_consumption  # Battery charge it takes to scan a landmark
        self.scan_time = scanning_time  # Time it takes to scan a landmark
        self.min_charge = minimum_charge  # Minimum charge it needs to have at all time
        self.charge_nodes = [0]  # Node 0 is always a charge node
        self.charge_constant = 1.43  # Battery consumption per kilometer in cruise
        self.time_constant = 0.857  # Time it takes to fly 1 kilometer in cruise
        self.charge_time = 1.32  # Calculated time it takes to charge 1 percent in minutes
        self.amount_visits_charg = 2  # Amount a drone can visit a charging point
        self.amount_of_landmarks = 0  # Amount of landmarks which need to be visited
        self.landing_takeoff_penalty = 8  # This value is chosen to account for the landing and takeoff time etc.
        self.plotting = plotting  # Regulates if plotting is on or not.
        self.custom_obj_value = 0

    ##############################
    # This method is used to create a coordinate list of the landmarks
    def area_discr(self):
        # This part aims to make a list of coordinates of the length and width.
        if self.area_shape == 'rectangle':
            # Discretizes the amount area into different coordinates
            for y in range(self.area_width * self.resolution):
                for x in range(self.area_length * self.resolution):
                    self.area_coord.append([self.i, x / self.resolution, y / self.resolution])
                    self.i += 1

        # Takes locations from an excel file and creates a list
        if self.area_shape == 'map_locations':
            df = pd.read_excel(r'points.xlsx', 'Sheet1')

            y = df['LAT'].tolist()
            x = df['LONG'].tolist()
            # Transforms the longitude and latitude to x and y locations
            if len(x) == len(y):
                self.amount_of_landmarks = len(x)
                for i in range(len(x)):
                    self.area_coord.append(
                        [self.i, round((abs(x[self.i] - x[0]) * 111.1), 2), round((abs(y[self.i] - y[0]) * 111.1), 2)])
                    self.i += 1
            else:
                warnings.warn("The x and y coordinates do not have the same amount")

        # Adds the coordinates of the charging stations
        for w in range(0, self.amount_visits_charg):
            for z in range(len(self.location_charg_point)):
                self.area_coord.append([self.i, self.location_charg_point[z][0], self.location_charg_point[z][1]])
                self.charge_nodes.append(self.i)
                self.i += 1
        self.location_charg_point.append([0, 0])

    ##############################
    # Calculates the distance between two nodes
    def link(self, node_i, node_j):
        dist = np.sqrt((self.area_coord[node_j][1] - self.area_coord[node_i][1]) ** 2 + (
                (self.area_coord[node_j][2]) - self.area_coord[node_i][2]) ** 2)
        return dist

    ##############################
    # Calculates the decision variables used in the gurobi model
    def dec_var(self):
        # Distance Variables to the nodes
        for k in range(self.drone_amount):
            self.value_list = []
            for i in range(len(self.area_coord)):
                lst = []
                for j in range(self.i):
                    if self.link(i, j) < 25 and i != j and self.i - len(self.location_charg_point) > j \
                            and self.i - len(self.location_charg_point) > i:
                        self.x_vars[i, j, k] = self.gm.addVar(ub=2, lb=0, vtype=gp.GRB.INTEGER,
                                                              name='Link' + str(i) + '-' + str(
                                                                  j) + '_' + 'Drone ' + str(k))
                        lst.append(j)
                        self.amount_of_links += 1
                        self.obj_dist += self.x_vars[i, j, k] * self.link(i, j)
                    elif i != j and self.i - len(self.location_charg_point) <= j \
                            or self.i - len(self.location_charg_point) <= i:
                        self.x_vars[i, j, k] = self.gm.addVar(ub=2, lb=0, vtype=gp.GRB.INTEGER,
                                                              name='Link' + str(i) + '-' + str(
                                                                  j) + '_' + 'Drone ' + str(k))
                        lst.append(j)
                        self.amount_of_links += 1
                        self.obj_dist += self.x_vars[i, j, k] * self.link(i, j)

                    self.gm.update()
                self.value_list.append(lst)
        # Charge Variables
        for k in range(self.drone_amount):
            for i in range(len(self.area_coord)):
                self.q[i, k] = self.gm.addVar(ub=100, vtype=gp.GRB.CONTINUOUS,
                                              name='Charge at node ' + str(i) + ' Drone ' + str(k))
                self.obj_charge += self.q[i, k]
        # Time Variables
        for k in range(self.drone_amount):
            for i in range(len(self.area_coord)):
                self.time[i, k] = self.gm.addVar(vtype=gp.GRB.CONTINUOUS,
                                                 name='Time at node ' + str(i) + ' Drone ' + str(k))
                self.obj_time += self.time[i, k]

    ##############################
    # Creates the objection function and adds it to the gurobi model.
    def create_obj_funct(self):
        c1 = 1
        c2 = -0.001
        c3 = 2.67
        #  Each part in the objective function has a different weight
        self.obj_function = c1 * self.obj_dist + c2 * self.obj_charge + c3 * self.obj_time
        self.gm.setObjective(self.obj_function, gp.GRB.MINIMIZE)

    ##############################
    # General constraints for a vehicle routing problem
    def general_constr(self):
        # Making sure if you enter a node, you leave the same node (flow continuity)
        for k in range(self.drone_amount):
            for h in range(0, self.i):
                self.gm.addConstr(gp.quicksum(self.x_vars[i, h, k] for i in self.value_list[h])
                                  - gp.quicksum(self.x_vars[h, j, k] for j in self.value_list[h]) == 0)

        # Making sure all the nodes are visited
        for i in range(0, self.i):
            if i not in self.charge_nodes:
                self.gm.addConstr(gp.quicksum(self.x_vars[i, j, k] for j in self.value_list[i]
                                              for k in range(self.drone_amount)) >= 1)

        # Making sure all the drones leave the home station
        for k in range(self.drone_amount):
            self.gm.addConstr(gp.quicksum(self.x_vars[0, j, k] for j in self.value_list[0]) == 1)

    ##############################
    # Constraints related to the battery level and consumption
    def charge_constr(self):
        # Set charge at a charging station equal to 100
        for k in range(self.drone_amount):
            for i in self.charge_nodes:
                # Charging station charge equals 100
                self.gm.addConstr(self.q[i, k] == 100)

        # Never go below a certain battery level
        for k in range(self.drone_amount):
            for i in range(0, self.i):
                self.gm.addConstr(self.q[i, k] >= self.min_charge)

        # Decrease the charge when moving from one node to another
        for k in range(self.drone_amount):
            for i in range(0, self.i):
                for j in self.value_list[i]:
                    if j not in self.charge_nodes:
                        self.gm.addConstr(self.q[i, k] - self.q[j, k] - self.charge_constant * self.link(i, j)
                                          * self.x_vars[i, j, k] - self.charge_scan + (
                                                      1 - self.x_vars[i, j, k]) * self.M >= 0)
                    if j in self.charge_nodes:
                        self.gm.addConstr(self.q[i, k] - self.charge_constant * self.link(i, j) * self.x_vars[i, j, k] +
                                          (1 - self.x_vars[i, j, k]) * self.M >= 0)

    ##############################
    # Constraints related to the time
    def time_constr(self):
        # Start at time equal to 0
        for k in range(self.drone_amount):
            self.gm.addConstr(self.time[0, k] == 0)

        # Decrease the time when moving from one node to another
        for k in range(self.drone_amount):
            for i in range(0, self.i):
                for j in self.value_list[i]:
                    if j not in self.charge_nodes:
                        self.gm.addConstr(self.time[i, k] - self.time[j, k] + self.time_constant * self.link(i, j)
                                          * self.x_vars[i, j, k] + self.scan_time - (
                                                      1 - self.x_vars[i, j, k]) * self.M <= 0)
                    if j in self.charge_nodes:
                        if j != 0:
                            self.gm.addConstr(self.time[i, k] - self.time[j, k] + self.time_constant * self.link(i, j)
                                              * self.x_vars[i, j, k] + self.charge_time * (100 - self.q[i, k]) +
                                              self.landing_takeoff_penalty - (1 - self.x_vars[i, j, k]) * self.M <= 0)

    ##############################
    # Plotting of the drone flight paths
    def plot(self, i, j, drone):
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        pyplot.plot([self.area_coord[i][1], self.area_coord[j][1]], [self.area_coord[i][2], self.area_coord[j][2]],
                    color=colors[drone])

    ##############################
    # Extracting values from the result list
    def get_values_from_result(self, inputt, which_drone):
        lst_links = []
        lst_charge = []
        lst_time = []
        if inputt == "links":
            for i in range(int(self.amount_of_links / self.drone_amount)):
                lst_links.append(self.result[(int(self.amount_of_links / self.drone_amount)) * which_drone + i])
            return lst_links
        if inputt == "charge":
            for i in range(self.amount_of_links, int(self.amount_of_links + len(self.area_coord))):
                lst_charge.append(round(self.result[int(i + (len(self.area_coord)) * which_drone)], 2))
            return lst_charge
        if inputt == "time":
            for i in range(self.amount_of_links,
                           int(self.amount_of_links + len(self.area_coord) + len(self.area_coord) / 2)):
                lst_time.append(round(self.result[int(i + len(self.area_coord) / 2 * which_drone)], 2))
            return lst_time

    ##############################
    # UNKNOWN
    def transfer(self):
        for i in range(len(self.value_list)):
            for j in range(len(self.value_list[i])):
                self.new_value_list.append([i, self.value_list[i][j]])

    ##############################
    # Constructs the plot
    def createplot(self):
        self.transfer()
        # How many different possibilities of links are there
        for j in range(self.drone_amount):
            for i in range(len(self.get_values_from_result("links", j))):
                if self.get_values_from_result("links", j)[i] == 1:
                    pass#self.plot(self.new_value_list[i][0], self.new_value_list[i][1], j)

        # Plots the nodes as dots
        print(self.location_charg_point)
        for i in range(self.i):
            pyplot.plot([self.area_coord[i][1]], self.area_coord[i][2], 'rp', markersize=1)
            for j in range(len(self.location_charg_point)):
                if self.location_charg_point[j][0] == 0 and  self.location_charg_point[j][1] == 0:
                    pyplot.plot(self.location_charg_point[j][0], self.location_charg_point[j][1], 'rp', markersize=15, color='red')
                else:
                    pyplot.plot(self.location_charg_point[j][0], self.location_charg_point[j][1], 'rp', markersize=15, color='coral')
            im = pyplot.imread("rainforest.jpg")
            # Get the range of the graph
            pyplot.imshow(im, extent=[-5, 22, -2, 26])
            pyplot.scatter(np.array(self.area_coord)[:, 1], np.array(self.area_coord)[:, 2])
            for j in range(self.drone_amount):
                n = self.get_values_from_result('charge', j)
                for i, txt in enumerate(n):
                    if txt != 100:
                        text_on_node = "Q: " + str(txt) + "%" + " D" + str(j) + " N" + str(i)
                        text_on_node = ''
                        
                        pyplot.annotate(text_on_node, (np.array(self.area_coord)[i, 1], np.array(self.area_coord)[i, 2])
                                        , color='white')
        #pyplot.title("Objective Value: " + str(round(self.gm.objVal, 1)) + " || " + "Run time: " +
           #          str(round(self.gm.RunTime, 2)) + 's || ' + "Amount of Drones: " + str(self.drone_amount))
        
    ##############################
    # Extracting values from the result list
    def get_obj_values(self, what):
        counter = 0
        total = 0
        if what == 'distance':
            for k in range(self.drone_amount):
                for i in range(0, self.i):
                    for j in self.value_list[i]:
                        total += self.result[counter] * self.link(i, j)
                        counter += 1
            return total
        elif what == 'time':
            lst = []
            lst_sum = []
            counter = 0
            for k in range(self.drone_amount):
                for i in range(len(self.area_coord)):
                    lst.append(self.result[int(self.amount_of_links + self.amount_of_landmarks * self.drone_amount +
                                               self.amount_of_charg_stat * self.amount_visits_charg * self.drone_amount
                                               + counter)])
                    counter += 1
                lst.sort()
            return lst[-1]
        else:
            print("Invalid input")

    ##############################
    #Saving the inputs and outputs in an excel sheet
    def save_data(self):
        self.name = self.wb.add_sheet(self.name)  # Add a sheet

        # Define different styles
        style_title = xlwt.easyxf('font: bold 1, color red;')
        style_massive_style = xlwt.easyxf('font: bold 1, color blue;')

        self.name.write(0, 0, 'INPUTS', style_massive_style)
        x_location_sec_title = 12
        if self.amount_of_landmarks + 3 > x_location_sec_title:
            x_location_sec_title = self.amount_of_landmarks + 3

        self.name.write(x_location_sec_title, 0, 'OUTPUTS', style_massive_style)

        ####################### Block 1
        self.name.write(1, 1, 'Model Parameters:', style_title)
        self.name.write(3, 1, 'Drone Amount')
        self.name.write(4, 1, 'Charging Constant')
        self.name.write(5, 1, 'Minimum Charge Level')
        self.name.write(6, 1, 'Charge Time')
        self.name.write(7, 1, 'Time Constant')
        self.name.write(8, 1, 'Amount it can visit a charging station')
        self.name.write(9, 1, 'Amount of Landmarks')
        self.name.write(3, 2, self.drone_amount)
        self.name.write(4, 2, self.charge_constant)
        self.name.write(5, 2, self.min_charge)
        self.name.write(6, 2, self.charge_time)
        self.name.write(7, 2, self.time_constant)
        self.name.write(8, 2, self.amount_visits_charg)

        ####################### Block 2
        self.name.write(1, 4, 'Charging Station Locations: ', style_title)
        for i in range(self.amount_of_charg_stat):
            self.name.write(i + 3, 4, 'Charge Location ' + str(i))
            self.name.write(i + 3, 5, str(self.location_charg_point[i]))

        ####################### Block 3
        self.name.write(1, 7, 'Landmark Locations: ', style_title)
        for i in range(self.amount_of_landmarks):
            self.name.write(i + 3, 7, 'Landmark Location ' + str(i))
            self.name.write(i + 3, 8, str('[' + str(self.area_coord[i][1]) + ', ' + str(self.area_coord[i][2]) + ']'))

        ####################### Block 4
        self.name.write(x_location_sec_title + 1, 1, 'Optimization specifics', style_title)
        self.name.write(x_location_sec_title + 2, 1, 'Run time')
        self.name.write(x_location_sec_title + 2, 2, self.gm.RunTime)
        self.name.write(x_location_sec_title + 2, 3, 'seconds')

        self.name.write(x_location_sec_title + 3, 1, 'Gap')
        self.name.write(x_location_sec_title + 3, 2, self.gm.MIPGap)
        self.name.write(x_location_sec_title + 3, 3, 'percentage')

        self.name.write(x_location_sec_title + 4, 1, 'Objective value')
        self.name.write(x_location_sec_title + 4, 2, self.gm.objVal)
        self.name.write(x_location_sec_title + 4, 3, '-')

        self.name.write(x_location_sec_title + 5, 1, 'Total Distance Flown by the Drones')
        self.name.write(x_location_sec_title + 5, 2, self.get_obj_values('distance'))
        self.name.write(x_location_sec_title + 5, 3, 'km')

        self.name.write(x_location_sec_title + 6, 1, 'Total Operation Time')
        self.name.write(x_location_sec_title + 6, 2, self.get_obj_values('time'))
        self.name.write(x_location_sec_title + 6, 3, 'minutes')

        x_location_sec_title += 7

        ####################### Block 5
        self.name.write(x_location_sec_title + 1, 1, 'Link Outputs', style_title)
        counter = 0
        for k in range(self.drone_amount):
            for i in range(0, self.i):
                for j in self.value_list[i]:
                    self.name.write(x_location_sec_title + counter + 2, 1, str(self.x_vars[(i, j, k)]))
                    self.name.write(x_location_sec_title + counter + 2, 2, str(self.result[counter]))
                    counter += 1

        ####################### Block 6
        self.name.write(x_location_sec_title + 1, 4, 'Charge Outputs', style_title)
        counter1 = 0
        for k in range(self.drone_amount):
            for i in range(len(self.area_coord)):
                self.name.write(x_location_sec_title + counter1 + 2, 4, str(self.q[(i, k)]))
                self.name.write(x_location_sec_title + counter1 + 2, 5, str(self.result[counter]))
                counter1 += 1
                counter += 1

        ####################### Block 7
        self.name.write(x_location_sec_title + 1, 7, 'Time Outputs', style_title)
        counter2 = 0
        for k in range(self.drone_amount):
            for i in range(len(self.area_coord)):
                self.name.write(x_location_sec_title + counter2 + 2, 7, str(self.time[(i, k)]))
                self.name.write(x_location_sec_title + counter2 + 2, 8, str(self.result[counter]))
                counter2 += 1
                counter += 1

        # Save to file
        self.wb.save('Output_Data.csv')

    ##############################
    # Gets the objective value from the optimization
   
    def get_objective_value(self, value):

        try:
            if value == 'runtime':
                return self.gm.RunTime
            elif value == 'objective':
                return self.custom_obj_value
            elif value == 'gap':
                return self.gm.MIPGap
            else:
                print("Wrong value given")
        except:
               
            if value == 'objective': 
                return 100000




    ##############################
    # Runs the complete optimization
    def run_optimization(self):
        self.area_discr()
        self.dec_var()

        self.general_constr()
        self.charge_constr()
        self.time_constr()
        self.create_obj_funct()
        self.gm.update()

        self.gm.write('test.lp')  # write to LP file
        self.gm.setParam('Timelimit', 50)  # Time limit in seconds
        #self.gm.setParam('MIPGap', .1)  # Set a gap at which it stops with 0.1 equal to 10 percent
        self.gm.optimize()
        try:
            self.gm.printAttr('X')
            self.result = self.gm.getAttr('X')
            self.custom_obj_value = self.gm.objVal
            if self.plotting:
                self.createplot()
                pyplot.show()
                self.save_data()
        except:
            print("Oops!", sys.exc_info()[0], "occurred.")
            print("INFEASIBLE MODEL")
            self.custom_obj_value = 7000


##############################
# This will only be executed when this file is ran, not if it is imported as a module.
if __name__ == "__main__":
    # Checks if the output file is in the directory
    if os.path.exists("Output_Data.csv"):
        pass
    # No file is detected and a new one is created
    else:
        f = open("Output_Data.csv", "x")
        f.close()
    wb = Workbook()  # Create Workbook instance called wb

    loop = False

#  Runs a loop with changing location of the charging point
if loop:
    fig = pyplot.figure()
    ax = fig.gca(projection='3d')
    x_range_charg = 25
    y_range_charg = 25
    gaps = []
    threedgraph = np.zeros((x_range_charg, y_range_charg))
    
    for i in range(x_range_charg):
        for j in range(y_range_charg):
            Model_loop = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=1,
                                          location_charging_points=[[i, j]], minimum_charge=10, scanning_time=15,
                                          scanning_consumption=2, name="map_location_run" +str(i) + str(j), logging_obj=wb,
                                          plotting=False)
            Model_loop.run_optimization()
            threedgraph[i][j] = (Model_loop.get_objective_value('objective'))

    gaps.append(Model_loop.get_objective_value('gap'))
    X = np.arange(x_range_charg)
    Y = np.arange(y_range_charg)
    avg_gap = sum(gaps)/len(gaps)
    X, Y = np.meshgrid(X, Y)
    surf = ax.plot_surface(X, Y, threedgraph, cmap=cm.coolwarm, linewidth=0, antialiased=False)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    fig.colorbar(surf, shrink=0.5, aspect=5)
    pyplot.show()
    print(threedgraph)
    print(avg_gap)
    
# Runs the model only once
else:
        cost_1 = []
        cost_2 = []
        cost_3 = []
        minimum_charge_1 = list(range(0,31))
        scanning_consumption_1 = list(range(0,31))
#         # for f in minimum_charge_1:
#         #     Model_1 = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=1,
#         #                         location_charging_points=[[5, 10]], minimum_charge= f, scanning_time=15,
#         #                       scanning_consumption= 2, name= str(f + 50), logging_obj=wb, plotting=False,
#         #                       area_length=3, area_width=2)
#         #     Model_2 = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=2,
#         #                         location_charging_points=[[5, 10]], minimum_charge=f, scanning_time=15,
#         #                         scanning_consumption=2, name= str(f + 61), logging_obj=wb, plotting=False,
#         #                         area_length=3, area_width=2)
#         #     Model_3 = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=3,
#         #                         location_charging_points=[[5, 10]], minimum_charge= f, scanning_time=15,
#         #                         scanning_consumption=2, name= str(f +  80), logging_obj=wb, plotting=False,
#         #                     area_length=3, area_width=2)
#         #     Model_1.run_optimization()
#         #     Model_2.run_optimization()
#         #     Model_3.run_optimization()
            
#         #     cost_1.append(Model_1.get_objective_value('objective'))
#         #     cost_2.append(Model_2.get_objective_value('objective'))
#         #     cost_3.append(Model_3.get_objective_value('objective'))
#         # pyplot.figure()
#         # pyplot.plot(minimum_charge_1,cost_1,'r-p',label = '1 drone')
#         # pyplot.plot(minimum_charge_1,cost_2,'g-h',label = '2 drones')
#         # pyplot.plot(minimum_charge_1,cost_3,'b-*',label = '3 drones')
#         # pyplot.xlabel('Minimum Charge(%)')
#         # pyplot.ylabel('Cost variation ($) ')
#         # pyplot.title('Cost variation wrt minimum charge consumption')
#         # pyplot.legend()
#         # pyplot.show()
        for k in scanning_consumption_1:
            Model_1 = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=1,
                                location_charging_points=[[5, 10]], minimum_charge= 8, scanning_time=15,
                              scanning_consumption= k, name= str(k + 50), logging_obj=wb, plotting=False,
                              area_length=3, area_width=2)
            Model_2 = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=2,
                                location_charging_points=[[5, 10]], minimum_charge=8, scanning_time=15,
                                scanning_consumption=k, name= str(k + 61), logging_obj=wb, plotting=False,
                                area_length=3, area_width=2)
            Model_3 = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=3,
                                location_charging_points=[[5, 10]], minimum_charge= 8, scanning_time=15,
                                scanning_consumption=k, name= str(k +  80), logging_obj=wb, plotting=False,
                            area_length=3, area_width=2)
            Model_1.run_optimization()
            Model_2.run_optimization()
            Model_3.run_optimization()
            
            cost_1.append(Model_1.get_objective_value('objective'))
            cost_2.append(Model_2.get_objective_value('objective'))
            cost_3.append(Model_3.get_objective_value('objective'))
        pyplot.figure()
        pyplot.plot(scanning_consumption_1,cost_1,'r-p',label = '1 drone')
        pyplot.plot(scanning_consumption_1,cost_2,'g-h',label = '2 drones')
        pyplot.plot(scanning_consumption_1,cost_3,'b-*',label = '3 drones')
        pyplot.xlabel('Scanning Charge consumption per location (%)')
        pyplot.ylabel('Cost variation ($) ')
        pyplot.title('Cost variation wrt scanning charge consumption')
        pyplot.legend()
        pyplot.show()
        
        
            
# Model_1 = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=1,
#                                   location_charging_points=[[5, 10]], minimum_charge= 5, scanning_time=15,
#                                 scanning_consumption= 8, name= str(50), logging_obj=wb, plotting=True,
#                                 area_length=3, area_width=2)

# Model_1.run_optimization()
            

# Model_2 = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=2,
#                                   location_charging_points=[[5, 10]], minimum_charge= 5, scanning_time=15,
#                                 scanning_consumption= 8, name= str(50), logging_obj=wb, plotting=True,
#                                 area_length=3, area_width=2)

# Model_2.run_optimization()

# Model_3 = DroneScanningOpt(area_shape='map_locations', amount_of_drones_used=3,
#                                   location_charging_points=[[5, 10]], minimum_charge= 5, scanning_time=15,

#                                 scanning_consumption= 8, name= str(50), logging_obj=wb, plotting=True,
#                                 area_length=3, area_width=2)

# Model_3.run_optimization()

# -*- coding: utf-8 -*-
"""
Created on Sun Dec 13 23:10:05 2020

@author: sarka
"""
import gurobipy as gp  # don't forget to download gurobi with academic license
import numpy as np
import pandas as pd
from matplotlib import pyplot
import warnings
import math as mth
import xlwt
from xlwt import Workbook
import os.path
# from mpl_toolkits.basemap import basemap


# Class definition
class DroneScanningOpt:
    def __init__(self, area_shape, drone_amount, location_charging_point, charge_const, charge_time, min_charge,
                 time_const, amount_of_visits_charging_points, name, logging_obj, area_length=3, area_width=3,
                 resolution=1):
        # Define instance variables
        self.area_shape = area_shape
        self.area_width = area_width
        self.area_length = area_length
        self.resolution = resolution
        self.drone_amount = drone_amount
        self.gm = gp.Model('Modelname')
        self.x = {}
        self.y = {}
        self.area_coord = []
        self.dist = np.array([[0, 0, 0]])
        self.i = 0  # i is the name of the node
        self.x_vars = {}
        self.result = []
        self.q = {}
        self.time = {}
        self.M = 1000
        self.value_list = []
        self.new_value_list = []
        self.amount_of_links = 0
        self.obj_time = 0
        self.obj_dist = 0
        self.obj_charge = 0        
        self.obj_function = 0
        self.location_charg_point = location_charging_point
        self.amount_of_charg_stat = len(location_charging_point)
        self.charge_nodes = [0]  # Node 0 is always a charge node
        self.charge_constant = charge_const
        self.time_constant = time_const
        self.charge_time = charge_time
        self.min_charge = min_charge
        self.amount_visits_charg = amount_of_visits_charging_points
        self.name = name
        self.wb = logging_obj
        self.amount_of_landmarks = 0
        self.charge_scan = 5
        self.scan_time = 3

    def area_discr(self):
        # This part aims to make a list of coordinates of the nodes.
        if self.area_shape == 'rectangle':
            # Discretizes the amount area into different coordinates
            for y in range(self.area_width*self.resolution):
                for x in range(self.area_length*self.resolution):
                    self.area_coord.append([self.i, x/self.resolution, y/self.resolution])
                    self.i += 1

        if self.area_shape == 'map_locations':
            df = pd.read_excel(r'points.xlsx', 'Sheet1')

            y = df['LAT'].tolist()
            x = df['LONG'].tolist()
            print("y is equal to", y)
            print("x is equal to", x)
            if len(x) == len(y):
                self.amount_of_landmarks = len(x)
                for i in range(len(x)):
                    self.area_coord.append([self.i, round((abs(x[self.i]-x[0])*111.1), 2), round((abs(y[self.i]-y[0])
                                                                                                  * 111.1), 2)])
                    self.i += 1
            else:
                warnings.warn("The x and y coordinates do not have the same amount")

        # Adds the coordinates of the charging stations
        for w in range(0, self.amount_visits_charg):
            for z in range(len(self.location_charg_point)):
                self.area_coord.append([self.i, self.location_charg_point[z][0], self.location_charg_point[z][1]])
                self.charge_nodes.append(self.i)
                self.i += 1
        print(self.area_coord)

    def link(self, i, j):
        dist = np.sqrt((self.area_coord[j][1] - self.area_coord[i][1]) ** 2 + (
                        (self.area_coord[j][2]) - self.area_coord[i][2]) ** 2)
        return dist

    def dec_var(self):
        # Distance Variables to the nodes
        for k in range(self.drone_amount):
            self.value_list = []
            for i in range(len(self.area_coord)):
                lst = []
                for j in range(self.i):
                    if self.link(i, j) < 40 and i != j and self.i - len(self.location_charg_point) > j \
                            and self.i - len(self.location_charg_point) > i:
                        self.x_vars[i, j, k] = self.gm.addVar(obj=self.link(i, j), ub=2, lb=0, vtype=gp.GRB.INTEGER,
                                                              name='Link' + str(i) + '-' + str(
                                                                  j) + '_' + 'Drone ' + str(k))
                        lst.append(j)
                        self.amount_of_links += 1
                        self.obj_function += self.x_vars[i, j, k] * self.link(i, j)
                    elif i != j and self.i - len(self.location_charg_point) <= j \
                            or self.i - len(self.location_charg_point) <= i:
                        self.x_vars[i, j, k] = self.gm.addVar(obj=self.link(i, j), ub=2, lb=0, vtype=gp.GRB.INTEGER,
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
                self.q[i, k] = self.gm.addVar(obj=-0.01, ub=100, vtype=gp.GRB.CONTINUOUS,
                                              name='Charge at node ' + str(i) + ' Drone ' + str(k))
                self.obj_charge += self.q[i, k] * (-0.01)
        # Time Variables
        for k in range(self.drone_amount):
            for i in range(len(self.area_coord)):
                self.time[i, k] = self.gm.addVar(vtype=gp.GRB.CONTINUOUS,
                                                 name='Time at node ' + str(i) + ' Drone ' + str(k))
                self.obj_time += self.time[i, k] * 3

    def create_obj_funct(self):
        c1 = 75
        c2 = 55
        c3 = 50
        self.obj_function = c1*self.obj_dist + c2*self.obj_charge+c3*self.obj_time
        self.gm.setObjective(self.obj_function, gp.GRB.MINIMIZE)

    def verified_constr(self):
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
                        self.gm.addConstr(self.q[i, k] - self.q[j, k] - self.charge_constant*self.link(i, j)
                                          * self.x_vars[i, j, k]-self.charge_scan + (1 - self.x_vars[i, j, k]) * self.M >= 0)
                    if j in self.charge_nodes:
                        self.gm.addConstr(self.q[i, k] - self.charge_constant*self.link(i, j)*self.x_vars[i, j, k] +
                                          (1 - self.x_vars[i, j, k]) * self.M >= 0)

    def time_constr(self):
        # Start at time equal to 0
        for k in range(self.drone_amount):
            self.gm.addConstr(self.time[0, k] == 0)

        # Decrease the time when moving from one node to another
        for k in range(self.drone_amount):
            for i in range(0, self.i):
                for j in self.value_list[i]:
                    if j not in self.charge_nodes:
                        self.gm.addConstr(self.time[i, k] - self.time[j, k] + self.time_constant*self.link(i, j)
                                          * self.x_vars[i, j, k] + self.charge_scan - (1 - self.x_vars[i, j, k]) * self.M <= 0)
                    if j in self.charge_nodes:
                        if j != 0:
                            self.gm.addConstr(self.time[i, k] - self.time[j, k] + self.time_constant*self.link(i, j)
                                              * self.x_vars[i, j, k] + self.charge_time - (1 - self.x_vars[i, j, k]) * self.M <= 0)

    def plot(self, i, j, drone):
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        pyplot.plot([self.area_coord[i][1], self.area_coord[j][1]], [self.area_coord[i][2], self.area_coord[j][2]], color=colors[drone])
            
    def get_values_from_result(self, inputt, which_drone):
        lst_links = []
        lst_charge = []
        lst_time = []
        if inputt == "links":
            for i in range(int(self.amount_of_links/self.drone_amount)):
                lst_links.append(self.result[(int(self.amount_of_links/self.drone_amount))*which_drone+i])
            return lst_links
        if inputt == "charge":
            for i in range(self.amount_of_links, int(self.amount_of_links + len(self.area_coord))):
                lst_charge.append(round(self.result[int(i+(len(self.area_coord))*which_drone)], 2))
            return lst_charge
        if inputt == "time":
            for i in range(self.amount_of_links, int(self.amount_of_links + len(self.area_coord) + len(self.area_coord) / 2)):
                lst_time.append(round(self.result[int(i + len(self.area_coord) / 2 * which_drone)], 2))
            return lst_time

    def transfer(self):
        for i in range(len(self.value_list)):
            for j in range(len(self.value_list[i])):
                self.new_value_list.append([i, self.value_list[i][j]])

    def createplot(self):
        self.transfer()
        # How many different possibilities of links are there
        for j in range(self.drone_amount):  
            for i in range(len(self.get_values_from_result("links", j))):
                if self.get_values_from_result("links", j)[i] == 1:
                    self.plot(self.new_value_list[i][0], self.new_value_list[i][1], j)

        # Plots the nodes as dots
        for i in range(self.i):
            pyplot.plot([self.area_coord[i][1]], self.area_coord[i][2], 'rp', markersize=1)
            for j in range(len(self.location_charg_point)):
                pyplot.plot(self.location_charg_point[j][0],self.location_charg_point[j][1], 'rp', markersize=15)
            #im = pyplot.imread("Crater.png")
            #pyplot.imshow(im,extent=[0, self.area_width-1, 0, self.area_length-1])
            pyplot.scatter(np.array(self.area_coord)[:,1], np.array(self.area_coord)[:, 2])
            for j in range(self.drone_amount):
                n = self.get_values_from_result('charge',j)
                for i, txt in enumerate(n):
                    if txt != 100:
                        text_on_node = "q", j, txt, "N", i
                        pyplot.annotate(text_on_node, (np.array(self.area_coord)[i, 1], np.array(self.area_coord)[i, 2]))


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
                    lst.append(self.result[int(self.amount_of_links + (self.amount_of_landmarks)*(self.drone_amount) + self.amount_of_charg_stat*self.amount_visits_charg*self.drone_amount + counter)])
                    counter += 1
                lst.sort()
            return lst[-1]
        else:
            print("Invalid input")

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
            self.name.write(i+3, 4, 'Charge Location '+ str(i))
            self.name.write(i+3, 5, str(self.location_charg_point[i]))

        ####################### Block 3
        self.name.write(1, 7, 'Landmark Locations: ', style_title)
        for i in range(self.amount_of_landmarks):
            self.name.write(i+3, 7, 'Landmark Location '+ str(i))
            self.name.write(i+3, 8, str('[' + str(self.area_coord[i][1]) + ', ' + str(self.area_coord[i][2]) + ']'))

        ####################### Block 4
        self.name.write(x_location_sec_title+1, 1, 'Optimization specifics', style_title)
        self.name.write(x_location_sec_title+2, 1, 'Run time')
        self.name.write(x_location_sec_title+2, 2, self.gm.RunTime)
        self.name.write(x_location_sec_title+2, 3, 'seconds')

        self.name.write(x_location_sec_title+3, 1, 'Gap')
        self.name.write(x_location_sec_title+3, 2, self.gm.MIPGap)
        self.name.write(x_location_sec_title+3, 3, 'percentage')

        self.name.write(x_location_sec_title+4, 1, 'Objective value')
        self.name.write(x_location_sec_title+4, 2, self.gm.objVal)
        self.name.write(x_location_sec_title+4, 3, '-')

        self.name.write(x_location_sec_title+5, 1, 'Total Distance Flown by the Drones')
        self.name.write(x_location_sec_title+5, 2, self.get_obj_values('distance'))
        self.name.write(x_location_sec_title+5, 3, 'km')

        self.name.write(x_location_sec_title+6, 1, 'Total Operation Time')
        self.name.write(x_location_sec_title+6, 2, self.get_obj_values('time'))
        self.name.write(x_location_sec_title+6, 3, 'minutes')

        x_location_sec_title += 7

        ####################### Block 5
        self.name.write(x_location_sec_title+1, 1, 'Link Outputs', style_title)
        counter = 0
        for k in range(self.drone_amount):
            for i in range(0, self.i):
                for j in self.value_list[i]:
                    self.name.write(x_location_sec_title+counter+2, 1, str(self.x_vars[(i, j, k)]))
                    self.name.write(x_location_sec_title+counter+2, 2, str(self.result[counter]))
                    counter += 1

        ####################### Block 6
        self.name.write(x_location_sec_title+1, 4, 'Charge Outputs', style_title)
        counter1 = 0
        for k in range(self.drone_amount):
            for i in range(len(self.area_coord)):
                    self.name.write(x_location_sec_title+counter1+2, 4, str(self.q[(i, k)]))
                    self.name.write(x_location_sec_title+counter1+2, 5, str(self.result[counter]))
                    counter1 += 1
                    counter += 1

        ####################### Block 7
        self.name.write(x_location_sec_title+1, 7, 'Time Outputs', style_title)
        counter2 = 0
        for k in range(self.drone_amount):
            for i in range(len(self.area_coord)):
                    self.name.write(x_location_sec_title+counter2+2, 7, str(self.time[(i, k)]))
                    self.name.write(x_location_sec_title+counter2+2, 8, str(self.result[counter]))
                    counter2 += 1
                    counter += 1

        # Save to file
        self.wb.save('Output_Data.csv')


    def run_optimization(self):
        self.area_discr()
        self.dec_var()

        self.verified_constr()
        self.charge_constr()
        self.time_constr()
        self.create_obj_funct()
        self.gm.update()

        self.gm.write('test.lp')  # write to LP file
        self.gm.setParam('Timelimit', 10)  # Time limit in seconds
        self.gm.setParam('MIPGap', .1)  # Set a gap at which it stops with 0.1 equal to 10 percent
        self.gm.optimize()
        self.gm.printAttr('X')
        self.result = self.gm.getAttr('X')
        self.createplot()
        pyplot.show()
        self.save_data()


if __name__ == "__main__":
    # Checks if the output file is in the directory
    if os.path.exists("Output_Data.csv"):
        pass
    # No file is detected and a new one is created
    else:
        f = open("Output_Data.csv", "x")
        f.close()
    wb = Workbook()  # Create Workbook instance called wb
    # Instance creation
    Model1 = DroneScanningOpt(area_shape='map_locations', drone_amount=3,
                              location_charging_point=[[0.5, 0.5]], charge_const=5, min_charge=10,
                              charge_time=80, time_const=.75, amount_of_visits_charging_points=2, name="map_location_run1", logging_obj=wb)

    Model2 = DroneScanningOpt(area_shape='map_locations', drone_amount=2,
                              location_charging_point=[[1, 2]], charge_const=1.5, min_charge=10,
                              charge_time=80, time_const=.75, amount_of_visits_charging_points=2, name="map_location_run2", logging_obj=wb)
    Model3 = DroneScanningOpt(area_shape='map_locations', drone_amount=1,
                              location_charging_point=[[0.5, 0.5]], charge_const=.10, min_charge=30,
                             charge_time=80, time_const=.75, amount_of_visits_charging_points=2, name="map_location_run3", logging_obj=wb)

    # Run first instance
   # Model1.run_optimization()
    Model2.run_optimization()
   # Model3.run_optimization()


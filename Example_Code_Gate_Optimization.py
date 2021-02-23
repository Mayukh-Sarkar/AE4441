# Import required packages
from gurobipy import *
import pandas as pd
from xlwt import Workbook

# DATA
# read data from excel
filename = 'data2.xlsx'
alpha = pd.read_excel(filename, sheet_name="alpha").values.tolist()[0][0]
C_pax = pd.read_excel(filename, sheet_name="C_pax").values.tolist()
schedule = pd.read_excel(filename, sheet_name="schedule").values.tolist()
P = pd.read_excel(filename, sheet_name="P").values.tolist()
B_fuel = pd.read_excel(filename, sheet_name="B_fuel").values.tolist()[0]
CR = pd.read_excel(filename, sheet_name="CR").values.tolist()[0][0]

# get nr of bays, nr of flights and nr of time steps
bays = range(len(C_pax))
flights = range(len(schedule)) 
time = range(max(schedule)[1]+1) # time runs from 0 untill the largest departure time 

# GUROBI MODEL
# initialize gurobi model
m = Model('planning')

# initialize decision variables x and y as dictionaries
x = {}
y = {}

# OBJECTIVE FUNCTION
for i in flights:
    for j in bays:
        # declare x[i,j] decision variable
        x[i,j] = m.addVar(obj=0, lb=0, vtype=GRB.BINARY, name = 'X[%s,%s]' %(i,j))
        
        for i_out in flights:
            for j_out in bays:
                # declare y[i,j,i',j'] decision variable and obj. func. coefficient
                y[i,j,i_out,j_out] = m.addVar(obj = alpha*P[i][i_out]/CR * 
                 C_pax[j][j_out], lb=0, vtype=GRB.BINARY, name = 'Y[%s,%s,%s, %s]'
                 %(i,j,i_out, j_out))   

# Set objective to minimize
m.update()
m.setObjective(m.getObjective(), GRB.MINIMIZE)  

# CONSTRAINTS
# Single bay per aircraft constraint
for i in flights:
        m.addConstr(quicksum(x[i,j] for j in bays) == 1, name = 'Csb/ac[i%s]' %(i)) 

# Concurrently contraint, single or no aircraft per bay 
for t in time:
    ground = {} # set of all aircraft on the ground during time interval i
    for i in flights:
        if t >= schedule[i][0] and t<= schedule[i][1]:
            ground[i]=i
    for j in bays:
        m.addConstr(quicksum(x[i,j] for i in ground) <= 1, name = 'Cconc[t%s,j%s]'
                    %(t,j) )

# linearization and conservation constraint
for i_out in flights:
    for i in range(i_out):
        for j in bays:
            for j_out in bays:
                m.addConstr(x[i,j] + x[i_out,j_out] -  2 * y[i,j,i_out,j_out] <= 1,
                            name = 'Clin[i%s,j%s,i_out%s,j_out%s]' %(i,j,i_out,j_out))
                            
# fuelling constraint, if a flight needs fuel, it must be at a fuel bay
for i in flights:
    if schedule[i][2] == 1:
            m.addConstr(quicksum(x[i,j] for j in B_fuel) == 1, name = 
                        'Cfuel[i%s,t%s]' %(i,t))

# SOLVE
m.update()
m.write('test.lp') # write to LP file 
m.optimize()

# OUTPUT
# Workbook is created 
wb = Workbook() 
# add_sheet is used to create sheet. 
sheet1 = wb.add_sheet('Sheet 1', cell_overwrite_ok=True) 
# bay numbers in column 0
for j in bays:
    sheet1.write(j+1,0, "bay "+str(j+1))
# time interval numbers in row 0
for t in time:
    sheet1.write(0,t, 't ' + str(t))
# write assignment     
for i in flights:
    for j in bays:
        if x[i,j].x==1:
            for t in time:
                if t >= schedule[i][0] and t<= schedule[i][1]:
                    sheet1.write(j+1, t , i+1)
# save in an excel file
wb.save('output2_fuel.xls')





from gurobipy import *
import pandas as pd

# DATA
# read data from excel
alpha = pd.read_excel(io="data.xlsx", sheet_name="alpha").values.tolist()[0][0]
C_pax = pd.read_excel(io="data.xlsx", sheet_name="C_pax").values.tolist()
schedule = pd.read_excel(io="data.xlsx", sheet_name="schedule").values.tolist()
P = pd.read_excel(io="data.xlsx", sheet_name="P").values.tolist()
B_fuel = pd.read_excel(io="data.xlsx", sheet_name="B_fuel").values.tolist()[0]
CR = pd.read_excel(io="data.xlsx", sheet_name="CR").values.tolist()[0][0]

# get nr of bays, nr of flights and nr of time steps
bays = range(len(C_pax))
flights = range(len(schedule))
time = range(9)

# GUROBI MODEL
m = Model('planning')
x = {}
y = {}

for i in flights:
    for j in bays:
        x[i,j] = m.addVar(obj=0, lb=0, vtype=GRB.BINARY, name = 'X[%s,%s]' %(i,j))
        
        for i_out in flights:
            for j_out in bays:
                # declare y variable
                y[i,j,i_out,j_out] = m.addVar(obj = alpha*P[i][i_out]/CR * C_pax[j][j_out], # cost of passenger connections
                lb=0, vtype=GRB.BINARY, name = 'Y[%s,%s,%s, %s]' %(i,j,i_out, j_out))   
            
m.update()
m.setObjective(m.getObjective(), GRB.MINIMIZE)  # The objective is to minimize costs

# CONSTRAINTS
# Single bay per aircraft constraint
for i in flights:
        m.addConstr(quicksum(x[i,j] for j in bays) == 1, name = 'Csb/ac[i%s]' %(i)) 

# Concurrently contraint, single or no aircraft per bay 
for t in time:
    ground = {}
    for i in flights:
        if t >= schedule[i][0] and t<= schedule[i][1]:
            ground[i]=i
    for j in bays:
        m.addConstr(quicksum(x[i,j] for i in ground) <= 1, name = 'Cconc[t%s,j%s]' %(t,j) )

# linearization constraint
for i_out in flights:
    for i in range(i_out):
        for j in bays:
            for j_out in bays:
                m.addConstr(x[i,j] + x[i_out,j_out] -  2 * y[i,j,i_out,j_out] <= 1, name = 'Clin[i%s,j%s,i_out%s,j_out%s]' %(i,j,i_out,j_out))
                            
# fuelling constraint, if a flight needs fuel, it must be at a fuel gate from t_a+1 untill t_d
for i in flights:
    if schedule[i][2] == 1:
        for t in range(schedule[i][0]+1, schedule[i][1]+1):
            m.addConstr(quicksum(x[i,j] for j in B_fuel) == 1, name = 'Cfuel[i%s,t%s]' %(i,t))

m.update()
m.write('test.lp')
m.optimize()





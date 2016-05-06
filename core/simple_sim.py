import numpy as np

def get_cell(x):
	i = np.floor(x[0]/0.1)
	j = np.floor(x[1]/0.1)
def get_control_output(cell_f,cell_t):
	velocity = 10.09
	ux = (cell_t[0]-cell_f[0])*0.1*velocity
	uy = (cell_t[1]-cell_f[1])*0.1*velocity

	return np.array([ux,uy])


x = np.array([0,0])
u = np.array([0,0])

trace_x = []



ltl_plan = [(1,2),(3,3),...]

ltl_id = 0
ltl_state = ltl_plan[0]

t = 0.0
dt = 0.1
while t<10.0:
	# simulation
	x = x + u*dt
	trace_x.append(x.copy())

	t +=dt

	# LTL plan
	current_cell = get_cell(x)

	if current_cell!=ltl_state:
		ltl_id+=1

		ltl_state = ltl_plan[ltl_id]
		u = get_control_output(current_cell,ltl_state)

	print "Time=",t,"\t LTL state",""  
	
#plots
import matplotlib.pyplot as plt

trace = np.array(trace_x)
plt.plot(trace[:,0],trace[:,1])

plt.show()
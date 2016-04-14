import matplotlib.cm as cmx
import matplotlib.colors as colors
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import numpy as np
import simulator

def get_cmap(N):
	'''Returns a function that maps each index in 0, 1, ... N-1 to a distinct 
	RGB color.'''
	color_norm  = colors.Normalize(vmin=0, vmax=N-1)
	scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='jet') 
	def map_index_to_rgb_color(index):
		return scalar_map.to_rgba(index)
	return map_index_to_rgb_color

class GUI:
	def __init__(self,env,sim):
		self.env = env
		self.sim = sim
		self.rate = 25

		self.fig = plt.figure()

		self.ax_env = env.plot(plt)
		self.ax_obj, = plt.plot([0],[0],"ro")


		self.anim = animation.FuncAnimation(self.fig, self.redraw, 25, fargs=None,interval=50, blit=False)


	def redraw(self,d):
		pts = []
		for o in self.sim.get_class_obj(simulator.DrawableObject):
			pts.append(o.state)
		p = np.array(pts)

		try:
			self.ax_obj.set_data(p[:,0],p[:,1])
		except IndexError,e:
			print e,p
		return self.ax_obj,

	def start(self):
		plt.show()
		self.sim.stop = True
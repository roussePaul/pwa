import numpy as np
from scipy.optimize import linprog
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import itertools
import time
from mayavi import mlab
def test_optimization():
	points = np.random.uniform(-1,1,(4,2))
	norms = np.apply_along_axis(np.linalg.norm, 1, points)
	points = np.transpose(points.T/norms)
	print points

	eps = 1e-4
	A = np.array(points)
	(M,N) = A.shape
	b = np.zeros((M,1))
	c = np.ones(N)

	D = np.concatenate((A,-A),axis = 0)
	e = np.concatenate((b,b+0.1),axis=0)


	t = time.time()
	print M,N
	print "min ", c,"* x"
	print D, "* x <", e

	res = linprog(c, A_ub=D, b_ub=e,bounds=None)
	print res

	print time.time()-t
	sol_x = res['x']

	t = np.array([-1000,1000])
	plt.plot(points[:,0],points[:,1],'ro')

	axes = plt.gca()
	for p,y in zip(D,e):
		r = y/(p[0]**2+p[1]**2)
		x = np.array([-p[1],p[0]]) 
		pts = np.array([p*r-1000*x,p*r+1000*x])

		if np.dot(p,[0,100])>y:
			axes.fill_between(pts[:,0], pts[:,1], 100, facecolor='green',alpha=0.7)
		else:
			axes.fill_between(pts[:,0], pts[:,1], -100, facecolor='green',alpha=0.7)

	for p,y in zip(D,e):
		r = y/(p[0]**2+p[1]**2)
		x = np.array([-p[1],p[0]]) 
		pts = np.array([p*r-1000*x,p*r+1000*x])
		
		if y>1.0e-3:
			plt.plot(pts[:,0],pts[:,1],'g')
		else:
			plt.plot(pts[:,0],pts[:,1],'b')
			plt.plot([p[0],0],[p[1],0],'b--')




	plt.axis('equal')
	axes.set_xlim([-1,1])
	axes.set_ylim([-1,1])


# def onclick(event):
# 	p = np.array([event.xdata, event.ydata])
# 	print np.array([np.dot(D,p)]).T-e
# 	print np.all(np.array([np.dot(D,p)]).T-e<0)

# plt.gcf().canvas.mpl_connect('button_press_event', onclick)

# plt.show()

# Deuxieme algorithm listant toute les solutions
# complexite O(Combinaison(dimension de l'espace-1 parmis nombre de points))
# dim 3 avec 30 points -> 30*29 droites possibles
# attention, j'ai ajoute de la tolerence, l'inegalite stricte n'est pas respectee
def test_2d():
	points = np.random.uniform(-1,1,(4,2))
	norms = np.apply_along_axis(np.linalg.norm, 1, points)
	points = np.transpose(points.T/norms)
	print points

	A = np.array(points)

	eps = 1e-16

	t = time.time()

	F = []
	for a in list(A):
		F.append(np.array([a[0],a[1]]))
	print F

	solutions = []
	n = 2
	for h in itertools.combinations(F,n-1):
		l = h+(0.0*h[0],)
		a = np.concatenate(tuple([np.array([j]) for j in l]),axis=0)
		#print a

		w,v = np.linalg.eig(a)
		#print "M=",a
		#print "eig_val=",w
		#print "eig_vect=",v


		null_eig_val = np.abs(w)<eps
		n_zero_eig = np.count_nonzero(null_eig_val)
		#print n_zero_eig

		if n_zero_eig==0:
			raise Exception("We should find at least one eigen value equal to zero")
		if n_zero_eig>1:
			#print "More than one eigenvalue equal to zero"
			pass

		idv = np.argmax(null_eig_val)
		line_kernel = np.array([v[:,idv]])
		#print "k= ",line_kernel
		#print np.dot(A,line_kernel.T)
		solutions.append(line_kernel[0])
		solutions.append(-line_kernel[0])

	in_cone = []

	for s in solutions:
		if np.all(np.dot(A,s.T)<=eps):
			#print "Found one: ",line_kernel
			in_cone.append(s)

	print "Time = ",time.time()-t

	if in_cone:
		pts = np.array(in_cone)
		for p in pts:
			m = np.array([[0,0],100*p])
			plt.plot(m[:,0],m[:,1],"m",linewidth=1.5)
	plt.savefig('foo.png')

	print list(itertools.combinations(F,1))



def get_cone(controls,verbose=0):
	A = np.array(controls)
	M,N = A.shape
	eps = 1e-10

	solutions = []
	in_cone = []
	for h in itertools.combinations(A,N-1):
		l = h+(0.0*h[0],)
		a = np.concatenate(tuple([np.array([j]) for j in l]),axis=0)

		w,v = np.linalg.eig(a)
		if verbose:
			print "M=",a
			print "eig_val=",w
			print "eig_vect=",v

		null_eig_val = np.abs(w)<eps
		n_zero_eig = np.count_nonzero(null_eig_val)

		id_null = np.where(np.abs(w)<eps)
 		n_indep_null_eig = np.linalg.matrix_rank(v[:,id_null[0]])

		if n_zero_eig==0:
			raise Exception("We should find at least one eigen value equal to zero")
		if n_indep_null_eig>1:
			print "More than one eigenvalue equal to zero"
			print "Matrice of the plans:",a
			print "Eigen values:",w
			print "Eigen vectors:",v
			continue

		idv = id_null[0][0]
		line_kernel = np.array([v[:,idv]])
		ker = line_kernel[0]
		if not np.all(ker.imag==0):
			raise Exception("imaginary part not null!",ker)
		s = ker.real
		solutions.append(s)
		solutions.append(-s)

		sol = [s,-s]
		for c in sol:
			if verbose:
				print "is vector",c,"in the cone?",np.dot(A,c.T)
			if np.all(np.dot(A,c.T)<=eps):
				in_cone.append(c)
				if verbose:
					print "Found cone vectrice:",c

	return in_cone

def is_system_fair(controls):
	A = np.array(controls)
	M,N = A.shape
	eps = 1e-10

	solutions = []
	in_cone = []
	for h in itertools.combinations(A,N-1):
		l = h+(0.0*h[0],)
		a = np.concatenate(tuple([np.array([j]) for j in l]),axis=0)

		w,v = np.linalg.eig(a)

		null_eig_val = np.abs(w)<eps
		n_zero_eig = np.count_nonzero(null_eig_val)

		id_null = np.where(np.abs(w)<eps)
 		n_indep_null_eig = np.linalg.matrix_rank(v[:,id_null[0]])

		if n_zero_eig==0:
			raise Exception("We should find at least one eigen value equal to zero")
		if n_indep_null_eig>1:
			print "More than one eigenvalue equal to zero"
			print "Matrice of the plans:",a
			print "Eigen values:",w
			print "Eigen vectors:",v
			continue

		idv = id_null[0][0]
		line_kernel = np.array([v[:,idv]])
		ker = line_kernel[0]
		if not np.all(ker.imag==0):
			raise Exception("imaginary part not null!",ker)
		s = ker.real
		solutions.append(s)
		solutions.append(-s)

		sol = [s,-s]
		for c in sol:
			if np.all(np.dot(A,c.T)<=eps):
				return True

	return False

def draw_plan(ax,normal,point):
	d = -np.sum(point*normal)
	xx, yy = np.meshgrid(np.linspace(-1,1,20), np.linspace(-1,1,20))
	z = (-normal[0]*xx - normal[1]*yy - d)*1./normal[2]
	ax.plot_surface(xx,yy,z)

def check_results(controls,cone,verbose=1):
	A = np.array(controls)
	eps = 1e-10
	for c in cone:
		p = np.dot(A,c)
		if verbose:
			print np.all(p<=eps),p,np.where(p>eps)

def test_3d():
	a = list(itertools.product(([0.5,1]),repeat=3))
	points = [np.array(list(b)) for b in a]

	print "Points:",points

	cone = get_cone(points,verbose=1)

	print "Cone:",cone

	fig = plt.figure()
	ax = Axes3D(fig)
	ax.set_aspect('equal')

	tmp = [(np.array([0,0,0]),p) for p in points]
	pts = np.array([a for e in tmp for a in e])
	ax.plot(pts[:,0],pts[:,1],pts[:,2],'g')

	tmp = [(np.array([0,0,0]),p) for p in cone]
	pts = np.array([a for e in tmp for a in e])
	ax.plot(pts[:,0],pts[:,1],pts[:,2],c='r')


	for p in points:
		print p
		draw_plan(ax,p,np.array([0,0,0]))


	# Create cubic bounding box to simulate equal aspect ratio
	max_range = 10
	Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten()
	Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten()
	Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten()
	# Comment or uncomment following both lines to test the fake bounding box:
	for xb, yb, zb in zip(Xb, Yb, Zb):
		ax.plot([xb], [yb], [zb], 'w')

	check_results(points,cone)
	plt.show()

def massive_test(N):
	t = time.time()
	errors = []
	for i in range(N):
		points = np.random.uniform(-1,1,(10,4))
		cone = get_cone(points,verbose=0)
		if check_results(points,cone,verbose=0)==False:
			errors.append(points)
		if (100.0*i/N)%5==0:
			print 100.0*i/N
	print "Time per iteration:",(time.time()-t)/N
	print "Errors:",len(errors)
	print errors

def plan3d(normal,point):
	d = -np.sum(point*normal)
	xx, yy = np.meshgrid(np.linspace(-1,1,20), np.linspace(-1,1,20))
	z = (-normal[0]*xx - normal[1]*yy - d)*1./normal[2]
	mlab.mesh(xx,yy,z,color=(1.0,0.4,0.2))


def show3d():
	# cube
	a = list(itertools.product(([0.5,1]),repeat=3))
	controls = [np.array(list(b)) for b in a]

	# random
	controls = np.random.uniform(0.0,1.0,(5,3))
	norms = np.apply_along_axis(np.linalg.norm, 1, controls)
	controls = np.transpose(controls.T/norms)

	cone = get_cone(controls,verbose=1)

	print "Controls:",controls
	print "Cone:",cone

	for c in controls:
		plan3d(c,np.array([0]*3))
	
	mlab.points3d(controls[:,0],controls[:,1],controls[:,2])

	for c in cone:
		l = np.array([c*0.0] + [c*1.0])
		x = l[:,0]
		y = l[:,1]
		z = l[:,2]
		mlab.plot3d(x,y,z,line_width=1.0)

	mlab.show()

show3d()
#massive_test(100)

print get_cone([np.array([1,1]) , np.array([-1,-1]) ])
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import numpy as np
import math

def dot(v,w):
    x,y = v
    X,Y = w
    return x*X + y*Y

def length(v):
    x,y = v
    return math.sqrt(x*x + y*y)

def vector(b,e):
    x,y = b
    X,Y = e
    return (X-x, Y-y)

def unit(v):
    x,y = v
    mag = length(v)
    return (x/mag, y/mag)

def distance(p0,p1):
    return length(vector(p0,p1))

def scale(v,sc):
    x,y = v
    return (x * sc, y * sc)

def add(v,w):
    x,y = v
    X,Y = w
    return (x+X, y+Y)

def pnt2line(pnt, start, end):
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    t = dot(line_unitvec, pnt_vec_scaled)    
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return (dist, nearest)

def point_in_poly(x,y,poly):

    n = len(poly)
    inside = False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside

def distance_to_hull(p,poly):
    dist_list = []
    for v_idx in range(len(poly)):
        start = poly[v_idx - 1]
        end = poly[v_idx]
        temp = pnt2line(p, start, end)
        dist_list.append(temp[0])

    dist_temp = min(dist_list)          

    return dist_temp

if __name__ == '__main__':
        
    # Original points, hull and test points
    points = np.random.rand(30, 2)   # 30 random points in 2-D
    hull = ConvexHull(points)
    newpoints = np.random.rand(30, 2)   # 30 random points in 2-D



    pt_dist = []
    for p_idx in range(30):
        pt = newpoints[p_idx,:]
        dist_list = []
        for v_idx in range(len(hull.vertices)):
            v1 = hull.vertices[v_idx - 1]
            v2 = hull.vertices[v_idx]
            start = points[v1]
            end = points[v2]
            temp = pnt2line(pt, start, end)
            dist_list.append(temp[0])

        dist_temp = min(dist_list)          

        pt_dist.append(dist_temp)


    # Plot original points, hull and new points
    plt.plot(points[:,0], points[:,1], 'ro')
    plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)
    plt.plot(newpoints[:,0], newpoints[:,1], 'go')
    for p_idx in range(30):
        pt = newpoints[p_idx,:]
        pt[1] = pt[1] + 0.01 
        dist = pt_dist[p_idx]
        distLabel = "%.2f" % dist
        plt.annotate(distLabel,xy=pt)
    plt.show()
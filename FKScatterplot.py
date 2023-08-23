import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from FK import FK

fig = plt.figure()
ax = fig.gca(projection='3d')

#Plot Robot Rough Outline
z = np.linspace(0,7,50)
theta = np.linspace(0,2*np.pi,50)
theta_grid,z_grid = np.meshgrid(theta,z)
x_grid = 3*np.cos(theta_grid)
y_grid = 3*np.sin(theta_grid)

res1 = 1
res2 = 5
res3 = 5
res4 = 1
basemin = -135
basemax = 135
shouldermin = -105
shouldermax = 105
elbowmin = -115
elbowmax = 115
wristmin = -90
wristmax = 90
x = np.zeros(res1*res2*res3*res4)
y = np.zeros(res1*res2*res3*res4)
z = np.zeros(res1*res2*res3*res4)
u = np.zeros(res1*res2*res3*res4)
v = np.zeros(res1*res2*res3*res4)
w = np.zeros(res1*res2*res3*res4)
counter = 0
for i in np.linspace(basemin,basemax,res1): #BASE
    for j in np.linspace(shouldermin,shouldermax,res2): # SHOULDER
        for k in np.linspace(elbowmin,elbowmax,res3): # ELBOW
            for m in np.linspace(wristmin,wristmax,res4): # WRIST
                T = FK([i,j,k,m])
                if T is not None:
                    x[counter] = T[0,3]-(6*T[0,2])
                    y[counter] = T[1,3]-(6*T[1,2])
                    z[counter] = T[2,3]-(6*T[2,2])
                    u[counter] = T[0,2]
                    v[counter] = T[1,2]
                    w[counter] = T[2,2]
                    print(counter)
                    counter = counter+1
ax.quiver(x,y,z,u,v,w,length=6,normalize=True,pivot='tail')
ax.plot_surface(x_grid,y_grid,z_grid, alpha=.5)
plt.show()
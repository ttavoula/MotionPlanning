from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

data = numpy.loadtxt('path_pendulum.txt')
fig = plt.figure()
ax = fig.add_subplot()
ax.plot(data[:,0],data[:,1],'.-')
plt.show()
import sys
import numpy as np
import math
from math import sin, cos
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as patches


def readPath(filename):
	lines = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]
	data = [[float(x) for x in line.split(' ')] for line in lines]
	return data

def plotEnvironment(obstacles):
	rect = patches.Rectangle((-10,-10), 20, 4, facecolor='gray', linewidth=1, )
	obstacles.add_patch(rect)
	rect = patches.Rectangle((-10,-4), 10, 8, facecolor='gray', linewidth=1)
	obstacles.add_patch(rect)
	rect = patches.Rectangle((2,-4), 8, 8, facecolor='gray', linewidth=1)
	obstacles.add_patch(rect)
	rect = patches.Rectangle((-10,6), 20, 2, facecolor='gray', linewidth=1)
	obstacles.add_patch(rect)

def plotCar(path):
	fig = plt.figure()
	obstacles = fig.gca()
	X = [p[0] for p in path]
	Y = [p[1] for p in path]
	obstacles.plot(X, Y)
	plotEnvironment(obstacles)

	# start state
	robotStart = patches.Rectangle((-8.25, -5.25), 0.5, 0.5, fill=True, edgecolor = 'r')
	x_g = (-0.25)*cos(1) - (-0.25)*sin(1) + 5
	y_g = (-0.25)*cos(1) + (-0.25)*sin(1) + 5

	# goal state
	robotGoal = patches.Rectangle((x_g, y_g), 0.5, 0.5, 180/math.pi, fill=True, color = 'g')
	plt.gca().add_patch(robotStart)
	plt.gca().add_patch(robotGoal)
	plt.axis([-10, 10, -10, 10])

	# show path states based on the center point
	for p in path:
		x1 = (-0.25)*cos(p[2]) - (-0.25)*sin(p[2]) + p[0]
		y1 = (-0.25)*cos(p[2]) + (-0.25)*sin(p[2]) + p[1]
		robot = patches.Rectangle((x1, y1), 0.5, 0.5, p[2]*180/math.pi, fill=False, edgecolor='b')
		plt.gca().add_patch(robot)
	plt.show()

if __name__ == '__main__':
	x = []
	y = []
	path = readPath('path_car.txt')
	plotCar(path)
import rospy 
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from dd_bot_control.msg import DDBotState
from nav_msgs.msg import Odometry

curr_x = []
curr_y = []
traj_x = []
traj_y = []
receivedDesTraj = False
receivedPose = False

fig= plt.figure()
ax1 = plt.axes(xlim=(-10, 10), ylim=(-10, 10))
line, = ax1.plot([], [], lw=2)

plotlays, plotcols = [2], ["red","black"]
lines = []
for index in range(2):
    lobj = ax1.plot([],[],lw=2,color=plotcols[index])[0]
    lines.append(lobj)


def desTrajCallback(data):

	global traj_x
	global traj_y

	traj_x.append(data.x)
	traj_y.append(data.y)

	receivedDesTraj = True

def poseCallback(data):

	global curr_x
	global curr_y

	curr_x.append(data.pose.pose.position.x)
	curr_y.append(data.pose.pose.position.y)

	receivedPose = True

def initFigure():

	global fig
	global ax1

	ax1.set_xlabel('X')
	ax1.set_ylabel('Y')
	ax1.set_title('MPC Tracking Performance')
	ax1.grid()

def init():
	global lines

	for line in lines:
		line.set_data([],[])
	return lines
	

def animate(i):

	global curr_x
	global curr_y
	global traj_x
	global traj_y
	global lines

	global ax1

	#ax1.clear()
	#ax1.plot(traj_x, traj_y, 'ko')
	#ax1.plot(curr_x, curr_y, 'ro')
	xlist = [curr_x, traj_x]
	ylist = [curr_y, traj_y]

	for lnum,line in enumerate(lines):
		line.set_data(xlist[lnum], ylist[lnum]) # set data for each line separately. 

	return lines



def plotter(): 

	global ax1
	global fig

	global curr_x
	global curr_y
	global traj_x
	global traj_y

	global receivedDesTraj
	global receivedPose


	rospy.init_node('plotter', anonymous=True)

	rospy.Subscriber('/dd_bot/des_traj', DDBotState, desTrajCallback)
	rospy.Subscriber('/ground_truth/state', Odometry, poseCallback)

	initFigure()
	ani = animation.FuncAnimation(fig, animate, init_func=init, frames=1000, interval=30, blit=True)
	plt.show()

	while not rospy.is_shutdown():

		a = 1

		# if receivedDesTraj:
		# 	plt.plot(traj_x, traj_y, 'ko')
		# 	receivedDesTraj = False

		# if receivedPose:
		# 	plt.plot(curr_x, curr_y, 'ro')
		# 	receivedPose = False





if __name__ == '__main__':

	try:
		plotter()
	except rospy.ROSInterruptException:
		pass

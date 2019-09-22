#!/usr/bin/env python

import rospy
from copy import deepcopy 
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from dd_bot.msg import CFTOCInput
from dd_bot.msg import CFTOCOutput
from cvxopt import matrix, spmatrix, sparse, spdiag
from cvxopt.solvers import qp
import numpy as np

N = 0
time_step = 0.0
H_bar = []
G_eq = []
E_eq = []

G_in = []
W_in = []
J = []
Xo = []
Et = []
q_diff = []


Q = []
R = []
P = []
B = []
I = []
received_input = False

cftoc_input = CFTOCInput()
cftoc_output = CFTOCOutput();

def initialize_parameters():

	global Q
	global R
	global P
	global B
	global I
	global N
	global Xo
	global Et
	global q_diff
	global time_step

	Q_flat = rospy.get_param('/dd_mpc/Q_flat')
	R_flat = rospy.get_param('/dd_mpc/R_flat')
	P_flat = rospy.get_param('/dd_mpc/P_flat')

	Q_dim = rospy.get_param('/dd_mpc/Q_dim')
	R_dim = rospy.get_param('/dd_mpc/R_dim')
	P_dim = rospy.get_param('/dd_mpc/P_dim')

	Q = matrix(Q_flat, (Q_dim, Q_dim))
	R = matrix(R_flat, (R_dim, R_dim))
	P = matrix(P_flat, (P_dim, P_dim))

	N = rospy.get_param('/dd_mpc/horizon_length')
	time_step = rospy.get_param('/dd_mpc/time_step')

	B = spmatrix([-time_step, -time_step], [0, 2], [0, 1])
	I = spmatrix(1.0, range(3), range(3))
	Xo = matrix(0.0, (Q_dim, 1))

	Et = matrix(0.0, (Q_dim, Q_dim));
	q_diff = matrix(0.0, (Q_dim, 1));



def formulate_H_bar():

	global H_bar
	global J
	global N

	QP_list = [Q]*(N-1)
	QP_list.append(P)
	R_list = [R]*N
	QPR_diag = QP_list + R_list

	H_bar = spdiag(QPR_diag)

	J = matrix(0.0, ((3*N + 2*N), 1))

def formulate_state_and_input_constraints():

	global G_in
	global E_in

	G_in = matrix(0.0, (1, (3*N + 2*N)))
	E_in = matrix(0.0, (1,1))

def formulate_dynamic_constraints():

	global G_eq
	global E_eq
	global cftoc_input
	global Xo
	global Et
	global q_diff


	G_eq = spmatrix([], [], [], (3*N, (3*N + 2*N)))
	E_eq = spmatrix([], [], [], (3*N, 3))

	q_diff[0, 0] = cftoc_input.ref_traj[0].x - cftoc_input.curr_state.x
	q_diff[1, 0] = cftoc_input.ref_traj[0].y - cftoc_input.curr_state.y
	q_diff[2, 0] = cftoc_input.ref_traj[0].theta - cftoc_input.curr_state.theta

	Et[0, 0] = np.cos(cftoc_input.curr_state.theta)
	Et[0, 1] = np.sin(cftoc_input.curr_state.theta)
	Et[1, 0] = -np.sin(cftoc_input.curr_state.theta)
	Et[1, 1] = np.cos(cftoc_input.curr_state.theta)

	Xo = Et * q_diff

	A = spmatrix(1.0, range(3), range(3))
	
	for i in range(N):

		A[0, 1] = cftoc_input.ref_traj[i].w * time_step
		A[1, 0] = -cftoc_input.ref_traj[i].w * time_step
		A[1, 2] = cftoc_input.ref_traj[i].v * time_step

		G_eq[i*3:(i+1)*3, i*3:(i+1)*3] = I
		G_eq[i*3:(i+1)*3, (3*N + i*2): (3*N + (i+1)*2)] = -B

		if i == 0:
			
			E_eq[0:3, 0:3] = A

		else: 

			G_eq[i*3:(i+1)*3, (i-1)*3:i*3] = -A


	#print(E_eq)
	#print(Xo)
	E_eq = E_eq*Xo
	#print(E_eq)



def solve_cftoc():
	#print(H_bar)
	#print(G_eq)
	#print(E_eq)

	x = qp(H_bar, J, G_in, E_in, G_eq, E_eq)['x']
	#print(x)
	cftoc_output.v_out = x[N*3]
	cftoc_output.w_out = x[N*3+1]


def cftocInputCallback(data):
	
	global cftoc_input
	global received_input
	cftoc_input = deepcopy(data)
	received_input = True



def cftoc_solver():

	rospy.init_node('cftoc_solver', anonymous=True)

	rospy.Subscriber("dd_bot/cftoc_input", CFTOCInput, cftocInputCallback)
	cftoc_output_pub = rospy.Publisher("dd_bot/cftoc_output", CFTOCOutput, queue_size=1)

	initialize_parameters()
	formulate_H_bar()
	formulate_state_and_input_constraints()
	global received_input

	rate = rospy.Rate(60)

	while not rospy.is_shutdown():

		while not received_input:
			a = 0#rospy.spin()

		received_input = False

		formulate_dynamic_constraints()
		solve_cftoc()
		cftoc_output_pub.publish(cftoc_output)

		rate.sleep()

	


if __name__ == '__main__':
    try:
        cftoc_solver()
    except rospy.ROSInterruptException:
        pass
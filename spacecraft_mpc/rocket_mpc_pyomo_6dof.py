from pyomo.environ import *
from pyomo.dae import *

import pyutilib.subprocess.GlobalData
pyutilib.subprocess.GlobalData.DEFINE_SIGNAL_HANDLERS_DEFAULT = False

import numpy as np
import math
import time
import subprocess

from multiprocessing.connection import Client

import signal
import json

import tornado.ioloop
import tornado.web

subprocess.Popen("python rocket_mpc_graph.py")
address = ('localhost', 6000)
conn = Client(address, authkey=bytes('mpc', 'utf-8'))

transform_dae = TransformationFactory('dae.finite_difference')
solver = SolverFactory('ipopt')
solver.options['tol'] = 1 * 10**-3

def perform_mpc(u_init, v_init, w_init,
				p_init, q_init, r_init,
				phi_init, theta_init, psi_init,
				x_init, y_init, h_init,
				goal_x, goal_y, goal_h):
    # start mpc
	mpc_time = 1.0
	mpc_samples = 20

	m = ConcreteModel()
	m.t = ContinuousSet(bounds=(0, mpc_time))

	g = 196.2
	mass = 1413.72

	m.u = Var(m.t)
	m.v = Var(m.t)
	m.w = Var(m.t)

	m.p = Var(m.t)
	m.q = Var(m.t)
	m.r = Var(m.t)

	m.phi = Var(m.t)
	m.theta = Var(m.t)
	m.psi = Var(m.t)

	m.x = Var(m.t)
	m.y = Var(m.t)
	m.h = Var(m.t)

	I_xx = (1/12) * mass * 50**2
	I_yy = (1/12) * mass * 50**2
	I_zz = (1/2) * (mass * 1.5**2)

	m.F_z = Var(m.t)
	m.F_z_dot = DerivativeVar(m.F_z)
	m.L = Var(m.t)
	m.L_dot = DerivativeVar(m.L)
	m.M = Var(m.t)
	m.M_dot = DerivativeVar(m.M)
	m.N = Var(m.t)
	m.N_dot = DerivativeVar(m.N)

	m.u_dot = DerivativeVar(m.u)
	m.v_dot = DerivativeVar(m.v)
	m.w_dot = DerivativeVar(m.w)

	m.p_dot = DerivativeVar(m.p)
	m.q_dot = DerivativeVar(m.q)
	m.r_dot = DerivativeVar(m.r)

	m.phi_dot = DerivativeVar(m.phi)
	m.theta_dot = DerivativeVar(m.theta)
	m.psi_dot = DerivativeVar(m.psi)

	m.phi_dot_dot = DerivativeVar(m.phi_dot)
	m.theta_dot_dot = DerivativeVar(m.theta_dot)

	m.x_dot = DerivativeVar(m.x)
	m.y_dot = DerivativeVar(m.y)
	m.h_dot = DerivativeVar(m.h)

	m.x_dot_dot = DerivativeVar(m.x_dot)
	m.y_dot_dot = DerivativeVar(m.y_dot)
	m.h_dot_dot = DerivativeVar(m.h_dot)

	# odes!

	m.ode_u_dot = Constraint(m.t, rule=lambda m, t: m.u_dot[t] == -g * sin(m.theta[t]) + m.r[t]*m.v[t] - m.q[t]*m.w[t])
	m.ode_v_dot = Constraint(m.t, rule=lambda m, t: m.v_dot[t] == g * sin(m.phi[t])*cos(m.theta[t]) - m.r[t]*m.u[t] + m.p[t]*m.w[t])
	m.ode_w_dot = Constraint(m.t, rule=lambda m, t: m.w_dot[t] == (1/mass) * (-m.F_z[t]) + g*cos(m.phi[t])*cos(m.theta[t]) + m.q[t]*m.u[t] - m.p[t]*m.v[t])

	m.ode_p_dot = Constraint(m.t, rule=lambda m, t: m.p_dot[t] == (1/I_xx) * (m.L[t] + (I_yy - I_zz)*m.q[t]*m.r[t]))
	m.ode_q_dot = Constraint(m.t, rule=lambda m, t: m.q_dot[t] == (1/I_yy) * (m.M[t] + (I_zz - I_xx)*m.p[t]*m.r[t]))
	m.ode_r_dot = Constraint(m.t, rule=lambda m, t: m.r_dot[t] == (1/I_zz) * (m.N[t] + (I_xx - I_yy)*m.p[t]*m.q[t]))
	
	m.ode_phi_dot = Constraint(m.t, rule=lambda m, t: m.phi_dot[t] == m.p[t] + (m.q[t]*sin(m.phi[t]) + m.r[t]*cos(m.phi[t])) * tan(m.theta[t]))
	m.ode_theta_dot = Constraint(m.t, rule=lambda m, t: m.theta_dot[t] == m.q[t]*cos(m.phi[t]) - m.r[t]*sin(m.phi[t]))
	m.ode_psi_dot = Constraint(m.t, rule=lambda m, t: m.psi_dot[t] ==(m.q[t]*sin(m.phi[t]) + m.r[t]*cos(m.phi[t])) * (1/cos(m.theta[t])) )

	m.ode_x_dot = Constraint(m.t, rule=lambda m, t: m.x_dot[t] == cos(m.theta[t])*cos(m.psi[t])*m.u[t] + (-cos(m.phi[t])*sin(m.psi[t]) + sin(m.phi[t])*sin(m.theta[t])*cos(m.phi[t]))*m.v[t] + (sin(m.phi[t])*sin(m.psi[t]) + cos(m.phi[t])*sin(m.theta[t])*cos(m.psi[t]))*m.w[t])
	m.ode_y_dot = Constraint(m.t, rule=lambda m, t: m.y_dot[t] == cos(m.theta[t])*sin(m.psi[t])*m.u[t] + (cos(m.phi[t])*cos(m.psi[t]) + sin(m.phi[t])*sin(m.theta[t])*sin(m.phi[t]))*m.v[t] + (-sin(m.phi[t])*cos(m.psi[t]) + cos(m.phi[t])*sin(m.theta[t])*sin(m.psi[t]))*m.w[t])
	m.ode_h_dot = Constraint(m.t, rule=lambda m, t: m.h_dot[t] == -1 * (-sin(m.theta[t])*m.u[t] + sin(m.phi[t])*cos(m.theta[t])*m.v[t] + cos(m.phi[t])*cos(m.theta[t])*m.w[t]))

	m.const = ConstraintList()

	m.const.add(m.u[0]==u_init)
	m.const.add(m.v[0]==v_init)
	m.const.add(m.w[0]==w_init)
	m.const.add(m.p[0]==p_init)
	m.const.add(m.q[0]==q_init)
	m.const.add(m.r[0]==r_init)
	m.const.add(m.phi[0]==phi_init)
	m.const.add(m.theta[0]==theta_init)
	m.const.add(m.psi[0]==psi_init)
	m.const.add(m.x[0]==x_init)
	m.const.add(m.y[0]==y_init)
	m.const.add(m.h[0]==h_init)

	m.L_const1 = Constraint(m.t, rule=lambda m, t: m.L[t] <= 200000)
	m.L_const2 = Constraint(m.t, rule=lambda m, t: m.L[t] >= -200000)
	m.M_const1 = Constraint(m.t, rule=lambda m, t: m.M[t] <= 200000)
	m.M_const2 = Constraint(m.t, rule=lambda m, t: m.M[t] >= -200000)
	m.N_const1 = Constraint(m.t, rule=lambda m, t: m.N[t] <= 1000)
	m.N_const2 = Constraint(m.t, rule=lambda m, t: m.N[t] >= -1000)
	m.F_z_const1 = Constraint(m.t, rule=lambda m, t: m.F_z[t] <= g * mass * 2)
	m.F_z_const2 = Constraint(m.t, rule=lambda m, t: m.F_z[t] >= g * mass * 0.5)

	m.general_const_7 = Constraint(m.t, rule=lambda m, t: m.F_z_dot[t] <= 1000)
	m.general_const_8 = Constraint(m.t, rule=lambda m, t: m.F_z_dot[t] >= -1000)

	m.general_const_1 = Constraint(m.t, rule=lambda m, t: m.L_dot[t] <= 10)
	m.general_const_2 = Constraint(m.t, rule=lambda m, t: m.L_dot[t] >= -10)
	m.general_const_3 = Constraint(m.t, rule=lambda m, t: m.M_dot[t] <= 10)
	m.general_const_4 = Constraint(m.t, rule=lambda m, t: m.M_dot[t] >= -10)
	m.general_const_5 = Constraint(m.t, rule=lambda m, t: m.N_dot[t] <= 1)
	m.general_const_6 = Constraint(m.t, rule=lambda m, t: m.N_dot[t] >= -1)

	m.angle_slack_const = Var(m.t)
	m.angle_slack_const_const = Constraint(m.t, rule=lambda m, t: m.angle_slack_const[t] >= 0)

	m.general_const_9 = Constraint(m.t, rule=lambda m, t: m.phi[t] <= math.radians(30) + m.angle_slack_const[t])
	m.general_const_10 = Constraint(m.t, rule=lambda m, t: m.phi[t] >= -math.radians(30) + -m.angle_slack_const[t])
	m.general_const_11 = Constraint(m.t, rule=lambda m, t: m.theta[t] <= math.radians(30) + m.angle_slack_const[t])
	m.general_const_12 = Constraint(m.t, rule=lambda m, t: m.theta[t] >= -math.radians(30) + -m.angle_slack_const[t])

	m.h_dot_slack_const = Var(m.t)
	m.h_dot_slack_const_const = Constraint(m.t, rule=lambda m, t: m.h_dot_slack_const[t] >= 0)

	m.general_const_13 = Constraint(m.t, rule=lambda m, t: m.h_dot[t] <= 50 + m.h_dot_slack_const[t])
	m.general_const_14 = Constraint(m.t, rule=lambda m, t: m.h_dot[t] >= -100 + -m.h_dot_slack_const[t])

	m.slack_var_glide = Var(m.t)
	m.slack_var_glide_const = Constraint(m.t, rule=lambda m, t: m.slack_var_glide[t] >= 5)

	#m.general_const_15 = Constraint(m.t, rule=lambda m, t: m.h[t] - goal_h >= 2*((m.x[t] - goal_x)**2 + (m.y[t] - goal_y)**2 + 0.1)**0.5 - m.slack_var_glide[t])

	def cost_function(m, t):
		cost_total = 0
		cost_total = cost_total + 0.1*(m.h[t] - goal_h)**2
		cost_total = cost_total + 0.1*(m.h_dot[t])**2
		cost_total = cost_total + 0.001*(m.h_dot_dot[t])**2

		cost_total = cost_total + 0.12*((m.y[t] - goal_y) * 0.9)**2
		cost_total = cost_total + 0.1*(m.y_dot[t])**2
		cost_total = cost_total + 0.05*(m.y_dot_dot[t])**2

		cost_total = cost_total + 0.12*((m.x[t] - goal_x) * 0.9)**2
		cost_total = cost_total + 0.1*(m.x_dot[t])**2
		cost_total = cost_total + 0.05*(m.x_dot_dot[t])**2

		cost_total = cost_total + 1.0*(m.phi[t])**2
		cost_total = cost_total + 1.0*(m.theta[t])**2

		cost_total = cost_total + 600.0*(m.phi_dot[t])**2
		cost_total = cost_total + 40.0*(m.phi_dot_dot[t])**2
		cost_total = cost_total + 600.0*(m.theta_dot[t])**2
		cost_total = cost_total + 40.0*(m.theta_dot_dot[t])**2

		cost_total = cost_total + 4000.0*(m.psi[t])**2
		cost_total = cost_total + 300.0*(m.psi_dot[t])**2

		cost_total = cost_total + 30.0*(m.angle_slack_const[t])**2
		cost_total = cost_total + 20.0*(m.h_dot_slack_const[t])**2

		cost_total = cost_total + 0.01*(m.slack_var_glide[t])**2

		cost_total = cost_total + 100.0*(m.F_z[t] / (g * mass * 10))**2

		return cost_total

	m.integral = Integral(m.t, wrt=m.t, rule=cost_function)
	m.obj = Objective(expr=m.integral)

	transform_dae.apply_to(m, wrt=m.t, nfe=mpc_samples)

	start = time.time()
	solver.solve(m, tee = False)
	end = time.time()

	print("solves per sec: " + str(1/(end - start)))

	t_res = np.array([t for t in m.t]).tolist()
	u_res = np.array([m.u[t]() for t in m.t]).tolist()
	v_res = np.array([m.v[t]() for t in m.t]).tolist()
	w_res = np.array([m.w[t]() for t in m.t]).tolist()
	p_res = np.array([m.p[t]() for t in m.t]).tolist()
	q_res = np.array([m.q[t]() for t in m.t]).tolist()
	r_res = np.array([m.r[t]() for t in m.t]).tolist()
	phi_res = np.array([m.phi[t]() for t in m.t]).tolist()
	theta_res = np.array([m.theta[t]() for t in m.t]).tolist()
	psi_res = np.array([m.psi[t]() for t in m.t]).tolist()
	x_res = np.array([m.x[t]() for t in m.t]).tolist()
	y_res = np.array([m.y[t]() for t in m.t]).tolist()
	h_res = np.array([m.h[t]() for t in m.t]).tolist()

	F_z_res = np.array([m.F_z[t]() for t in m.t]).tolist()
	L_res = np.array([m.L[t]() for t in m.t]).tolist()
	M_res = np.array([m.M[t]() for t in m.t]).tolist()
	N_res = np.array([m.N[t]() for t in m.t]).tolist()

	use_graph = 1
	if (use_graph == 1):
		conn.send(json.dumps([t_res, x_res, y_res, h_res, F_z_res, L_res, M_res, N_res]))

	return [F_z_res, L_res, M_res, N_res]

for x in range(0):
	start = time.time()
	perform_mpc(0, 0, 0, 0, 0, 0, 0, 0, 0, x, 0, 0)
	end = time.time()
	print("solves per sec: " + str(1/(end - start)))

class MainHandler(tornado.web.RequestHandler):
	is_closing = False
	def get(self):

		u_init = float(self.get_argument("u_init"))
		v_init = float(self.get_argument("v_init"))
		w_init = float(self.get_argument("w_init"))
		p_init = float(self.get_argument("p_init"))
		q_init = float(self.get_argument("q_init"))
		r_init = float(self.get_argument("r_init"))

		phi_init = float(self.get_argument("phi_init"))
		theta_init = float(self.get_argument("theta_init"))
		psi_init = float(self.get_argument("psi_init"))
		x_init = float(self.get_argument("x_init"))
		y_init = float(self.get_argument("y_init"))
		h_init = float(self.get_argument("h_init"))

		goal_x = float(self.get_argument("goal_x"))
		goal_y = float(self.get_argument("goal_y"))
		goal_h = float(self.get_argument("goal_h"))

		start = time.time()
		u_values = perform_mpc(u_init, v_init, w_init, p_init, q_init, r_init,
								phi_init, theta_init, psi_init, x_init, y_init, h_init,
								goal_x, goal_y, goal_h)
		end = time.time()

		u_values_json = json.dumps(u_values)
		self.write(u_values_json)

class PyomoServe(tornado.web.Application):
	is_closing = False
	def signal_handler(self, signum, frame):
		print('exiting...')
		self.is_closing = True

	def try_exit(self):
		if self.is_closing:
			tornado.ioloop.IOLoop.instance().stop()
			print('exit success')

def make_web_app():
	return PyomoServe([
		(r"/", MainHandler),
    ])

if __name__ == "__main__":
	if True:
		app = make_web_app()
		app.listen(7777)

		signal.signal(signal.SIGINT, app.signal_handler)
		tornado.ioloop.PeriodicCallback(app.try_exit, 100).start()
		tornado.ioloop.IOLoop.instance().start()
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

def perform_mpc(x_init, y_init, pitch_init, 
				x_dot_init, y_dot_init, pitch_dot_init, goal_x, goal_y):
    # start mpc
	mpc_time = 2
	mpc_samples = 20

	m = ConcreteModel()
	m.t = ContinuousSet(bounds=(0, mpc_time))

	mass_c = 50
	grav_c = 196.2
	I_pitch_c = (1/12) * mass_c * mass_c**2

	m.fuel = Var(m.t)

	m.u1 = Var(m.t)
	m.u2 = Var(m.t)

	m.u1_dot = DerivativeVar(m.u1)
	m.u2_dot = DerivativeVar(m.u2)

	m.x = Var(m.t)
	m.y = Var(m.t)
	m.pitch = Var(m.t)
	
	m.x_dot = DerivativeVar(m.x)
	m.y_dot = DerivativeVar(m.y)
	m.pitch_dot = DerivativeVar(m.pitch)

	m.x_dot_dot = DerivativeVar(m.x_dot)
	m.y_dot_dot = DerivativeVar(m.y_dot)
	m.pitch_dot_dot = DerivativeVar(m.pitch_dot)

	m.ode_y_dot_dot = Constraint(m.t, rule=lambda m, t: m.y_dot_dot[t] == (m.u2[t] / mass_c) * cos(m.pitch[t]) - grav_c)
	m.ode_x_dot_dot = Constraint(m.t, rule=lambda m, t: m.x_dot_dot[t] == (m.u2[t] / mass_c) * -sin(m.pitch[t]))
	m.ode_pitch_dot_dot = Constraint(m.t, rule=lambda m, t: m.pitch_dot_dot[t] == m.u1[t] / I_pitch_c)
	m.ode_fuel = Constraint(m.t, rule=lambda m, t: m.fuel[t] == m.u2[t]) 

	m.const = ConstraintList()
	m.const.add(m.x[0]==x_init)
	m.const.add(m.y[0]==y_init)
	m.const.add(m.pitch[0]==pitch_init)

	m.const.add(m.x_dot[0]==x_dot_init)
	m.const.add(m.y_dot[0]==y_dot_init)
	m.const.add(m.pitch_dot[0]==pitch_dot_init)

	m.u1_const_1 = Constraint(m.t, rule=lambda m, t: m.u1[t] <= 2000)
	m.u1_const_2 = Constraint(m.t, rule=lambda m, t: m.u1[t] >= -2000)
	m.u2_const_1 = Constraint(m.t, rule=lambda m, t: m.u2[t] <= 1000000)
	m.u2_const_2 = Constraint(m.t, rule=lambda m, t: m.u2[t] >= 2000)

	m.u1_const_dot_1 = Constraint(m.t, rule=lambda m, t: m.u1_dot[t] <= 300)
	m.u1_const_dot_2 = Constraint(m.t, rule=lambda m, t: m.u1_dot[t] >= -300)
	m.u2_const_dot_1 = Constraint(m.t, rule=lambda m, t: m.u2_dot[t] <= 1)
	m.u2_const_dot_2 = Constraint(m.t, rule=lambda m, t: m.u2_dot[t] >= -1)


	m.geo_const1 = Constraint(m.t, rule=lambda m, t: m.y_dot[t] <= 800)
	m.geo_const2 = Constraint(m.t, rule=lambda m, t: m.y_dot[t] >= -800)

	m.geo_const3 = Constraint(m.t, rule=lambda m, t: m.y_dot_dot[t] <= 100)
	m.geo_const4 = Constraint(m.t, rule=lambda m, t: m.y_dot_dot[t] >= -100)

	m.geo_const5 = Constraint(m.t, rule=lambda m, t: m.pitch[t] <= math.radians(20))
	m.geo_const6 = Constraint(m.t, rule=lambda m, t: m.pitch[t] >= math.radians(-20))


	def cost_function(m, t):
		cost_total = 0

		cost_total = cost_total + 100.0*(m.y[t] - goal_y)**2
		cost_total = cost_total + 150.0*(m.y_dot[t])**2

		cost_total = cost_total + 200.0*(m.x[t] - goal_x)**2
		cost_total = cost_total + 300.0*(m.x_dot[t])**2
		cost_total = cost_total + 100.0*(m.x_dot_dot[t])**2

		cost_total = cost_total + 1.0*(m.pitch[t]) ** 2
		cost_total = cost_total + 80.0*(m.pitch_dot[t]) ** 2
		cost_total = cost_total + 50.0*(m.pitch_dot_dot[t]) ** 2

		cost_total = cost_total + 0.001*(m.fuel[t])**2

		return cost_total

	m.integral = Integral(m.t, wrt=m.t, rule=cost_function)
	m.obj = Objective(expr=m.integral)

	TransformationFactory('dae.finite_difference').apply_to(m, wrt=m.t, nfe=mpc_samples)
	SolverFactory('ipopt').solve(m)

	t_res = np.array([t for t in m.t]).tolist()
	x_res = np.array([m.x[t]() for t in m.t]).tolist()
	y_res = np.array([m.y[t]() for t in m.t]).tolist()
	pitch_res = np.array([m.pitch[t]() for t in m.t]).tolist()

	u1_res = np.array([m.u1[t]() for t in m.t]).tolist()
	u2_res = np.array([m.u2[t]() for t in m.t]).tolist()

	use_graph = 1
	if (use_graph == 1):
		conn.send(json.dumps([t_res, x_res, y_res, pitch_res, u1_res, u2_res]))

	print(pitch_res)

	return [u1_res, u2_res]

#perform_mpc(0, 0, 0, math.radians(-90), math.radians(0), 
#			0, 0, 0, 0, 0)

class MainHandler(tornado.web.RequestHandler):
	is_closing = False
	def get(self):
		x_init = float(self.get_argument("x_init"))
		y_init = float(self.get_argument("y_init"))
		pitch_init = float(self.get_argument("pitch_init"))

		x_dot_init = float(self.get_argument("x_dot_init"))
		y_dot_init = float(self.get_argument("y_dot_init"))
		pitch_dot_init = float(self.get_argument("pitch_dot_init"))

		goal_x = float(self.get_argument("goal_x"))
		goal_y = float(self.get_argument("goal_y"))

		start = time.time()
		u_values = perform_mpc(x_init, y_init, pitch_init,
								x_dot_init, y_dot_init, pitch_dot_init,
								goal_x, goal_y)
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
local sin = math.sin
local cos = math.cos
local tan = math.tan

local u = 0 -- body velocity
local v = 0
local w = 0

local p = 0 -- body roll rate
local q = 0 -- body pitch rate
local r = 0 -- body yaw rate

local phi = 0
local theta = math.rad(90)
local psi = 0

local x = 0
local y = 0
local h = 0

local function update(dt)
	local g = 9.81
	local m = 1
	
	local I_xx = 1
	local I_yy = 1
	local I_zz = 1

	local F_x = 0
	local F_y = 0
	local F_z = 0

	local L = 0
	local M = 0
	local N = 0
		
	local u_dot = -g * sin(theta) + r*v - q*w
	local v_dot = g * sin(phi)*cos(theta) - r*u + p*w
	local w_dot = (1/m) * (-F_z) + g*cos(phi)*cos(theta) + q*u - p*v
	
	local p_dot = (1/I_xx) * (L + (I_yy - I_zz)*q*r)
	local q_dot = (1/I_yy) * (M + (I_zz - I_xx)*p*r)
	local r_dot = (1/I_zz) * (N + (I_xx - I_yy)*p*q)

	local phi_dot = p + (q*sin(phi) + r*cos(phi)) * tan(theta)
	local theta_dot = q*cos(phi) - r*sin(phi)
	local psi_dot = (q*sin(phi) + r*cos(phi)) * (1/cos(theta))

	local x_dot = cos(theta)*cos(psi)*u + (-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(phi))*v + (sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi))*w
	local y_dot = cos(theta)*sin(psi)*u + (cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(phi))*v + (-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi))*w
	local h_dot = -1 * (-sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w)

	print(u_dot, v_dot, w_dot)
	print(p_dot, q_dot, r_dot)
	print(phi_dot, theta_dot, psi_dot)
	print(x_dot, y_dot, h_dot)
	print("---")
	print(u, v, w)
	print(p, q, r)
	print(phi, theta, psi)
	print(x, y, h)

	print("------------")

	u = u + u_dot * dt
	v = v + v_dot * dt
	w = w + w_dot * dt

	p = p + p_dot * dt
	q = q + q_dot * dt
	r = r + r_dot * dt

	phi = phi + phi_dot * dt
	theta = theta + theta_dot * dt
	psi = psi + psi_dot * dt

	x = x + x_dot * dt
	y = y + y_dot * dt
	h = h + h_dot * dt
end

local dt = 0.01
for i=0, 1.0, dt do
	update(dt)
end
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

from multiprocessing.connection import Listener
import json

fig = plt.figure(figsize=plt.figaspect(1.5))
def plot(m_1, x, y, h, F_z, L, M, N):
	plt.clf()

	ax = fig.add_subplot(3, 1, 1, projection='3d')
	ax.plot3D(x, y, h, 'gray')
	ax.set_xlim3d(-10 + x[0], 10 + x[0])
	ax.set_ylim3d(-10 + y[0], 10 + y[0])
	ax.set_zlim3d(-10 + h[0], 10 + h[0])

	plt.subplot(3, 1, 2)
	#plt.plot(m_1,F_z,'',label='F_z')
	plt.plot(m_1,L,'',label='L')
	plt.plot(m_1,M,'',label='M')
	plt.plot(m_1,N,'',label='N')
	#plt.ylim(-20000, 20000)
	plt.legend(loc='best')

	plt.subplot(3, 1, 3)
	plt.plot(m_1,x,'',label='x')
	plt.plot(m_1,y,'',label='y')
	plt.plot(m_1,h,'',label='h')
	plt.legend(loc='best')

	plt.pause(0.001)

address = ('localhost', 6000)
listener = Listener(address, authkey=bytes('mpc', 'utf-8'))
conn = listener.accept()
print("accepted")
while True:
    msg = conn.recv()
    decoded_json = json.loads(msg)
    plot(decoded_json[0], decoded_json[1], decoded_json[2], decoded_json[3], decoded_json[4], decoded_json[5], decoded_json[6], decoded_json[7])

listener.close()
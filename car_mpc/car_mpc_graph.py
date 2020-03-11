import matplotlib.pyplot as plt

from multiprocessing.connection import Listener
import json

def plot(m_1, x, y, cte, delta, v):
	plt.clf()
	#plt.figure()
	plt.subplot(2,1,1)
	plt.plot(x,y,'',label='path')
	#plt.gcf().gca().add_artist(plt.Circle((50, - 1), 7.5, color='r'))
	plt.xlim(-5, 110)
	plt.ylim(-100, 100)
	plt.gca().set_aspect('equal', adjustable='box')
	plt.legend()
	plt.subplot(2,1,2)
	plt.plot(m_1,x,'',label='x')
	plt.plot(m_1,y,'',label='y')
	plt.plot(m_1,cte,'',label='cte')
	plt.plot(m_1,delta,'',label='delta')
	plt.plot(m_1,v,'',label='Velocity')
	plt.legend(loc='best')
	plt.pause(0.01)

address = ('localhost', 6000)
listener = Listener(address, authkey=bytes('mpc', 'utf-8'))
conn = listener.accept()
print("accepted")
while True:
    msg = conn.recv()
    decoded_json = json.loads(msg)
    plot(decoded_json[0], decoded_json[1], decoded_json[2], decoded_json[3], decoded_json[4], decoded_json[5])

listener.close()
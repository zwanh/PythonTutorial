import socket
import time
host = '192.168.199.161'
port = 31501
def Client():
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((host, port))
	a = raw_input("Please input something:\n")
	sock.send(a)
	time.sleep(0.5)
	sock.send('1')
	print sock.recv(1024)
	sock.close()

if __name__ == '__main__':
	Client()

import socket
host = '192.168.199.246'
port = 31500
def Server():
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.bind((host, port))
	sock.listen(1)

	while True:
		connection, address = sock.accept()
		print connection, address
		try:
			connection.settimeout(3)
			buf = connection.recv(1024)
			print buf
			if buf == '1':
				connection.send('Welcome to server!')
			else:
				connection.send('Goodbye!')
		except socket.timeout:
			print 'socket timeout'
		connection.close()

if __name__ == '__main__':
	Server()


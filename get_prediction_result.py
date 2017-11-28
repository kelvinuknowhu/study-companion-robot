import socket

if __name__ == '__main__':
    s = socket.socket()
    ip = '127.0.0.1'
    port = 20000
    s.connect((ip, port))
    result = s.recv(1024)
    print 'I received a result: %s' % result
    s.close()
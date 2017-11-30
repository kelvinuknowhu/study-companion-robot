import socket

if __name__ == '__main__':
    s = socket.socket()
    ip = '192.168.199.1'
    port = 20000
    s.connect((ip, port))
    result = s.recv(1024)
    print 'I received a result: %s' % result
    s.close()
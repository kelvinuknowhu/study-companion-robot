import socket

if __name__ == '__main__':
    result = 'Distracted'

    # Provides socket connection for the robot manipulation script
    s = socket.socket()
    ip = '127.0.0.1'
    port = 20000
    s.bind(ip, port)
    s.listen()

    c, addr = s.accept()
    c.send(result)

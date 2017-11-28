import socket

if __name__ == '__main__':
    result = 'Distracted'

    # Provides socket connection for the robot manipulation script
    s = socket.socket()
    ip = '127.0.0.1'
    port = 20000
    s.bind((ip, port))
    s.listen(5)
    print 'Server running at (\'127.0.0.1\', 20000) is ready for the robot to poll the prediction result'
    c, addr = s.accept()
    c.send(result)
    c.close()
    s.close()    

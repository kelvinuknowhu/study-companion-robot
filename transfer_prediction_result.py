import socket

if __name__ == '__main__':

    # Provides prediction service for the robot manipulation script
    s = socket.socket()
    ip = ''
    port = 20000
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ip, port))
    s.listen(5)
    print 'Server running at (\'127.0.0.1\', 20000) is ready for the robot to poll the prediction result'
    c, addr = s.accept()
    while True:
        cmd = raw_input('Press d to send Distracted, n to send Not Distracted, q to Quit: ')
        if cmd == 'd':
            c.send('Distracted')
        elif cmd == 'n':
            c.send('Not Distracted')
        elif cmd == 'q':
            break;
    c.close()
    s.close()    

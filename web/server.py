import tornado
import tornado.ioloop
import tornado.web
import tornado.websocket

import os

PATH = os.path.dirname(os.path.realpath(__file__))

WEBSOCKETS = set()


class Application():
    
    def __init__(self):
        
        self.port = 8888                
        self.static_root = os.path.join(PATH)
        
        handlers = [(r'/', MainHandler),
                    (r'/websocket', WebSocketHandler),
                    (r'/(.*)', tornado.web.StaticFileHandler, {'path': self.static_root})]

        application = tornado.web.Application(handlers)
        application.listen(self.port)
        tornado.ioloop.IOLoop.instance().start()

    
    def WS_send_message(message):
        removable = set()
        for ws in WEBSOCKETS:
            if not ws.ws_connection or not ws.ws_connection.stream.socket:
                removable.add(ws)
            else:
                ws.write_message(message)
                
        for ws in removable:
            WEBSOCKETS.remove(ws)    
    
    
    
    
    
    

class MainHandler(tornado.web.RequestHandler):
    
    def post(self):
        self.write("Hello, world")
        
        
        
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    
    def open(self):
        print("WebSocket opened")
        WEBSOCKETS.add(self)

    def on_message(self, message):
        self.write_message(u"You said: " + message)

    def on_close(self):
        print("WebSocket closed")
        
    def check_origin(self, origin):
        return True
        


if __name__=='__main__':
    
    app = Application()
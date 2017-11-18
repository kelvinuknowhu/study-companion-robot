import tornado
import tornado.ioloop
import tornado.web
import tornado.websocket

import os
import json

PATH = os.path.dirname(os.path.realpath(__file__))

WEBSOCKETS = set()


class Application():
    
    def __init__(self):
        self.port = 8889                
        self.static_root = os.path.join(PATH)
        
        handlers = [(r'/', MainHandler),
                    (r'/get-facial-expression', RequstFacialExpressionHandler),
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
            
class RequstFacialExpressionHandler(tornado.web.RequestHandler):
    def post(self):
        WS_send_message("request_facial_expression")
        
class MainHandler(tornado.web.RequestHandler):
    def post(self):
        self.write("Hello, world")    
    
class WebSocketHandler(tornado.websocket.WebSocketHandler):    
    def open(self):
        print("WebSocket opened")
        WEBSOCKETS.add(self)

    def on_message(self, message):
                
        if message is None:
            return
        elif len(message)==0:
            return
        else:
            facialExpression = json.loads(message)        
            data_file = os.path.join(PATH,"facial_expressions.txt")
            try:
                with open(data_file, 'a') as f: 
                    f.write('attention:{0}\n'.format(facialExpression['attention']))
                    f.write('browFurrow:{0}\n'.format(facialExpression['browFurrow']))
                    f.write('browRaise:{0}\n'.format(facialExpression['browRaise']))
                    f.write('cheekRaise:{0}\n'.format(facialExpression['cheekRaise']))
                    f.write('chinRaise:{0}\n'.format(facialExpression['chinRaise']))
                    f.write('dimpler:{0}\n'.format(facialExpression['dimpler']))
                    f.write('eyeClosure:{0}\n'.format(facialExpression['eyeClosure']))
                    f.write('eyeWiden:{0}\n'.format(facialExpression['eyeWiden']))
                    f.write('innerBrowRaise:{0}\n'.format(facialExpression['innerBrowRaise']))
                    f.write('jawDrop:{0}\n'.format(facialExpression['jawDrop']))
                    f.write('lidTighten:{0}\n'.format(facialExpression['lidTighten']))
                    f.write('lipCornerDepressor:{0}\n'.format(facialExpression['lipCornerDepressor']))
                    f.write('lipPress:{0}\n'.format(facialExpression['lipPress']))
                    f.write('lipPucker:{0}\n'.format(facialExpression['lipPucker']))
                    f.write('lipSuck:{0}\n'.format(facialExpression['lipSuck']))
                    f.write('mouthOpen:{0}\n'.format(facialExpression['mouthOpen']))
                    f.write('noseWrinkle:{0}\n'.format(facialExpression['noseWrinkle']))
                    f.write('smile:{0}\n'.format(facialExpression['smile']))
                    f.write('smirk:{0}\n'.format(facialExpression['smirk']))
                    f.write('upperLipRaise:{0}\n'.format(facialExpression['upperLipRaise']))
            except IOError as e:
                print(str(e))
            
    def on_close(self):
        print("WebSocket closed")
        
    def check_origin(self, origin):
        return True
        
if __name__=='__main__':
    app = Application()
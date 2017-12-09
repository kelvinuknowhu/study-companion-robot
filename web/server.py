import tornado
import tornado.ioloop
import tornado.web
import tornado.websocket
import fcntl
import os
import json

PATH = os.path.dirname(os.path.realpath(__file__))

PORT = 8899

WEBSOCKETS = set()

useEmotion = False


class Application():
    
    def __init__(self):
        self.port = PORT                
        self.static_root = os.path.join(PATH)
        print("Server running...")

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
        elif len(message) == 0:
            return
        else:
            data = json.loads(message)
            if data is not None:
                print(data)       
                data_file = os.path.join(PATH, "facial_expressions.txt")
                try:
                    with open(data_file, 'w') as f:
                        fcntl.lockf(f, fcntl.LOCK_EX)
                        if useEmotion:
                            f.write('joy:{0}\n'.format(data['joy']))
                            f.write('sadness:{0}\n'.format(data['sadness']))
                            f.write('disgust:{0}\n'.format(data['disgust']))
                            f.write('contempt:{0}\n'.format(data['contempt']))
                            f.write('anger:{0}\n'.format(data['anger']))
                            f.write('fear:{0}\n'.format(data['fear']))
                            f.write('surprise:{0}\n'.format(data['surprise']))
                            f.write('valence:{0}\n'.format(data['valence']))
                            f.write('engagement:{0}\n'.format(data['engagement']))
                        else:                    
                            f.write('attention:{0}\n'.format(data['attention']))
                            f.write('browFurrow:{0}\n'.format(data['browFurrow']))
                            f.write('browRaise:{0}\n'.format(data['browRaise']))
                            f.write('cheekRaise:{0}\n'.format(data['cheekRaise']))
                            f.write('chinRaise:{0}\n'.format(data['chinRaise']))
                            f.write('dimpler:{0}\n'.format(data['dimpler']))
                            f.write('eyeClosure:{0}\n'.format(data['eyeClosure']))
                            f.write('eyeWiden:{0}\n'.format(data['eyeWiden']))
                            f.write('innerBrowRaise:{0}\n'.format(data['innerBrowRaise']))
                            f.write('jawDrop:{0}\n'.format(data['jawDrop']))
                            f.write('lidTighten:{0}\n'.format(data['lidTighten']))
                            f.write('lipCornerDepressor:{0}\n'.format(data['lipCornerDepressor']))
                            f.write('lipPress:{0}\n'.format(data['lipPress']))
                            f.write('lipPucker:{0}\n'.format(data['lipPucker']))
                            f.write('lipSuck:{0}\n'.format(data['lipSuck']))
                            f.write('mouthOpen:{0}\n'.format(data['mouthOpen']))
                            f.write('noseWrinkle:{0}\n'.format(data['noseWrinkle']))
                            f.write('smile:{0}\n'.format(data['smile']))
                            f.write('smirk:{0}\n'.format(data['smirk']))
                            f.write('upperLipRaise:{0}\n'.format(data['upperLipRaise']))
                        fcntl.lockf(f, fcntl.LOCK_UN)
                        f.close()
                except IOError as e:
                    print(str(e))
            
    def on_close(self):
        print("WebSocket closed")
        
    def check_origin(self, origin):
        return True
        
if __name__=='__main__':
    app = Application()
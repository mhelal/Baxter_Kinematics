





#!/usr/bin/env python2

#import asyncore
#import socket
#import rospy
#from Game import setPlayer, calibrate_board, set_move
#from baxter_core_msgs.msg import NavigatorState
"""
def talker(data):
	navstate = NavigatorState()
	navstate.button_names = ['ok', 'back', 'show']
	navstate.buttons = [True, False, False]
	navstate.wheel = int(data)
	navstate.light_names = ['inner', 'outer']
	navstate.lights = [False, False]

	pub = rospy.Publisher('/robot/navigators/right_navigator/state', NavigatorState, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	if not rospy.is_shutdown(): 
		rospy.loginfo(navstate)
		pub.publish(navstate)

class EchoHandler(asyncore.dispatcher_with_send):

    def handle_read(self):
        data = self.recv(8192)
        print(data)
        talker(data)
        self.send(data)

class EchoServer(asyncore.dispatcher):

    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)

    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print 'Incoming connection from %s' % repr(addr)
            handler = EchoHandler(sock)

server = EchoServer("0.0.0.0", 11000)
asyncore.loop()
"""

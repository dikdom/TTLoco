#!/usr/bin/python
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from os import curdir, sep
from io import TextIOWrapper
from ardpi import ArdPi
import json

PORT_NUMBER = 8080

#This class will handles any incoming request from
#the browser

ardpi = ArdPi(device="/dev/ttyUSB0")

class myHandler(BaseHTTPRequestHandler):
	def do_POST(self):
		print "POST on: " + self.path
		try:
			data=self.rfile.readline(int(self.headers.getheader('Content-Length')))
			jsondata=json.loads(data)
			if self.path == "/slider":
				if 'slider' in jsondata.keys():
					ardpi.changeSpeed(track=jsondata['slider'], speed=int(float(jsondata['value'][0])))
				if 'servo' in jsondata.keys():
					ardpi.changeServo(jsondata['servo'])
			elif self.path == "/servo":
				ardpi.changeServo(servo=jsondata['value'])
			elif self.path == "/switch":
				ardpi.changeSwitch(switch=jsondata['value'])

		except IOError as err:
			print "failed: " + err
		self.send_response(200)
		self.end_headers()

	#Handler for the GET requests
	def do_GET(self):
		if self.path=="/":
			self.path="/index.html"

		try:
			#Check the file extension required and
			#set the right mime type

			sendReply = False
			if self.path.endswith(".html"):
				mimetype='text/html'
				sendReply = True
			if self.path.endswith(".jpg"):
				mimetype='image/jpg'
				sendReply = True
			if self.path.endswith(".gif"):
				mimetype='image/gif'
				sendReply = True
			if self.path.endswith(".js"):
				mimetype='application/javascript'
				sendReply = True
			if self.path.endswith(".css"):
				mimetype='text/css'
				sendReply = True

			if sendReply == True:
				#Open the static file requested and send it
				f = open(curdir + sep + self.path) 
				self.send_response(200)
				self.send_header('Content-type',mimetype)
				self.end_headers()
				self.wfile.write(f.read())
				f.close()
			else:
				self.send_response(404)
				self.end_headers()
			return


		except IOError:
			self.send_error(404,'File Not Found: %s' % self.path)

try:
	#Create a web server and define the handler to manage the
	#incoming request
	server = HTTPServer(('', PORT_NUMBER), myHandler)
	print 'Started httpserver on port ' , PORT_NUMBER
	
	#Wait forever for incoming htto requests
        
	server.serve_forever()

except KeyboardInterrupt:
	print '^C received, shutting down the web server'
	server.socket.close()
	ardpi.stop()


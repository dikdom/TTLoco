from threading import Thread
from serial import Serial
from time import sleep

class ArdPi:
  def __init__(self, device="/dev/USB", baudrate="9600"):
    self.initialized=False
    self.keepon = True
    self.baudrate = baudrate
    self.device = device
    self.ser = None
    self.reader = Thread(target=self.mainloop)
    self.reader.start()

  def init(self):
    print "ard-pi: initializing USB..."
    while self.keepon and not self.initialized:
      try:
        self.ser = Serial(port=self.device, baudrate=self.baudrate, timeout=1)
        print "ard-pi: USB initialized!"
        self.initialized = True
      except:
        print "ard-pi: USB failed to init"
        sleep(3)

  def stop(self):
    self.keepon = False
    if self.ser:
      self.ser.close()

  def readit(self):
    try:
      while self.keepon:
        line=self.ser.readline()
        if line != "":
          print "ard->pi:" + line
    except:
      print "ard-pi: USB connection lost"
      self.ser.close()

  def mainloop(self):
    while self.keepon:
      self.initialized = False
      self.init()
      self.readit()

  def changeSpeed(self, track, speed):
    print "ard-pi: changeSpeed called with speed: %d, track: %d" % (speed, track)
    if self.initialized:
      command="tr: %d %d\n" % (track, speed)
      self.ser.write(command)
      print "pi->ard:" + command
    else:
      print "ard-pi: not initialized yet!"

  def changeServo(self, servo):
    print "ard-pi: changeServo called with servo: %d" % servo
    if self.initialized:
      command="se: %d\n" % servo
      self.ser.write(command)
      print "pi->ard:" + command
    else:
      print "ard-pi: not initialized yet"

  def changeSwitch(self, switch):
    print "ard-pi: changeSwitch called with switch: %d" % switch
    if self.initialized:
      command="sw: %d\n" % switch
      self.ser.write(command)
      print "pi->ard:" + command
    else:
      print "ard-pi: not initialized yet"


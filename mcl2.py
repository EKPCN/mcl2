#!/usr/bin/python
import os
import sys
import serial
import time
import random
import json

class MCL2(object):
    """ Lang MCL2 Motor Control 

    x_max = 800000
    y_max = 400000
    
    """

    def __init__(self,cfgFile="config.cfg"):
        
        self.cfgFile = cfgFile

        try:
            self.cfg = json.load(open(cfgFile))
        except Exception:
            self.cfg = self.getDefault()
            self.cfgFile = 'config.cfg'
            self.write()
            
        self.__ser = serial.Serial(self.cfg['Serial']['port'],
                                   self.cfg['Serial']['baudrate'],
                                   self.cfg['Serial']['bytesize'],
                                   self.parity(self.cfg['Serial']['parity']),
                                   self.stopbits(self.cfg['Serial']['stopbits']))
 
        self.setPos(self.cfg['Current']['x'],self.cfg['Current']['y'])

    def getDefault(self):
        dic = { "Serial":{ "port" : '/dev/ttyUSB1',
                           "baudrate" : 2400,
                           "parity" : None,
                           "stopbits" : 2,
                           "bytesize" : 8
                           },
                "HardLimits": { "xmin" : 0,
                                "xmax" : 800000,
                                "ymin" : 0,
                                "ymax" : 400000
                                },
                "SoftLimits": { "xmin" : 0,
                                "xmax" : 800000,
                                "ymin" : 0,
                                "ymax" : 400000,
                                "margin" : 500
                                },
                "Home" : { 'x' : 0,
                           'y' : 0
                           },
                
                "Current" : { 'x' : 0,
                              'y' : 0
                              },
                
                "Conversion" : { 'x' : 4,
                                 'y' : 4
                                 }
                }
        return dic

    def parity(self, parity):
        if parity == "Odd":
            return serial.PARITY_ODD
        elif parity == "Even":
            return serial.PARITY_EVEN
        elif parity == "Mark":
            return serial.PARITY_MARK
        if parity == "SPACE":
            return serial.PARITY_SPACE
        else:
            return serial.PARITY_NONE


    def stopbits(self,stopBits):
        if stopBits == 1:
            return serial.STOPBITS_ONE
        elif stopBits == 1.5:
            return serial.STOPBITS_ONE_POINT_FIVE
        else:
            return serial.STOPBITS_TWO

    # Execute (move, calibrate, measureXY)
    def execute(self):
        self.__ser.write(str('U'+chr(80)+'\r'))
        try:
            msg = self.readMCL()
        except KeyboardInterrupt:
            self.abort()
            print("Aborted!")
            self.updatePos()
            raise

        return msg

    def abort(self):
        self.__ser.write(str('a\r'))
        time.sleep(0.1)
        return self.readMCL()

    # Calibration
    def calibrate(self):
        self.__ser.write('U'+chr(7)+'c\r')
        self.execute()
        self.cfg["HardLimits"]["xmin"], self.cfg["HardLimits"]["ymin"] \
            = self.getPos()
        self.write()

    def measureXY(self):
        self.__ser.write('U'+chr(7)+'l\r')
        self.execute()
        self.cfg["HardLimits"]["xmax"], self.cfg["HardLimits"]["ymax"] \
            = self.getPos()
        self.write()

    def fullCalibration(self):
        self.calibrate()        
        self.moveTicks(self.cfg['SoftLimits']['margin'],
                       self.cfg['SoftLimits']['margin'],)

        self.setPos(0,0)
        self.cfg["HardLimits"]["xmin"] -= self.cfg["SoftLimits"]["margin"]
        self.cfg["HardLimits"]["ymin"] -= self.cfg["SoftLimits"]["margin"]
        self.cfg["SoftLimits"]["xmin"] = self.cfg["Current"]["x"]
        self.cfg["SoftLimits"]["ymin"] = self.cfg["Current"]["y"]
        self.write()

        self.measureXY()
        self.cfg["SoftLimits"]["xmax"] = self.cfg["Current"]["x"] \
                                         -self.cfg["SoftLimits"]["margin"]
        self.cfg["SoftLimits"]["ymax"] = self.cfg["Current"]["y"] \
                                         -self.cfg["SoftLimits"]["margin"]
    
        self.write()
        self.center()

        print("\nCalibration finished!\n")
        print("Hard limits x: %s - %s" %(self.cfg["HardLimits"]["xmin"], 
                                         self.cfg["HardLimits"]["xmax"]))
        print("Hard limits y: %s - %s" %(self.cfg["HardLimits"]["ymin"], 
                                         self.cfg["HardLimits"]["ymax"]))
        print("Soft limits x: %s - %s" %(self.cfg["SoftLimits"]["xmin"], 
                                         self.cfg["SoftLimits"]["xmax"]))
        print("Soft limits y: %s - %s\n" %(self.cfg["SoftLimits"]["ymin"], 
                                           self.cfg["SoftLimits"]["ymax"]))
        self.printPos()

    def write(self):
        with open(self.cfgFile,'w') as outfile:
            json.dump(self.cfg,outfile,indent=4)
        

    # Movement

    def moveAbs(self,x=0,y=0):
        self.moveAbsTicks(x*self.cfg['Conversion']['x'],
                          y*self.cfg['Conversion']['y'])

    def move(self,x,y):
        self.moveTicks(x*self.cfg['Conversion']['x'],
                       y*self.cfg['Conversion']['y'])


    def moveAbsTicks(self,x=400000,y=200000):
        self.moveTicks(x,y,False)
    
    def moveTicks(self,x=0,y=0,rel=True):
        self.checkLimits(x,y,rel)

        if rel:
            self.__ser.write(str('U'+chr(7)+'v\r'))
        else:
            self.__ser.write(str('U'+chr(7)+'e\r'))

        self.__ser.write(str('U'+chr(0)+str(x)+'\r'))
        self.__ser.write(str('U'+chr(1)+str(y)+'\r'))
        status = self.execute()
        self.updatePos()
        
        return status

    def checkLimits(self,x=0,y=0,rel=True):
        if rel:
            if self.cfg['Current']['x']+x not in \
                    range(self.cfg["SoftLimits"]["xmin"],
                          self.cfg["SoftLimits"]["xmax"]):
                    sys.exit("X movement out of range")

            if self.cfg['Current']['y']+y not in \
                    range(self.cfg["SoftLimits"]["ymin"],
                          self.cfg["SoftLimits"]["ymax"]):
                    sys.exit("Y movement out of range")
        else:
            if x not in range(self.cfg["SoftLimits"]["xmin"],
                              self.cfg["SoftLimits"]["xmax"]):
                sys.exit("X movement out of range")

            if y not in range(self.cfg["SoftLimits"]["ymin"],
                              self.cfg["SoftLimits"]["ymax"]):
                sys.exit("Y movement out of range")

        return True

    def center(self):
        self.moveAbsTicks((self.cfg['SoftLimits']['xmax'] \
                               -self.cfg['SoftLimits']['xmin'])/2,
                          (self.cfg['SoftLimits']['ymax'] \
                               -self.cfg['SoftLimits']['ymin'])/2
                          )

    def home(self):
        self.moveAbsTicks(self.cfg['Home']['x'],self.cfg['Home']['y'])

    def updatePos(self):
        self.cfg["Current"]["x"] = int(self.read(67))
        self.cfg["Current"]["y"] = int(self.read(68))
        self.write()

    def getPos(self):
        self.updatePos()
        return self.cfg["Current"]["x"], self.cfg["Current"]["y"]

    def printPos(self):
        self.updatePos()
        print("x: %s\ny: %s\n" %(self.getPos()))

    def setHome(self,x=0,y=0):
        self.cfg['Home']['x'] = x
        self.cfg['Home']['y'] = y
        self.write()

    # Read
    def readMCL(self):
        try:
            while self.__ser.inWaiting()==0:
                pass
            time.sleep(0.1)
            return(self.__ser.read(self.__ser.inWaiting()))
        except KeyboardInterrupt:
            raise
        
    def read(self,register):
        self.__ser.write(str('U'+chr(register)+'\r'))
        return(self.readMCL())

    def status(self):
        print(self.read(70))

        
    # Configuration
    def setSpeed(self,speed=50):
        self.__ser.write(str('U'+chr(9)+str(speed)+'\r'))
        return(self.read(73))

    def joystick(self,on=True):
        if on:
            self.__ser.write(str('U'+chr(7)+'j\r'))
            self.execute()
        else:
            self.__ser.write(str('j\r'))
            self.updatePos()

    def setPos(self,x,y):
        self.__ser.write(str('U'+chr(3)+str(x)+'\r'))
        self.__ser.write(str('U'+chr(4)+str(y)+'\r'))
        self.updatePos()

if __name__ == "__main__":
    
    motor = MCL2()
    #motor.fullCalibration()
    motor.center()
#try:
    #    while True:
    #        motor.setSpeed(random.randint(10,90))
    #        motor.moveAbs(random.randint(0,700000),random.randint(0,400000))
    #except KeyboardInterrupt:
    #    motor.abort()
    #    motor.setSpeed(50)
    #    motor.home()

    #if len(sys.argv) == 3:
    #    motor.move(sys.argv[1],sys.argv[2])
    #if len(sys.argv) == 2:
    #    motor.move(sys.argv[1])
    
    #motor.moveAbs(380356, 203625)
    

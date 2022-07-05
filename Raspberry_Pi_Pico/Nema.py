from machine import Pin
from time import sleep

class Nema:
    
    def __init__(self, st, dr, ust = 16):
        self.step_pin = Pin(st, Pin.OUT)
        self.dir_pin = Pin(dr, Pin.OUT)
        self.ust = ust
        
        self.steps = 0
        self.direction = 0
        
        self.edge = 0
        self.dt = 0.01
        
    def setDt(self, newDt):
        self.dt = newDt
    
    def hold(self):
        self.step_pin.value(self.edge)
        self.dir_pin.value(self.direction)
    
    def go(self, angle, path = 0):
        cur_ang = self.steps/2 * 1.8  / self.ust
        dangle = angle - cur_ang
        steps = int( abs(dangle)*self.ust / 1.8 )
        self.direction = path
        if path == 0:
            if dangle > 0:
                self.dir_pin.value(self.direction)
                for i in range(0, 2*steps):
                    self.step()
                    sleep(self.dt)
                
            elif dangle < 0:
                self.dir_pin.value(self.direction)
                for i in range(0, 2*(200*self.ust - steps)):
                    self.step()
                    sleep(self.dt)
                    
        elif path == 1:
            if dangle < 0:
                self.dir_pin.value(self.direction)
                for i in range(0, 2*steps):
                    self.step()
                    sleep(self.dt)
                
            elif dangle > 0:
                self.dir_pin.value(self.direction)
                for i in range(0, 2*(200*self.ust - steps)):
                    self.step()
                    sleep(self.dt)
            
        
    def step(self):
        if self.edge == 0:
            self.step_pin.value(1)
            self.edge = 1
        else:
            self.step_pin.value(0)
            self.edge = 0
        
        if self.direction == 0:
            self.steps = self.steps+1
        else:
            self.steps = self.steps-1
        
        
            
    def getSteps(self):
        return self.steps
    
    def getAng(self):
         return self.steps / 2 * 1.8  / self.ust
        


#m1 = Nema(3,2)
#m1.go(360)
#m1.go(0,1)
#m2 = Nema(5,4, ust = 32)
#m2.setDt(0.001)
#m2.go(90)


        

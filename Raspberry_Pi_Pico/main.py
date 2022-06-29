from machine import I2C, Pin, UART, ADC, PWM
from time import sleep
from pico_i2c_lcd import I2cLcd
from rotary import Rotary
from nema import Nema

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

I2C_ADDR = i2c.scan()[0] #0x27
lcd = I2cLcd(i2c, I2C_ADDR, 2, 16)

pres = ADC(26)

light0 = ADC(27)
light1 = ADC(28)

servo = PWM(Pin(9))
servo.freq(50)
MIN = 1000000
MAX = 2000000

uart0 = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))

m1 = Nema(3,2)
m2 = Nema(5,4)

lcd.clear()
lcd.blink_cursor_off()

rotary = Rotary(7,6,8) #dt=GPIO7 clk=GPIO6 sw=GPIO8

laser = Pin(10, Pin.OUT)
leds = Pin(11, Pin.OUT)

val = 0
sw = False
update = True
lcd_mes=''
sel_res=False
sel_ang=False
sel_cal=False
sel_wait=False

ip = ''
angle=0

laser.off()
leds.on()
sleep(10)

start = False

def rotary_changed(change):
    global val
    global sw
    global update
    
    if change == Rotary.ROT_CW:
        val = val + 1
    elif change == Rotary.ROT_CCW:
        val = val - 1
    elif change == Rotary.SW_PRESS:
        sw = True
    elif change == Rotary.SW_RELEASE:
        sw = False
    update = True
 
rotary.add_handler(rotary_changed)

def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def servoP(angle):
    ns = _map(angle, 0, 90, MIN, MAX)
    servo.duty_ns(ns)

def lcd_update():
    global val
    global sw
    global update
    global ip
    global start
    
    if update:
        lcd.clear()
        
        if sw:
            start = True
        
        if not lcd_mes == '':
            lcd.putstr(lcd_mes)
            
        else:   
            if ip == '':
                lcd.putstr('Not connected!')
                start = False
            else:
                lcd.putstr('IP:{}'.format(ip))
                if not start:
                    lcd.putstr('Press to start!')
                else:
                    lcd.putstr('Progress: {}%'.format(angle/3.6))
                
        update = False

def selectResolution():
    global sel_res
    global sw
    global val
    global lcd_mes
    global start

    if sel_res:
        lcd_mes="Select the resolution:\n {}".format(val)
        start = False
    
    if sw:
        sel_res = False
        uart0.write(str(val)+'\n')
        val=0
        lcd_mes = ''
        sw = False
        

def selectAngularRes():
    global sel_ang
    global sw
    global val
    global lcd_mes
    global start

    if sel_ang:
        lcd_mes="Number of steps:\n {}".format(val)
        start = False
    
    if sw:
        sel_ang = False
        uart0.write(str(val)+'\n')
        val=0
        lcd_mes = ''
        sw = False

def selectCalibration():
    global sel_cal
    global sw
    global lcd_mes
    global start

    if sel_cal:
        lcd_mes="Calibration needed\n"
        start = False
    
    if sw:
        sel_cal = False
        uart0.write('start\n')
        lcd_mes = ''
        sw = False
        start=False

def selectWait():
    global sel_wait
    global start
    global lcd_mes
    global sw

    if sel_wait:
        lcd_mes="Too much light\n to begin scanning!"
        start=False

    if sw:
        sel_wait = False
        start=False
        lcd_mes = ''
        uart0.write('start\n')




while True:
    
    ret = uart0.read()         # read up to 5 bytes
    if ret:
        try:
            ret = ret.decode("utf-8", "ignore")
            
        except:
            print('error')
            
        if ret[-1] == '\n':
            uart0.write('ok\n')
            
        comm = ret[0]
        param = ret[1:]

        mes_ok = True
        
        if comm == 'r':
            dec = param.split('_')
            angle = float(dec[1])
            m1.go(angle, int(dec[0]))
        
        elif comm == 'z':
            dec = param.split('_')
            angle = float(dec[1])
            m2.go(angle, int(dec[0]))
            
        elif comm == 'k':
            servoP(float(param))
            
        elif comm == 'i':
            ip = param
            update = True
            
        elif comm == 'l':
            laser.value(int(param))
        
        elif comm == 'e':
            leds.value(int(param))
        
        elif comm == 'x':
            state = int(param)
            if state == 1:
                lcd.backlight_on()
            elif state == 0:
                lcd.backlight_off()
        
        elif comm == 's':
            sleep(0.1)
            if start:
                uart0.write('start\n')
            else:
                uart0.write('wait\n')
            
        elif comm == 'w':
            reading = pres.read_u16()
            sleep(0.2)
            uart0.write(str(reading)+'\n')
            print(reading)
        
        elif comm == 'p':
            reading0 = light0.read_u16()
            reading1 = light0.read_u16()
            sleep(0.2)
            uart0.write(str(reading0)+ '_' + str(reading1) + '\n')
            print(reading0)
            
        elif comm == 'o':
            sel_res=True
            update=True
            val = 0
        
        elif comm == 'y':
            sel_ang=True
            update=True
            val = 200
        
        elif comm == 'h':
            sel_cal=True
            update=True
        
        elif comm == 'j':
            sel_wait=True
            update=True

        elif comm == 'c':
            pass
        
        else:
            mes_ok = False
                
    selectResolution()
    selectAngularRes()
    selectCalibration()
    selectWait()
    lcd_update()
    sleep(0.01)

 
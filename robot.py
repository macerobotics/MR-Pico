# Mace Robotics
# Robot MR-Pico
# Version : 0.1
# Date : 13/01/23

from machine import Pin, Timer, PWM
import encoder
import vl6180x
import machine

# I2C
i2c = machine.I2C(0, scl=machine.Pin(13), sda=machine.Pin(12), freq=400000)

# prox sensors
xshut1 = Pin(9, Pin.OUT)
xshut2 = Pin(8, Pin.OUT) 
xshut3 = Pin(4, Pin.OUT)
xshut4 = Pin(3, Pin.OUT)
xshut5 = Pin(2, Pin.OUT)







# init led RGB
rgb_red = Pin(18, Pin.OUT)   
rgb_blue = Pin(17, Pin.OUT)      
rgb_green = Pin(16, Pin.OUT)

# motors
pwm1 = PWM(Pin(11))
pwm2 = PWM(Pin(15))  
dir1 = Pin(10, Pin.OUT)
dir2 = Pin(14, Pin.OUT)

# encodeurs
encoder1 = encoder.Encoder(0,1)
encoder2 = encoder.Encoder(6,7)

# buzzer
buzzer = PWM(Pin(19))

#servomoteur
servo1 = PWM(Pin(26))
servo1.freq(50)
  
  
#####################################################  
#####################################################
#####################################################
def proxRead(sensor):
  if (sensor == 1):
    xshut1.value(1)
    xshut2.value(0)
    xshut3.value(0)
    xshut4.value(0)
    xshut5.value(0)
  elif (sensor == 2):
    xshut1.value(0)
    xshut2.value(1)
    xshut3.value(0)
    xshut4.value(0)
    xshut5.value(0)
  elif (sensor == 3):
    xshut1.value(0)
    xshut2.value(0)
    xshut3.value(1)
    xshut4.value(0)
    xshut5.value(0)
  elif (sensor == 4):
    xshut1.value(0)
    xshut2.value(0)
    xshut3.value(0)
    xshut4.value(1)
    xshut5.value(0)    
  elif (sensor == 5):
    xshut1.value(0)
    xshut2.value(0)
    xshut3.value(0)
    xshut4.value(0)
    xshut5.value(1)
    
  try:
    proxSensor1 = vl6180x.Sensor(i2c, 0x29)
    return (proxSensor1.range())
  except:
    print("Error sensor!")
    return (-1)    


def ledRgb(red, green, blue):
  rgb_red.value(red)
  rgb_blue.value(blue)
  rgb_green.value(green)
  
def buzzer(frequence, duty):
  buzzer.freq(frequence)
  buzzer.duty_u16(int(duty))
  
def init_motor():
  pwm1.freq(10000)
  pwm2.freq(10000)
  pwm1.duty_u16(0)
  pwm2.duty_u16(0)
  
def forward(speed):
  dir1.value(1)
  dir2.value(1)
  speed = 255.35*speed + 40000
  pwm1.duty_u16(int(speed))
  pwm2.duty_u16(int(speed))
  print(speed)
      
def back(speed):
  dir1.value(0)
  dir2.value(0)
  speed = 255.35*speed + 40000
  pwm1.duty_u16(int(speed))
  pwm2.duty_u16(int(speed))
  print(speed)
  
def stop():
  pwm1.duty_u16(0)
  pwm2.duty_u16(0)
      
def brake():
  pwm1.duty_u16(65025)
  pwm2.duty_u16(65025)   
  
# End of file
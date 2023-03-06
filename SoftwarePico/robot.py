# Author : Mace Robotics (www.macerobotics.com)
# Robot MR-Pico
# Version : 0.25
# Date : 06/02/23
# MIT Licence


from machine import Pin, Timer, PWM
import encoder
import vl6180x
import machine
import time
import math 
from servo import Servo


# CONSTANTES
PERIMETER_ROBOT_MM = 273 # Robot perimeter (mm)
ENCODEURS_RESOLUTION = 1024 # tick pour 1 tour de roue.
PERIMETER_WHEEL = 100 # wheels perimeter (mm)
#GAIN_P = 0.7
GAIN_P = 1
GAIN_PO = 1 #pid orientation
# FIN CONSTANTES


# 1200 ticks = 90°
# 900 ticks = 100 mm

# Variable globales
deltaDistance = 0
old_stepDistance = 0
stepDistance = 0
stepOrientation = 0
XposRobot = 0
YposRobot = 0
controleEnable = False
command_distance = 0
command_orientation = 0
wheelRightCommand = 0
wheelLeftCommand = 0
max_speed_control = 0
error_distance = 9999999
error_orientation = 9999999
error_asv = 9999999
b_endControle = False
TypeControleAsv = 0 # 1 = Distance, 2 = Orientation, 0 = nothing
# Fin variables globales











# encoder init
codeurRight = encoder.Encoder(1,0)
codeurLeft = encoder.Encoder(6,7)

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
rgb_blue = Pin(20, Pin.OUT)      
rgb_green = Pin(21, Pin.OUT)

# motors
pwm1 = PWM(Pin(11))
pwm2 = PWM(Pin(15))  
dir1 = Pin(10, Pin.OUT)
dir2 = Pin(14, Pin.OUT)
pwm1.freq(100)
pwm2.freq(100)
dir1.value(0)
dir2.value(0)
pwm1.duty_u16(0)
pwm2.duty_u16(0)


# battery measure
batt = machine.ADC(28) # 12 bits

# buzzer
buzz = PWM(Pin(19, mode=Pin.OUT))

# Servomoteurs
s1 = Servo(22)
s2 = Servo(5)

ledPico = Pin(25, Pin.OUT)


time.sleep(3)
  
#####################################################  
#####################################################
#####################################################

# read the battery tension
def battery():
  value = batt.read_u16()
  tension = (value * 3.3)/65535
  return(tension*2)

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

# read the right encoder
def encoderRight():
  return(codeurRight.read())

# read the left encoder
def encoderLeft():
  return(codeurLeft.read())

# reset encoder counter
def encoderReset():
  codeurLeft.reset()
  codeurRight.reset()
  
def ledRgb(red, green, blue):
  rgb_red.value(red)
  rgb_blue.value(blue)
  rgb_green.value(green)
  
# buzzer
def buzzer(frequence, duty):
  buzz.freq(frequence)
  buzz.duty_u16(int(duty))
  
def buzzerStop():
  buzz.duty_u16(int(0))
  
# direction (0 ou 1), speped (0 à 100)
def motorRight(direction, speed):
  dir1.value(direction)
  pwm1.duty_u16(int((speed/100)*65_535))

# direction (0 ou 1), speped (0 à 100)
def motorLeft(direction, speed):
  dir2.value(direction)
  pwm2.duty_u16(int((speed/100)*65_535))
  
def forward(speed):
  dir1.value(1)
  dir2.value(1)
  pwm1.duty_u16(int((speed/100)*65_535))
  pwm2.duty_u16(int((speed/100)*65_535))
      
def back(speed):
  dir1.value(0)
  dir2.value(0)
  pwm1.duty_u16(int((speed/100)*65_535))
  pwm2.duty_u16(int((speed/100)*65_535))

def turnRight(speed):
  dir1.value(0)
  dir2.value(1)
  pwm1.duty_u16(int((speed/100)*65_535))
  pwm2.duty_u16(int((speed/100)*65_535))

def turnLeft(speed):
  dir1.value(1)
  dir2.value(0)
  pwm1.duty_u16(int((speed/100)*65_535))
  pwm2.duty_u16(int((speed/100)*65_535))
  
def stop():
  pwm1.duty_u16(0)
  pwm2.duty_u16(0)
      
def brake():
  pwm1.duty_u16(65025)
  pwm2.duty_u16(65025)
  
def positionX():
  global XposRobot
  return((XposRobot*100)/1024)

def positionY():
  global YposRobot
  return(YposRobot)
  
def orientation():
  stepOrientation = encoderRight() - encoderLeft()
  print("stepOrientation:", stepOrientation)
  angle = (stepOrientation * 90)/(696)
  return(angle)

# forward with control
def forwardmm(distance, speed):
  global controleEnable, command_distance, max_speed_control, command_orientation, b_endControle, error_distance
  global TypeControleAsv

  if (distance > 50):
      encoderReset()
      TypeControleAsv = 1
      controleEnable = True
      b_endControle = False
      error_asv = 9999999
      command_distance = (distance*900)/100 #conversion en tick
      command_orientation = 0
      max_speed_control = speed
      
      while (b_endControle != True):
        time.sleep(1)

def backmm(distance, speed):
  global controleEnable, command_distance, max_speed_control, command_orientation, b_endControle, error_distance
  global TypeControleAsv

  encoderReset()
  TypeControleAsv = 1
  controleEnable = True
  b_endControle = False
  error_asv = 9999999
  command_distance = (-distance*1024)/100 #conversion en tick
  command_orientation = 0
  max_speed_control = speed
  
  while (b_endControle != True):
    time.sleep(1)
  
def turnAngle(angle, speed):
  global controleEnable, command_distance, max_speed_control, command_orientation, b_endControle, error_distance
  global TypeControleAsv
  
  encoderReset()
  TypeControleAsv = 2
  controleEnable = True
  b_endControle = False
  error_asv = 99999
  command_distance = 0 
  command_orientation = (angle*1200)/90
  max_speed_control = speed

  while (b_endControle != True):
    time.sleep(1)


def servo_Map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
 
#servo = 0 ou 1, Angle de 0 à 180
def servo_Angle(servo, angle):
    if angle < 0:
        angle = 0
    if angle > 180:
        angle = 180
    if servo == 1:
      s1.goto(round(servo_Map(angle,0,180,0,1024))) # Convert range value to angle value
    else:
      s2.goto(round(servo_Map(angle,0,180,0,1024))) # Convert range value to angle value        


################################################################
################################################################
################################################################
################################################################
################################################################
################################################################
# INTERRUPTION
timer=Timer()


timer2=Timer()

timer3=Timer() 

def controlRobot(timer2):
  global command_distance, command_orientation, stepDistance, controleEnable, wheelRightCommand, wheelLeftCommand
  global stepOrientation, error_distance, b_endControle, error_orientation
  global error_asv,TypeControleAsv 
  
  #print("controlRobot")
  #print("error_distance", error_distance)
  print("error_orientation", error_orientation)
  #print("stepDistance", stepDistance)
  
  
  
  #if ((abs(error_distance) > 10) or (abs(error_orientation) > 10)) :
  if (abs(error_asv) > 10) :
    if controleEnable == True:
            
          # erreur distance
          error_distance = command_distance - stepDistance
          
          # erreur orientation
          error_orientation = command_orientation - stepOrientation    
          
          if (TypeControleAsv == 1):
            error_asv = error_distance
          elif (TypeControleAsv == 2):
            error_asv = error_orientation
            
            
          command = error_distance*GAIN_P
          
          command_orientaton = error_orientation*GAIN_PO
          
          
          # wheels command
          wheelRightCommand = command + command_orientaton
          wheelLeftCommand = command - command_orientaton
    
    
    
    
          # saturation
          if(wheelRightCommand > 50):
              wheelRightCommand = 50
              
          # saturation            
          if(wheelLeftCommand > 50):
              wheelLeftCommand = 50
              
          # saturation
          if(wheelRightCommand < -50):
              wheelRightCommand = -50
              
          # saturation            
          if(wheelLeftCommand < -50):
              wheelLeftCommand = -50
              
          
          if(wheelRightCommand > 0):
              motorRight(1,wheelRightCommand)
          else:
              motorRight(0,abs(wheelRightCommand))      
    
          if(wheelLeftCommand > 0):
              motorLeft(1,wheelLeftCommand)
          else:
              motorLeft(0,abs(wheelLeftCommand))
              
              
          
    else:
      #print("Fin CONTROLE2")
      stop()
      b_endControle = True



# Position du robot (x, y et orientation)
def positionControl(timer):
  global old_stepDistance, XposRobot, YposRobot, stepOrientation, stepDistance, stepOrientation
  

  
  stepDistance = (encoderRight() + encoderLeft())/2
  stepOrientation = encoderRight() - encoderLeft()
  
  deltaDistance = stepDistance - old_stepDistance
  old_stepDistance = stepDistance
  stepOrientation = encoderRight() - encoderLeft()
  radOrientation = stepOrientation*(0.0011)
  
  # delat X and Y position robot calculator
  deltaX = deltaDistance*math.cos(radOrientation)
  deltaY = deltaDistance*math.sin(radOrientation)
  
  # X and Y position of the robot
  XposRobot = XposRobot + deltaX
  YposRobot = YposRobot + deltaY
  
  Orientation = math.degrees(radOrientation)

  #print("stepOrientation :", stepOrientation)
  #print("Orientation :", Orientation)

# Gestion de la led de la carte Raspberry Pico
def ledManage(timer):  
  ledPico.toggle()  
  
  
####################################################################################
####################################################################################  
  
# timer.init(freq=10, mode=Timer.PERIODIC, callback=tick)
timer.init(freq=100, callback=positionControl) # forme minimale

# timer.init(freq=10, mode=Timer.PERIODIC, callback=tick)
timer2.init(freq=100, callback=controlRobot) # forme minimale

timer3.init(freq=0.5, callback=ledManage) # gestion led pico
################################################################
# End of file
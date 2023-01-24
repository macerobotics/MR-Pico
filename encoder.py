#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Date : 19/09/2022
# Version : 0.1
# Auteur : Mace Robotics
# 
# 
#

from machine import Pin



class Encoder:
    
  def __init__(self, pin1, pin2):
      self.pin1 = pin1
      self.pin2 = pin2
      self.count = 0
      pir1 = Pin(pin1, Pin.IN, pull=Pin.PULL_UP)
      pir1.irq(trigger=Pin.IRQ_RISING, handler=self.handle_interrupt1)  
      self.pir2 = Pin(pin2, Pin.IN, pull=Pin.PULL_UP)  
      
  def read(self):
    return(self.count)   

  def handle_interrupt1(self, pin):
    global flag, count1
    
    if (self.pir2.value() == 0):
      self.count = self.count + 1
    else:
      self.count = self.count - 1
    
    
# end of file
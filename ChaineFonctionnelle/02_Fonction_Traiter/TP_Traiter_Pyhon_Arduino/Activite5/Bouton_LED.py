# -*- coding: utf-8 -*-
"""
@author: Xavier Pessoles
"""

# Déclarations initiales
COM_ARDUINO = 19
BOUT_A = 12
BOUT_B = 11
BOUT_C = 10
BOUT_D = 9
LED_V = 3

from py2duino import *
import time

# Déclaration de la carte
MaCarte=Arduino(COM_ARDUINO)
# Attente de la communication
time.sleep(2) 
print("Fin d'attente")

# Déclaration des variable
a = DigitalInput(MaCarte,BOUT_A)
b = DigitalInput(MaCarte,BOUT_B)
c = DigitalInput(MaCarte,BOUT_C)
d = DigitalInput(MaCarte,BOUT_D)

led_V = DigitalOutput(MaCarte,LED_V)
led_V.low()
start = time.time()
duree = 10
ba,bb,bc,bd = False,False,False,False
# Exécution du programme pendant duree secondes
while time.time()<start+duree :
    if a.read() == 1 :
        ba = False
    else : 
        ba = True
    
    if b.read() == 1 :
        bb = False
    else : 
        bb = True
        
    if c.read() == 1 :
        bc = False
    else : 
        bc = True
        
    if d.read() == 1 :
        bd = False
    else : 
        bd = True
    if (ba and bb) or (ba and bc) or (bb and bc and bd)==True :
        print("OK")
        led_V.high()
    else :
        led_V.low()
    time.sleep(.5)
    #led_V.high() 
    #print(type(a.read()))
    #time.sleep(.5)
   # print(a.read())
    #led_V.low() 
led_V.low() 
    
# On libère la carte Arduino
MaCarte.close()


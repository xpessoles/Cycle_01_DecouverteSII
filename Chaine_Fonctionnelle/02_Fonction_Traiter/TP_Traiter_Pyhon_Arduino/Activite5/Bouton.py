# -*- coding: utf-8 -*-
"""
@author: Xavier Pessoles
"""

# Déclarations initiales
COM_ARDUINO = 4
BOUT_A = 2
LED_V = 7

from py2duino import *
import time

# Déclaration de la carte
MaCarte=Arduino(COM_ARDUINO)
# Attente de la communication
time.sleep(2) 
print("Fin d'attente")

# Déclaration des variable
bout_A = DigitalInput(MaCarte,BOUT_A)
led_V = DigitalOutput(MaCarte,LED_V)

start = time.time()
duree = 10
# Exécution du programme pendant duree secondes
while time.time()<start+duree :
    time.sleep(.5)
    led_V.high() 
    print(bout_A.read())
    time.sleep(.5)
    print(bout_A.read())
    led_V.low() 
    
    
# On libère la carte Arduino
MaCarte.close()


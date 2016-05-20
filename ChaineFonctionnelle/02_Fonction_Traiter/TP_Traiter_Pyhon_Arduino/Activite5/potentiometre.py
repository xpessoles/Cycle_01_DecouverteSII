# -*- coding: utf-8 -*-
"""
@author: Xavier Pessoles
"""

# Déclarations initiales
COM_ARDUINO = 11
LED_R = 2

from py2duino import *
import time

# Déclaration de la carte
MaCarte=Arduino(COM_ARDUINO)
# Attente de la communication
time.sleep(2) 
print("Fin d'attente")

# Déclaration des variable
ledr = DigitalOutput(MaCarte,LED_R)

start = time.time()
duree = 10
# Exécution du programme pendant duree secondes
while time.time()<start+duree :
    ledr.high()
    time.sleep(.3)
    ledr.low()
    time.sleep(.3)
# On libère la carte Arduino
MaCarte.close()


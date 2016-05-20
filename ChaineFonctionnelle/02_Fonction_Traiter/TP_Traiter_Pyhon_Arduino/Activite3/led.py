# -*- coding: utf-8 -*-
"""
@author: Xavier Pessoles
"""

# Déclarations initiales
COM_ARDUINO = 11
PIN_LED = 13

from py2duino import *
import time

# Déclaration de la carte
MaCarte=Arduino(COM_ARDUINO)
# Attente de la communication
time.sleep(2) 
print("Fin d'attente")

# Déclaration des variables
led = DigitalOutput(MaCarte,PIN_LED)

start = time.time()
duree = 5
# Exécution du programme pendant duree secondes
while time.time()<start+duree :
    # On allume la led
    led.high()
    time.sleep(1)
    # On éteint la led
    led.low()
    time.sleep(1)

# On libère la carte Arduino
MaCarte.close()


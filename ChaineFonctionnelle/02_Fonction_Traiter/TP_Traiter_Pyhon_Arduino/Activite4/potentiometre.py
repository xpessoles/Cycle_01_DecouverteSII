# -*- coding: utf-8 -*-
"""
@author: Xavier Pessoles
"""

# Déclarations initiales
COM_ARDUINO = 11
POT = 5

from py2duino import *
import time

# Déclaration de la carte
MaCarte=Arduino(COM_ARDUINO)
# Attente de la communication
time.sleep(2) 
print("Fin d'attente")

# Déclaration des variables
potentiometre = AnalogInput(MaCarte,POT)

start = time.time()
duree = 10
# Exécution du programme pendant duree secondes
while time.time()<start+duree :
    # On lit 
    print(potentiometre.read())
    
    time.sleep(.2)

# On libère la carte Arduino
MaCarte.close()


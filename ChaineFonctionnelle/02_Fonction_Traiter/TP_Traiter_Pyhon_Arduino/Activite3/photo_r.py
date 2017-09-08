# -*- coding: utf-8 -*-
"""
@author: Xavier Pessoles
"""

# Déclarations initiales
COM_ARDUINO = 3
PIN_res = 1

from py2duino import *
import time
import matplotlib.pyplot as plt


# Déclaration de la carte
MaCarte=Arduino(COM_ARDUINO)
# Attente de la communication
time.sleep(2) 
print("Fin d'attente")

# Déclaration des variables
res = AnalogInput( MaCarte,PIN_res)

start = time.time()
duree = 10
# Exécution du programme pendant duree secondes
tt = []
lum=[]
while time.time()<start+duree :
    # On allume la led
    lum.append(res.read())
    time.sleep(.005)
    tt.append(time.time())
    

# On libère la carte Arduino
MaCarte.close()
plt.plot(tt,lum)


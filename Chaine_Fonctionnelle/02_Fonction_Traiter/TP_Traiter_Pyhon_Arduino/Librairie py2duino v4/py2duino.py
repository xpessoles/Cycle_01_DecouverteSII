# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
# Name:        py2duino
# Purpose:     Programming arduino from python commands
#
# Author:      David Violeau, Alain Caignot
#
# Created:     01/01/2014
# Modified :   01/03/2016 D. Violeau

# Copyright:   (c) Demosciences UPSTI
# Licence:     GPL
#-------------------------------------------------------------------------------
import serial
import sys
import time
import struct
import traceback

class Servo():
    """
     Classe Servo pour piloter un ou deux servomoteurs
    
     Les servomoteurs doivent etre definis sur les broches 9 et 10 obligatoirement.
     Il suffit ensuite de definir les numeros des servomoteurs 1 ou 2
     
     servo=Servo(monarduino,1) # avec monarduino la carte definie
    
     Pour demander au servomoteur de tourner de x degre (de 0 à 180), taper servo.write(x)
    """
    def __init__(self,parent,number):
        self.number=number
        self.parent=parent
        if number!=1 and number!=2:
            print("Les servomoteurs doivent etre branches sur les pin digitaux 9 ou 10, taper Servo(monarduino,1 ou 2)")
            self.parent.close()
        else:
            self.attach()
            print("attach ok")
        
    def attach(self):
        mess="Sa"+str(self.number)
        self.parent.ser.write(bytes(mess,"ascii"))
        
    def dettach(self):
        mess="Sd"+str(self.number)
        self.parent.ser.write(bytes(mess,"ascii"))
        print("detach ok")

    def write(self,value):
        if value<0 :
            value=0
        elif value>180:
            value=180
        
        mess="Sw"+str(self.number)+chr(value)
        self.parent.ser.write(bytes(mess,"ISO-8859-1"))

class PulseIn():
    """ 
    Classe PulseIn pour connaitre le temps écoulé entre l'émission et la réception d'un signal de durée donnée
    
    Vous devez spécifier la carte arduino et le pin digital du trigger (émetteur) puis du pin digital récepteur (echo)
    
    pulse=DigitalOutput(macarte,6,5) (pin trigger 6, pin echo 5)
    
    Pour lancer une lecture, taper puilse.read(duree) où duree est la durée en milliseconde entre le passage à l'état haut puis bas du trigger. La grandeur renvoyée est en microsecondes.
    
    Si aucune duree n'est renseignée, une valeur de 20 ms est adoptée par défaut. La durée maximale est de 255 ms.
    """

    def __init__(self,parent,pin_trigger,pin_echo):
        self.trigger=pin_trigger
        self.echo=pin_echo
        self.parent=parent
        self.init()
        self.duree=20
        
    def init(self):
        self.parent.pinMode(self.trigger,"OUTPUT")
        self.parent.pinMode(self.echo,"INPUT")


    def read(self,duree=20):
        if duree>255:
            duree=20
        if duree <=0:
            duree=20
        self.duree=duree
        val=self.parent.pulseIn(self.duree,self.trigger,self.echo)
        return val


class DigitalOutput():
    """
    Classe DigitalOutput pour mettre à l'état haut ou bas une sortie digitale
    
    Vous devez spécifier la carte arduino et le pin digital de sortie souhaité
    led=DigitalOutput(macarte,9)
    
    Pour mettre à l'état haut taper : led.high(), pour l'état bas : led.low()
    """

    def __init__(self,parent,pin):
        self.pin=pin
        self.parent=parent
        self.init()
        
    def init(self):
        self.parent.pinMode(self.pin,"OUTPUT")


    def high(self):
        self.parent.digitalWrite(self.pin,1)

    def low(self):
        self.parent.digitalWrite(self.pin,0)

class DigitalInput():
    """
    Classe DigitalInput pour lire la donnée binaire d'un pin digital
    
    Vous devez spécifier la carte arduino et le pin digital d'entré souhaité
    push=DigitalInput(macarte,9)
    
    Pour lire la valeur, tapez : push.read(). La valeur obtenue est 0 ou 1 (état haut)
    La fonction push.upfront() (ou downfront) permet de renvoyer 1 ou 0 sur front montant ou descendant de l'entrée
    La fonction push.pulse() renvoie la durée (en microsecondes) écoulée entre deux passages au niveau haut de l'entrée
    """
    def __init__(self,parent,pin):
        self.pin=pin
        self.parent=parent
        self.previous_value=0
        self.value=0
        self.duree=0
        self.init()

    def init(self):
        self.parent.pinMode(self.pin,"INPUT")
        self.value=self.parent.digitalRead(self.pin)
        self.previous_value=self.value
        
    def read(self):
        return self.parent.digitalRead(self.pin)

    def upfront(self):
          self.value=self.parent.digitalRead(self.pin)
          val=0
          if  ((self.value!=self.previous_value) & (self.value==1)):
              val=1
          else :
              val=0
          self.previous_value=self.value
          return val

    def downfront(self):
          self.value=self.parent.digitalRead(self.pin)
          val=0
          if  ((self.value!=self.previous_value) & (self.value==0)):
              val=1
          else :
              val=0
          self.previous_value=self.value
          return val
    def pulse(self):
        print("pulse In ")
        self.duree=self.parent.pulseIn(self.pin)
        return self.duree

class AnalogInput():
    """
    Classe AnalogInput pour lire les données issues d'une voie analogique
    
    Vous devez spécifier la carte arduino et le pin analogique d'entrée souhaitée
    analog=AnalogInput(macarte,0)
    
    Pour lire la valeur analogique : analog.read(). La valeur renvoyée est comprise entre 0 et 1023
    """
    def __init__(self,parent,pin):
        self.pin=pin
        self.parent=parent
        self.value=0

    def read(self):
        self.value=self.parent.analogRead(self.pin)
        return self.value

class AnalogOutput():
    """
    Classe AnalogOutput pour envoyer une grandeur analogique variant de 0 à 5 V 
    ce qui correspond de 0 à 255 \n
    Vous devez spécifier la carte arduino et le pin Analogique (pwm ~) de sortie souhaité
    led=AnalogOutput(macarte,9)
    
    Utiliser la commande led.write(200)
    """
    def __init__(self,parent,pin):
        self.pin=pin
        self.parent=parent
        self.value=0

    def write(self,val):
        if val>255 :
            val=255
        elif val<0:
            val=0
        self.parent.analogWrite(self.pin,val)

class DCmotor():
    """
    Classe DCmotor pour le pilotage des moteurs a courant continu
    
    Les parametres de definition d'une instance de la classe sont :\n
    - la carte arduino selectionnee\n
    - le numero du moteur de 1 a 4\n
    - le pin du PWM1\n
    - le pin de sens ou de PWM2\n
    - le type de pilotage : 0 (type L293 avce 2 PWM) ou  1 (type L298 avec un PWM et une direction)\n
    Ex : monmoteur=DCmotor(arduino1,1,3,5,0) (moteur 1 pilotage de type L293 avec les PWM des pins 3 et 5\n
    
    Pour faire tourner le moteur a une vitesse donne, taper : monmoteur.write(x) avec x compris entre -255 et 255)
    """

    def __init__(self,parent,number,pin_pwm,pin_dir,mode):
        self.pin1=pin_pwm
        self.pin2=pin_dir
        self.mode=mode
        self.number=number
        self.parent=parent
        if mode!=0 and mode!=1:
            print("Choisir un mode egal a 0 pour un pilotage par 2 PWM ou 1 pour un pilotage par un PWM et un pin de sens")
        elif number!=1 and number!=2 and number!=3 and number!=4:
            print("Choisir un numero de moteur de 1 a 4")
        else :
            try:
                mess="C"+str(self.number)+chr(48+self.pin1)+chr(48+self.pin2)+str(self.mode)
                self.parent.ser.write(bytes(mess,"ISO-8859-1"))
                tic=time.time()
                toread=self.parent.ser.inWaiting()
                value=""
                while(toread < 2 and time.time()-tic < 1):  # attente retour Arduino
                    toread=self.parent.ser.inWaiting();
                value=self.parent.ser.read(toread);
                if value==b"OK":
                    print("Moteur "+str(self.number)+" correctement connecté")
                else:
                    print("Problème de déclaration et connection au moteur")
                    self.parent.close()
                    return
            except:
                print("Problème de déclaration et connection au moteur")
                self.parent.close()
            
    def write(self,value):
        value=int(value)
        if value<-255:
            value=-255
        elif value>255:
            value=255
        if value<0:
            direction=0
        else:
            direction=1
        mess="M"+str(self.number)+chr(48+direction)+chr(abs(round(value)))
        self.parent.ser.write(bytes(mess,"ISO-8859-1"))


class Stepper():
    """
    Classe Stepper pour le pilotage d'un moteur pas à pas accouplé à un driver nécessitant la donnée d'une direction et d'un signal pulsé
    
    Les parametres de definition d'une instance de la classe sont :\n
    - la carte arduino selectionnee\n
    - un numero de 1 à 4 pour activer de 1 à 4 moteurs pas à pas\n
    - le numéro du pin donnant le signal pulsé (STEP)\n
    - le numéro du pin donnant la direction (DIR)\n
    
    Ex : monstepper=Stepper(arduino1,1,10,11) (moteur 1 sur les pins STEP 10, DIR 11 
    
    Pour faire tourner le moteur d'un nombre de pas donné, taper : monmoteur.move(nb_steps, dt) 
    avec nb_steps le nombre de pas ou demi-pas (ou autre en fonction du mode) qui peut etre positif ou négatif (ce qui définit le sens de déplacement), dt le temps laissé entre chaque pas en millisecondes (de 0.05 à 15). Attention, le choix du pas de temps est fonction de la dynamique souhaitée et donc du réglage de l'intensité de consigne sur le driver du moteur.
    
    Il est peut etre nécessaire de définir des sorties digitales ENABLE, pour autoriser ou non le moteur pas à pas à bouger et des sorties digitales MS1, MS2, MS3 pour définir le mode de pas à pas (pas complet, demi-pas, ou autre...)
    """

    def __init__(self,parent,number,pin_pwm,pin_dir):
        self.pin_pwm=pin_pwm
        self.pin_dir=pin_dir
        self.parent=parent
        self.number=number
        self.nb_steps=0
        self.dt=0
        self.launched=0
        self.tic=0

        try:
            mess="Pa"+chr(48+self.number)+chr(48+self.pin_pwm)+chr(48+self.pin_dir)
            print(mess)
            self.parent.ser.write(bytes(mess,"ISO-8859-1"))
            tic=time.time()
            toread=self.parent.ser.inWaiting()
            value=""
            while(toread < 2 and time.time()-tic < 1):  # attente retour Arduino
                toread=self.parent.ser.inWaiting();
            value=self.parent.ser.read(toread);
            if value==b"OK":
                print("Moteur pas a pas correctement connecté")
            else:
                print("Problème de déclaration et connection au moteur pas a pas")
                self.parent.close()
                return
        except:
            print("Problème de déclaration et connection au moteur pas a pas")
            self.parent.close()
    

    def move(self,nb_steps,dt):
        self.dt=dt
        dt=dt/2
        self.nb_steps=nb_steps
        if dt>15:
            dt=15
        if dt<0.05:
            dt=0.05
        dt=int(dt*1000)

        value=int(nb_steps)
        if value >=0:
            direction=1
        else:
            direction=0
        
        value=abs(value)
        if value>=2**32:
            value=2**32-1
        
        messb=bytes("Pm"+chr(48+self.number)+chr(48+direction),"ISO-8859-1")+struct.pack('L',value)+struct.pack('i',dt)
        self.parent.ser.write(messb)
        tic=time.time()
        toread=self.parent.ser.inWaiting()
        value=""
        while(toread < 2 and time.time()-tic < 1):  # attente retour Arduino
            toread=self.parent.ser.inWaiting()
        value=self.parent.ser.read(toread)
        if value!=b'OK':
            self.stop()
        self.launched=1
        self.tic=time.time()

    def stop(self):
        mess="Ps"+chr(48+self.number)
        self.parent.ser.write(bytes(mess,"ISO-8859-1"))
        self.launched=0
        
    def isMoving(self):
        mess="Pi"+chr(48+self.number)
        self.parent.ser.write(bytes(mess,"ISO-8859-1"))
        tic=time.time()
        toread=self.parent.ser.inWaiting()
        value=""
        while(toread < 1 and time.time()-tic < 1):  # attente retour Arduino
            toread=self.parent.ser.inWaiting()
        value=self.parent.ser.read(toread)
        if value!=b'1' and value!=b'0':
            return 0
        else:
            return int(value)
            
    def isMoving2(self):
        #print(time.time()-self.tic)
        if time.time()-self.tic>abs(self.nb_steps*self.dt/1000)+0.01:
            self.launched=0
        return self.launched

        
class Encoder():
    """
    Classe Encoder pour la lecture d'interruption sur des encodeurs 2 voies
    
    Les parametres de definition d'une instance de la classe sont :\n
     - la carte arduino selectionnee\n
     - le pin de l'entrée digitale de la voie A (avec interruption)\n
     - le pin de l'entrée digitale de la voie B (avec ou sans interruption)\n
     - le type de lecture : 1 pour front montant de la voie A (et sens donné par la voie B), 2 pour fronts montants et descendants de la voie A seule, 4 pour fronts montants et descendants des deux voies
    
     codeur=Encoder(macarte,2,3,1)
     
     pour lire le nombre de tops, taper codeur.read(). La valeur obtenue peut etre positive ou négative (codée sur 4 octets)
     
     Pour remettre à 0 la valeur lue, codeur.reset()
     
    """
    def __init__(self,parent,pinA,pinB,mode):
        self.parent=parent
        self.pinA=pinA
        self.pinB=pinB
        self.mode=mode
        self.corresp=[-1,-1,0,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,5,4,3,2]
        try:
            if self.corresp[self.pinA]==-1 or self.corresp[self.pinB]==-1:
                print("Les pins utilisés pour l'encodeur doivent accepter les interruptions. Choisir les pins 2,3 (UNO, MEGA) ou 18 à 21 (MEGA)")
                self.parent.close()
        except:
            print("Les pins utilisés pour l'encodeur doivent accepter les interruptions. Choisir les pins 2,3 (UNO, MEGA) ou 18 à 21 (MEGA)")
            self.parent.close()
        if mode!=1 and mode!=2 and mode !=4:
            print(["Choisir un mode pour le codeur egal a : ",
                   "1 : front montant de la voie A, ",
                   "2 : front montant et descendants de la voie A,",
                   "4 : fronts montants et descendants des voies A et B"])
            self.parent.close()
        else :
            if mode==4:
                mess="Ea"+chr(self.corresp[self.pinA])+chr(self.corresp[self.pinB])+str(mode)
            else:
                mess="Ea"+chr(self.corresp[self.pinA])+chr(self.pinB)+str(mode)
            self.parent.ser.write(bytes(mess,"ISO-8859-1"))

    def reset(self):
        mess="Ez"+chr(self.corresp[self.pinA])
        self.parent.ser.write(bytes(mess,"ISO-8859-1"))          

    def read(self):
        mess="Ep"+chr(self.corresp[self.pinA])
        self.parent.ser.write(bytes(mess,"ISO-8859-1"))
        toread=self.parent.ser.inWaiting()
        tic=time.time()
        while(toread < 4 and time.time()-tic < 1):  # attente retour Arduino
            toread=self.parent.ser.inWaiting();
        value=self.parent.ser.read(toread);
        value=struct.unpack('l',value)
        return value[0]
                  
class Arduino():
    """
    Classe Arduino pour definition d'une carte arduino. Ne fonctionne qu'en python 3.X
    
    Il faut charger le programme toolbox_arduino_vX.ino dans la carte Arduino pour pouvoir utiliser cette classe.
    
    Les parametres de definition d'une instance de la classe sont :\n
     - le port de communication (numero sous Windows, chemin sous linux /dev/xxx )\n
    
    Le taux de transfert peut être passé en argument mais il faut modifier celui du fichier ino pour qu'il corresponde. Par défaut il est à 115200
    
     La connection à la carte est faite automatiquement à la déclaration\n
     Taper macarte=Arduino(8) pour se connecter automatiquement à la carte sur le port 8\n
     Taper macarte.close() pour fermer la connexion\n
     Il faudra alors toujours passer en argument l'objet macarte pour toute déclaration de pins arduino (DigitalOutput, DigitalInput, DCmotor, Stepper, Servo, AnalogInput, AnalogOutput, Encoder)
    """    
    def __init__(self,com,baudrate=115200):
        self.ser = serial.Serial()
        self.com=com #define com port
        self.baudrate=baudrate #define com debit
        self.digitalPins=[] #list of defined digital pins
        self.analogPins=[]  #list of defined analog pins
        self.interrupts=[]  #list of defined interrupt
        self.steppers=[] #list of defined pins for stepper
        self.connect()
    
    def ask(self):
        mess="B"
        self.ser.write(bytes(mess,"ISO-8859-1"))
        toread=self.ser.inWaiting();
        #tic=time.time()
        #while(toread < 2 and time.time()-tic < 2):  # attente retour Arduino
        #    toread=self.ser.inWaiting();
        value=0
        if toread!=0:
            value=self.ser.read(toread)
        print(value)
            
    def connect(self):
        self.ser.baudrate=self.baudrate   #definition du debit
        import os
        if os.name == "posix":
            self.ser.port=self.com
        else:
            if int(serial.VERSION.split('.')[0])>=3:
                self.ser.port="COM"+str(self.com)
            else:
                self.ser.port=self.com-1
        connect = 0
        try:
            self.ser.open();         #open port4
            time.sleep(2);           #wait for stabilisation
            self.ser.write(b"R3");    #send R3 to ask for arduino program
            tic = time.time();

            toread=self.ser.inWaiting();
            while(toread < 2 and time.time()-tic < 2):  # attente retour Arduino
                toread=self.ser.inWaiting();
            value=self.ser.read(toread)
            if value == b"v4":
                print("Connexion ok avec l'arduino")
                connect=1

        finally:
            if connect==0:
                print("Connexion impossible avec l'arduino. Verifier que le com est le bon et que le programme toolbox_arduino_v4.ino est bien chargé dans l'arduino")
                return 0
            else:
                return 1

    def close(self):
        self.ser.close()


    def pinMode(self,pin,type):
        mode='x'
        if type=='OUTPUT':
            mode='1'
        elif type=='INPUT':
            mode='0'
        else:
            print("Attention le type OUTPUT ou INPUT du pin n'est pas bien renseigne")
        if mode != 'x':
            mess="Da"+chr(pin+48)+mode
            #self.ser.write(bytes(mess,"ascii"))
            self.ser.write(bytes(mess,"ISO-8859-1"))
            self.digitalPins.append(pin)

    def digitalwrite(self,pin,value):
        self.digitalWrite(self,pin,value)

    def digitalWrite(self,pin,value):
        try:
            self.digitalPins.index(pin)
            data='x'
            if value=="HIGH":
                data='1'
            elif value=="LOW":
                data='0'
            elif value==1 or value==0:
                data=str(value)
            else:
                print("Valeur incorrecte pour un pin digital")
            if data !='x':
                mess="Dw"+chr(pin+48)+data
                #self.ser.write(bytes(mess,"ascii"))
                self.ser.write(bytes(mess,"ISO-8859-1"))

        except:
            print("Erreur de transmission ou bien le pin digital n'a pas ete bien assigne au prealable avec arduino.pinMode")
            self.close()
            traceback.print_exc(file=sys.stdout)


    def analogwrite(self,pin,value):
        self.analogWrite(self,pin,value)

    def analogWrite(self,pin,value):
        try:
            if abs(value)>255:
                code_sent="W"+chr(pin+48)+chr(255);
            else:
                code_sent="W"+chr(pin+48)+chr(abs(round(value)))
            self.ser.write(bytes(code_sent,"ISO-8859-1"))

        except:
            print("Erreur de transmission")
            self.close()
            #traceback.print_exc(file=sys.stdout)

    def digitalread(self,pin):
        return self.digitalRead(self,pin)

    def digitalRead(self,pin):
        try:
            self.digitalPins.index(pin)
            mess="Dr"+chr(pin+48)
            #self.ser.write(bytes(mess,"ascii"))
            self.ser.write(bytes(mess,"ISO-8859-1"))
            tic=time.time()
            toread=self.ser.inWaiting();
            while(toread < 1 and time.time()-tic < 1):  # attente retour Arduino
                toread=self.ser.inWaiting();
            value=self.ser.read(toread)
            return int(value)
        except:
            print("Erreur de transmission ou bien le pin digital n'a pas ete bien assigne au prealable avec arduino.pinMode")
            self.close()
            #traceback.print_exc(file=sys.stdout)

    def pulseIn(self,duree,trigger,echo):
        try:
            self.digitalPins.index(trigger)
            self.digitalPins.index(echo)
            mess="Dp"+chr(trigger+48)+chr(echo+48)+chr(abs(round(duree)))
            #self.ser.write(bytes(mess,"ascii"))
            self.ser.write(bytes(mess,"ISO-8859-1"))
            
            toread=self.ser.inWaiting()
            tic=time.time()
            while(toread < 4 and time.time()-tic < 1):  # attente retour Arduino
                toread=self.ser.inWaiting()
            value=self.ser.read(toread)
            value=struct.unpack('l',value)
            return value[0]
        except:
            print("Erreur de transmission")
            self.close()
            #traceback.print_exc(file=sys.stdout)

    def analogread(self,pin):
        return self.analogRead(self,pin)

    def analogRead(self,pin):
        try:
            mess="A"+chr(pin+48)
            #self.ser.write(bytes(mess,"ascii"))
            self.ser.write(bytes(mess,"ISO-8859-1"))
            toread=self.ser.inWaiting()
            tic=time.time()
            while(toread < 2 and time.time()-tic < 1):  # attente retour Arduino
                toread=self.ser.inWaiting();
            value=self.ser.read(toread);
            value=struct.unpack('h',value)
            return value[0]
        except:
            print("Erreur de transmission")
            self.close()
            #traceback.print_exc(file=sys.stdout)

    def Servo(self,pin):
        return Servo(self,pin)

        

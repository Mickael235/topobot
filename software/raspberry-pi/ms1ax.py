# programme de commande de la station Topcon MS1AX II
#
# utilisation : python ms1ax.py 'COM' 'mode'
#  COM = le port série, COM2 ou \dev\ttyUSB0 par exemple
#  mode : la commande parmis
#     std : passe en mode standard de mesure
#     trk : passe en mode tracking
#     mes : lance une mesure en mode standard
#     suivi  : lance un suivi et une mesure continue, stoppée par touche "q"


import sys
import serial
import keyboard
import time


#--------------------- FONCTIONS ----------------
def readport(port,wait):
   ok=0
   i=15 #attente max 15 secondes
   while ((ok==0) and (i>0)):
      result=str(port.readline().rstrip(),'utf-8')
      if (result.find(wait)>-1):
         ok=1
      else:
         i=i-1
         time.sleep(1)
      #end ifs
   #end while
   return result      

def mode_standard(port):
    port.write(str.encode("*/PA 1,0,, \r\n")) # mode search
    r=readport(port,'\x06')
    port.write(str.encode("*/PH 0 \r\n")) # pointé précis
    r=readport(port,'\x06')
    port.write(str.encode("Xa\r\n")) # measurement mode Fine S
    r=readport(port,'\x06')

    print ('--> Mode de mesure standard')

def mode_tracking(port):
    port.write(str.encode("*/PA 1,1,, \r\n")) # mode track
    r=readport(port,'\x06')
    port.write(str.encode("*/PH 1 \r\n")) # pointé rapide
    r=readport(port,'\x06')
    port.write(str.encode("Xe\r\n")) # measurement mode track
    r=readport(port,'\x06')

    print ('--> Mode de mesure tracking')

def do_measure(port):
    port.write(str.encode("*ST2\r\n")) # mesure
    result=readport(port,'*ST2')
    tab=result.split(',')
    print('Mesure | Hz='+tab[2]+" gon / V ="+tab[3]+ " gon / D = "+tab[4]+" m")


def stop_tracking(event):
    global track
    track=False
    
def listen_serial(port):
    result=readport(port,'*ST2')
    tab=result.split(',')
    print('Mesure | Hz='+tab[2]+" gon / V ="+tab[3]+ " gon / D = "+tab[4]+" m")    

# --------- PRINCIPAL --------------

if (len(sys.argv)<3):
    print('usage : ms1ax.py COM MODE')
    sys.exit()

port=sys.argv[1]
mode=sys.argv[2]


port=serial.Serial(port,baudrate=9600,timeout=1.0)

if (mode=='std'):
    mode_standard(port)
if (mode=='trk'):
    mode_tracking(port)
if (mode=='mes'):
    do_measure(port)
    
if (mode=='suivi'):
   # lance la commande de suivi automatisé
   port.write(str.encode("*ST2\r\n")) # mesure
   print("Appuyer une touche pour stopper")
   track=True
   keyboard.on_press(stop_tracking)
   while track:
      listen_serial(port)
      time.sleep(0.5)
   port.write(str.encode("*ST0\r\n")) # stoppe mesure
   port.write(str.encode("*Q\r\n")) # stoppe suivi
   port.flushInput()
   print("Mesure continue stoppée")
      
    

port.close()

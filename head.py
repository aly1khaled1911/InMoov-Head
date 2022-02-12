from threading import Thread
import time
from time import sleep , perf_counter_ns # measures time in nano seconds
import speech_recognition as sr
import pyttsx3
engine = pyttsx3.init()
engine.setProperty('rate', 130)
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[3].id)
engine.setProperty('volume',10)
r=sr.Recognizer()
mic=sr.Microphone(device_index=1)

from board import SCL, SDA
import busio
import RPi.GPIO as GPIO
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)# Create a simple PCA9685 class instance.
pca.frequency = 50
neckhor = servo.Servo(pca.channels[15])#head (30 to 160)
neckver = servo.Servo(pca.channels[14])#head (0 to 110)
jaw = servo.Servo(pca.channels[13])#head (0 to 110)
#oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, reset=reset_pin, addr=0x40)


#GPIO SETUP
channel1 = 21  #right sensor pin number
channel2 = 20  #left sensor pin number
channel3 = 6  #front sebsor pin number
channel4 = 5  #down sensor pin number
n=3
GPIO.setmode(GPIO.BCM) #connection mode as BCM ("Boardcom SOC channel" change according to rasspbery's version number) 

#setting the sensors pins as inputs
GPIO.setup(channel1, GPIO.IN )
GPIO.setup(channel2, GPIO.IN )
GPIO.setup(channel3, GPIO.IN )
GPIO.setup(channel4, GPIO.IN )


#giving the initial and normal angles for horizontal and vertical servoes
normal_angle_hor=100
normal_angle_ver=65
count_start=0 #initialize the counter

def order():
    global n
    global spoken
    t2 = Thread(target=move_jaw)
    t3 = Thread(target=say)
    spoken="can i help you"
    t2.start()
    t3.start()
    while True:
        t2 = Thread(target=move_jaw)
        t3 = Thread(target=say)
        with mic as source:
            try:
                audio=r.listen(source)
                words= r.recognize_google(audio,language='en')
                t2 = Thread(target=move_jaw)
                t3=Thread(target=say)
                print (words)
                x=angle_hor
                if words == "how are you":
                    n=10
                    spoken ="i am fine, well at least my head is fine, thank you"
                    t2.start()
                    t3.start()
                elif words=="meet the doctor":
                    n=9
                    spoken='nice to meet you doctor, Please give them full marks'
                    t2.start()
                    t3.start()
                elif words=="what's your name":
                    n=5
                    spoken='my name is robot and you ?'
                    t2.start()
                    t3.start()
                elif words=="turn left":
                    n=2
                    spoken='at your service'
                    t2.start()
                    t3.start()
                    sleep(1)
                    neckhor.angle=150
                    angle_hor=150
                    sleep(1)
                elif words=="turn back":
                    n=2
                    spoken='at your service'
                    t2.start()
                    t3.start()
                    sleep(1)
                    neckhor.angle=x
                    angle_hor=x
                elif words=="can you help me":
                    n=3
                    spoken='I am at your service'
                    t2.start()
                    t3.start()
                elif words=="thank you":
                    n=3
                    spoken='you are welcome'
                    t2.start()
                    t3.start()
                    sleep(2)
                    break
                elif (words.split()[0])=="my":
                    z=words.split()
                    n=3
                    spoken="hello"+z[-1]
                    t2.start()
                    t3.start()
                    sleep(2)
                else:
                    print(words)
            except:
                sr.UnknownValueError()
def say():
    global spoken
    engine.say(spoken)
    engine.runAndWait()

def turn_neck():
  global angle_hor
  global angle_ver
  global normal_angle_hor
  global normal_angle_ver
  h=normal_angle_hor #normal hor. angle (angle at which robot turned front in horizontal)
  v=normal_angle_ver  #normal vertical angle (angle at which robot turned middle in vertical)
  if (angle_ver >=normal_angle_ver):     #up
    if (angle_hor >=normal_angle_hor):  #left
        while (True):
            neckhor.angle = h  #always start from front 
            neckver.angle = v  #always start from middle
            sleep(0.03)
            if (h!=angle_hor):
                h=h+1
            if (v!=angle_ver):
                v=v+1
            if (h==angle_hor and v==angle_ver):  # change angle till it reach to left up direction
                sleep(1)
                break
    elif(angle_hor <=normal_angle_hor): # if right up directon
        while (True):
            neckhor.angle = h   #always start from front 
            neckver.angle = v   #always start from middle
            sleep(0.03)
            if (h!=angle_hor):
                h=h-1
            if(v!=angle_ver):
                v=v+1
            if (h==angle_hor and v==angle_ver):  # change angle till it reach to righrt up direction
                sleep(1)
                break
  elif (angle_ver <=normal_angle_ver):  #if down
    if (angle_hor >=normal_angle_hor): #if left 
        while (True):
            neckhor.angle = h  #always start from front 
            neckver.angle = v  #always start from middle
            sleep(0.03)
            if (h!=angle_hor):
                h=h+1
            if(v!=angle_ver):
                v=v-1
            if (h==angle_hor and v==angle_ver):  # change angle till it reach to left down direction
                sleep(1)
                break
    elif(angle_hor <=normal_angle_hor): #if right down direction
        while (True):
            neckhor.angle = h  #always start from front 
            neckver.angle = v  #always start from middle
            sleep(0.03)
            if (h!=angle_hor):
                h=h-1
            if(v!=angle_ver):
                v=v-1
            if (h==angle_hor and v==angle_ver):  # change angle till it reach to right down direction
                sleep(1)
                break
            
def turn_neck_back():
  global normal_angle_hor
  global normal_angle_ver
  global angle_hor
  global angle_ver
  h=angle_hor  #set the horizontal variable with with the current hor. angle from (turn_neck) function (start angle)
  v=angle_ver  #set the vertical variable with with the current ver. angle from (turn_neck) function (start angle)
  if (angle_ver >=normal_angle_ver):    #if neck is in up position currently from the (turn_neck) function
    if (angle_hor >=normal_angle_hor): #if neck is in left position currently from the (turn_neck) function 
        while (True):
            neckhor.angle = h
            neckver.angle = v 
            sleep(0.02)
            if (h!=normal_angle_hor):
                h=h-1
            if (v!=normal_angle_ver):
                v=v-1
            if (h==normal_angle_hor and v==normal_angle_ver): #change the hor.and ver. angles till they reach to normal (front middle) position
                sleep(1)
                break
    elif(angle_hor <=normal_angle_hor):  #if neck is in righr up position currently from the (turn_neck) function 
        while (True):
            neckhor.angle = h
            neckver.angle = v 
            sleep(0.02)
            if (h!=normal_angle_hor):
                h=h+1
            if(v!=normal_angle_ver):
                v=v-1
            if (h==normal_angle_hor and v==normal_angle_ver): #change the hor.and ver. angles till they reach to normal (front middle) position
                sleep(1)
                break
  elif (angle_ver <=normal_angle_ver):    #if neck is in down position currently from the (turn_neck) function 
    if (angle_hor >=normal_angle_hor):   #if neck is in left position currently from the (turn_neck) function 
        while (True):
            neckhor.angle = h
            neckver.angle = v 
            sleep(0.01)
            if (h!=normal_angle_hor):
                h=h-1
            if(v!=normal_angle_ver):
                v=v+1
            if (h==normal_angle_hor and v==normal_angle_ver): #change the hor.and ver. angles till they reach to normal (front middle) position
  #              sleep(1)
                break
    elif(angle_hor <=normal_angle_hor):   #if neck is in right down position currently from the (turn_neck) function 
        while (True):
            neckhor.angle = h
            neckver.angle = v 
            sleep(0.01)
            if (h!=normal_angle_hor):
                h=h+1
            if(v!=normal_angle_ver):
                v=v+1
            if (h==normal_angle_hor and v==normal_angle_ver): #change the hor.and ver. angles till they reach to normal (front middle) position
#                 sleep(1)
                break
def move_jaw():
        global n
        for i in range (n):
            jaw.angle = 60
            sleep(0.2)
            jaw.angle = 0
            sleep(0.2)
#............................................................................................................
#function that takes the front,right,left,down capture arguments and sets the required vertical angle
def callback(front,right,left,down):
    global count_start
    global angle_ver
    global angle_hor
    if (angle_ver!=25 and angle_ver!=50):   #if the down sensor detected the signal first (then ver. angle is already given)
        diff=perf_counter_ns()-count_start   #calculates the diffrence in time between the up and down sensor's signals
        
        if ((front==1 or right==1 or left==1 )and down==0): #up  
            while (GPIO.input(channel4)==0 and (diff<555555)): #wait for the down sensor signal or diffrence in time if exeaded the 1111111
              diff=perf_counter_ns()-count_start  #keep on capturing the diffrence in time between the up and down sensor's signals                  
            if (diff>555555 or diff>300000):    #if while loop was exsites bec 555555 is exceeded or down signal is detected after 111111
              print ("count=" ,diff)
              print ("Sound Detected up!")
              angle_ver=120              
            elif (diff<300000 and diff>33333):  #if while loop was exsites bec down signal is detected between 111111 and 55555 diff. in time
              print ("count=" ,diff)
              print ("Sound Detected middle!")
              angle_ver=normal_angle_ver              
            else:                               #if while loop was exsites bec down signal is detected between in less than 55555 diff. in time
              print ("count=" ,diff)
              print ("Sound Detected down 45!")
              angle_ver=40                
        elif ((front==1 or right==1 or left==1 ) and down==1): #if both up and down sensors where detected at the same time
            print ("count=" ,diff)
            print ("Sound Detected down 45!11")
            angle_ver=40
    turn_neck() #at this step both horizontal and vertical angles where given and ready to be used in (turn_neck) function
    print (angle_hor , angle_ver)
    


def hor_detect(right ,left ,front): #this function takes (right ,left ,front) captured arguments and sets the required horizontal angle
    global angle_hor
    ff=1  #flage that detectes if the counter finished before the front sensor's signal is detected 
    if (right==1 and left==0 and  front==0 ): #if the right sensor detected the sound first
           print ("right" )
           angle_hor=50
    elif (right==0 and left==1 and  front==0 ): #if the left sensor detected the sound first
                print ("left")
                angle_hor=150
    elif (right==0 and left==0 and  front==1 ): #if the front sensor detected the sound first
          print ("front1")
          angle_hor=normal_angle_hor
    elif (right==1 and left==1 and  front==0 ): #if the right & left  sensors detected the sound at the same time
        print ("front2")
        angle_hor=normal_angle_hor
    elif (right==1 and left==1 and  front==1 ): #if the right & left & front  sensors detected the sound at the same time
        print ("front3")
        angle_hor=normal_angle_hor
    elif (right==1 and left==0 and  front==1 ): #if the right & front sensors detected the sound at the same time
        print ("right 45")
        angle_hor=80
    elif (right==0 and left==1 and  front==1 ): #if the left & front sensors detected the sound at the same time
        print ("left 45")
        angle_hor=130
    elif (right==0 and left==0 and  front==0 ): #if no signal is detected
        print ("zero break")
    callback(front,right,left,down) #call functinon that sets the ver. angle since the distance between ver. sensors > hor.sensors
    
#.........................................................................................................

while True:
    
    neckver.angle = normal_angle_ver   #set the initial middle vertical angle
    neckhor.angle=normal_angle_hor    #set the initial front horizontal angle 
    sleep(2)
    print("start")
    angle_ver=normal_angle_ver
    #capture the right & left & front & down sensor states at the bigening of each cycle
    right=GPIO.input(channel1) 
    left=GPIO.input(channel2)
    front=GPIO.input(channel3)
    down=GPIO.input(channel4)
    while (right ==0 and left==0 and front==0 and down==0): #(polling) wait for a signal from right/left/front/down sensors (wait for any sound )
      right=GPIO.input(channel1)
      left=GPIO.input(channel2)
      front=GPIO.input(channel3)
      down=GPIO.input(channel4)
    if (down==1):   # if the down sensor detected the signal first (sound source is down )
              count_start=perf_counter_ns()  #start counting time by taking a capture of time to be used later in gettin time diffrence (diff)
              while (right==0  and left==0 and  front==0 ): # now wait for any signal from the up horizontal sensors
                right=GPIO.input(channel1)
                left=GPIO.input(channel2)#llllllllllllllllllll if only down detected
                front=GPIO.input(channel3)
                down=GPIO.input(channel4)
             
              print ("Sound Detected down !" )
              angle_ver=25
    count_start=perf_counter_ns()  #start counting time by taking a capture of time to be used later in gettin time diffrence (diff)   
    hor_detect(right ,left ,front) #call function that sets the horizontal angle
    order()
    sleep(0.3)
    turn_neck_back() #call the function that turn back the robot's head to normal (front middle) position
#   sleep(0.5)       
    print ("..............")       
t1 = Thread(target=order)
t2 = Thread(target=move_jaw)
t3 = Thread(target=say)
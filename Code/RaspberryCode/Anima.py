import tflite_runtime.interpreter as tflite
from picamera import PiCamera
import numpy as np
import time
import serial
from pygame import mixer
import threading
import math
import random


### AI Code Section ###
class AIModel():
    def __init__(self):
        super(AIModel, self).__init__()
        POSE_PATH = '/home/pi/Desktop/RAD/PoseNet.tflite'
        self.camera = PiCamera()
        self.camera.framerate = 30
        self.cont=0
        
        self.mov_class = 'M'
        self.detected = False
        self.target = None
        self.distance = 0
        self.centered = False
        self.AIMode = 'detect'
        
        interpreter = tflite.Interpreter(model_path=POSE_PATH, num_threads=3)
        s = interpreter.get_signature_list()['serving_default']
        self.inputsig = s['inputs'][0]
        self.outputsig = s['outputs'][0]
        self.model = interpreter.get_signature_runner()
    
    def PoseToClass(self, out):
        c = 'M'
        if(out[3][2]>0.4 and out[4][2]>0.4
           and out[5][2]>0.1 and out[6][2]>0.1):
            #set threshold for precision dependant on
            #person by person fixed characteristic
            T = abs(out[3][1] - out[4][1]) * 0.5
            
            #class I: left shoulder x = left elbow x
            # and right shoulder y = right elbow y
            # and left Shoulder y = left elbow y
            if(abs(out[3][0]-out[5][0])<T and
                 abs(out[4][0]-out[6][0])<T and
                 abs(out[3][1]-out[5][1])<T):
                c = 'I'
            #class L: left shoulder x = left elbow x
            # and right shoulder y = right elbow y
            # and right Shoulder y = left elbow y
            elif(abs(out[3][0]-out[5][0])<T and
                 abs(out[4][0]-out[6][0])<T and
                 abs(out[4][1]-out[6][1])<T):
                c = 'L'
            #class A: left shoulder y = left elbow y
            # and right shoulder y = right elbow y
            elif(abs(out[3][0]-out[5][0])<T and abs(out[4][0]-out[6][0])<T):
                c = 'A'
            #class B: left shoulder y = left elbow y
            # and right shoulder x = right elbow x
            elif(abs(out[3][0]-out[5][0])<T and abs(out[4][1]-out[6][1])<T):
                c = 'B'
            #class C: left shoulder x = left elbow x
            # and right shoulder y = right elbow y
            elif(abs(out[3][1]-out[5][1])<T and abs(out[4][0]-out[6][0])<T):
                c = 'C'
            #class D: left shoulder x = left elbow x
            # and right shoulder x = right elbow x
            #and left arm pointing down: left shoulder-left elbow <0
            elif(abs(out[3][1]-out[5][1])<T
                 and abs(out[4][1]-out[6][1])<T
                 and (out[3][0]-out[5][0])<0
                 and (out[4][0]-out[6][0])>0):
                c = 'D'
            #class E: left shoulder x = left elbow x
            # and right shoulder x = right elbow x
            #and left arm pointing up: left shoulder-left elbow <0
            #and right arm pointing down
            elif(abs(out[3][1]-out[5][1])<T
                 and abs(out[4][1]-out[6][1])<T
                 and (out[3][0]-out[5][0])>0
                 and (out[4][0]-out[6][0])<0):
                c = 'E'
            #class H: left shoulder x = left elbow x
            # and right shoulder x = right elbow x
            #and left arm pointing up: left shoulder-left elbow <0
            #and right arm pointing up
            elif(abs(out[3][1]-out[5][1])<T
                 and abs(out[4][1]-out[6][1])<T
                 and (out[3][0]-out[5][0])>0
                 and (out[4][0]-out[6][0])>0):
                c = 'H'
        else:
            c='out_arms'
        
        if c=='M':
            if(out[1][2]>0.2 and out[2][2]>0.2):
                T = abs(out[1][1] - out[2][1]) / 3
                #class F: left eye - righteye < -T
                #(left eye higher than right by T margin)
                if(out[1][0]-out[2][0] < -T):
                    c='F'
                #class G: left eye - righteye < -T
                #(left eye lower than right by T margin)
                if(out[1][0]-out[2][0] > T):
                    c='G'
            else:
                c='out_eyes'
        self.mov_class = c
    
    def Detect(self, out):
        mov_class = 'M'
        detected = False
        target = None
        distance = 0
        centered = False
        
        #check the confidence of identification of torso and eyes markers
        
        avgtorso = (out[3][2]+out[4][2]+out[5][2]+out[6][2])/4
        avgeyes = (out[0][2]+out[1][2]+out[2][2])/3
        
        #if both are above 60%, a person has been detected,
        #our target is the nose of the person, we calculate the distance from camera
        # and we check if the person is centered in the frame
        if avgtorso > 0.5 and avgeyes>0.5:
            detected=True
            torso_h = out[5][0]-out[3][0]
            delta = torso_h * 48
            p =60*math.cos(math.radians(15))
            distance = p/math.tan(math.radians(delta))
            target = (90 + (0.5-out[0][0])*48,
                      90+(out[0][1]-0.5)*60) #vertical angle, #horizontal angle
            if abs(((out[3][1]+out[4][1])/2)-0.5) < 0.2:
                centered=True
                
        
        #if instead only the torso is above 60%, a person has been detected,
        #our target is estimated starting from his right shoulder
        #we calculate distance from camera and we check if the person is centered
        elif avgtorso > 0.5 and avgeyes<0.5:
            detected=True
            torso_h = out[5][0]-out[3][0]
            delta = torso_h * 48
            p = 60*math.cos(math.radians(15))
            distance = p/math.tan(math.radians(delta))
            target = (90+(0.5-out[0][0])*48,
                      90+(out[0][1]-0.5)*60)  #vertical angle, horizontal angle
            if abs(((out[3][1]+out[4][1])/2)-0.5) < 0.2:
                centered=True
                
        self.detected = detected
        self.target = target
        self.distance = distance
        self.centered = centered
                           
    
    def PoseNet(self, output):
        # take nose, eyes, shoulders, elbows
        out = (output[0][0][0], output[0][0][1],
               output[0][0][2], output[0][0][5], output[0][0][6],
               output[0][0][7], output[0][0][8])
        #legend for output of the model:
        #print("nose: "+str(out[0])+"\nlefteye: "+str(out[1])+"\nrighteye: "
         #     +str(out[2])+"\nleftshoulder: "+str(out[3])+"\nrightshoulder"+
          #    str(out[4])+"\nleftelbow: "+str(out[5])+"\nrightelbow"+str(out[6]))
        self.PoseToClass(out)
    
    def PersonDetection(self, output):
        #take nose eyes shoulders hips
        out = (output[0][0][0], output[0][0][1],
               output[0][0][2], output[0][0][5], output[0][0][6],
               output[0][0][11], output[0][0][12])
        self.Detect(out)

    def infer(self):
        image = np.empty((256, 256, 3), dtype=np.uint8)
        self.camera.capture(image, 'rgb', resize=(256,256),
                            use_video_port=True)
        result = self.model(input=image)
        output = result[self.outputsig]
        if self.AIMode == 'posenet':
            self.PoseNet(output)
        elif self.AIMode == 'detect':
            self.PersonDetection(output)
        
    
def AILoop(AI):
    while True:
        AI.infer()
        

### Main Part ###

#function to move back to initial position
def resetPos():
    msg = "M: 0:30, 0:100, 0:30, 0:100, 90:50, 108:50, 90:100, 180:50\n"
    SendCommand(msg)

#function to send messages to Arduino and wait for its completio response
def SendCommand(msg):
    Arduino.write(msg.encode())
    ack=Arduino.readline().decode()
    while ack != "Done\r\n":
        ack=Arduino.readline().decode()
        print(ack)

# function with pre-made commands for each class of movement. we can choose class,
# speed of movement and wether to actuate breathing or not
def MOV_Class(c, speed, breath):
    if breath==1:
        b=160
    else:
        b=300
        
    if c=='A':
        msg = "M: 55:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 55:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 90:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='B':
        msg = "M: 40:"+str(int(speed))+", 180:"+str(int(speed*1.8))+", 55:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 90:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='C':
        msg = "M: 55:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 40:"+str(int(speed))+", 180:"+str(int(speed*1.8))+", 90:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='D':
        msg = "M: 40:"+str(int(speed))+", 180:"+str(int(speed*1.8))+", 0:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 90:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='E':
        msg = "M: 0:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 40:"+str(int(speed))+", 180:"+str(int(speed*1.8))+", 90:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='G':
        msg = "M: 0:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 0:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 105:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='F':
        msg = "M: 0:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 0:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 75:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='H':
        msg = "M: 40:"+str(int(speed))+", 180:"+str(int(speed*1.8))+", 40:"+str(int(speed))+", 180:"+str(int(speed*1.8))+", 90:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='I':
        msg = "M: 55:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 0:"+str(int(speed))+", 90:"+str(int(speed*1.8))+", 90:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='L':
        msg = "M: 0:"+str(int(speed))+", 90:"+str(int(speed*1.8))+", 55:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 90:"+str(int(speed/4))+", 112:"+str(int(speed/4))+", 90:"+str(int(speed))+", "+str(int(b))+":"+str(int(speed/3))+"\n"
        SendCommand(msg)
    if c=='M':
        msg = "M: 0:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 0:"+str(int(speed))+", 0:"+str(int(speed*1.8))+", 90:"+str(int(speed/4))+", 108:"+str(int(speed/4))+", 90:"+str(int(speed))+", 180:"+str(int(speed/3))+"\n"
        SendCommand(msg)
    
  
def PointFace():
    if(AI.target!=None):
        (v, h) = AI.target
        d = AI.distance
        alpha = 180-v-15
        #Teorema di Carnot
        a = math.sqrt((d**2)+(6**2)-2*d*6*math.cos(math.radians(alpha)))
        beta = math.degrees(math.asin((6*math.sin(math.radians(alpha)))/a))
        #Teorema di Eulero
        v = 180-alpha-beta
        if v>108:
            v=108
        msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, "+str(int(v))+":50, "+str(int(h))+":50, 300:1\n"
        SendCommand(msg)
    

def IdleMode():
    print("idle")
    hy = 95
    hz = 90
    boostdx=0
    boostsx=0
    
    while AI.detected==False:
        # we define a range of angle update range depending on the current position and previous movement,
        # in order to augment the likelihood that the random movements will not drift to extreme position
        # by consecutive movements in the same direction
        if hz>90:
            boostdx=30
            boostsx=-20
        else:
            boostdx=-20
            boostsx=30
        hy = random.randint(95, 115)
        rhzp = min(50+boostdx, hz) 
        rhzm = min(50+boostsx, 180-hz)
        hz = random.randint(hz-rhzp, hz+rhzm)
        if hz>144:
            msg = "L: fa\n"
        elif hz<=144 and hz>108:
            msg = "L: fb\n"
        elif hz<=108 and hz>72:
            msg = "L: fc\n"
        elif hz<=72 and hz>36:
            msg = "L: fd\n"
        elif hz<=36:
            msg = "L: fe\n"
        SendCommand(msg)
        time.sleep(1)
        
        msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, "+str(int(hy))+":50, "+str(int(hz))+":50, 300:1\n"
        SendCommand(msg)
        
        msg = "L: fc\n"
        SendCommand(msg)
        time.sleep(2)
    
    AI.AIMode='detect'
    ISeeYou()
        
def ISeeYou():
    print("I see you")
    count=0
    
    msg = "L: ff\n"
    SendCommand(msg)

    #keep repeating the attraction movements until one of the outcomes defined by the conditions become clear
    start = time.time()
    while (AI.centered==False or AI.distance<150 or count<2) and time.time()-start<20:
        PointFace()
        time.sleep(0.5)
        count = count+1
        
        time.sleep(0.5)
        
        mixer.music.load("/home/pi/Desktop/RAD/Audio 1.mp3")
        mixer.music.play()
        
        #first attraction movement phase
        msg = "M: 40:80, 300:1, 40:80, 300:1, 80:20, 300:1, 300:1, 300:1\n"
        SendCommand(msg)
        
        #second attraction movement phase
        msg = "M: 0:80, 300:1, 0:80, 300:1, 100:20, 300:1, 300:1, 300:1\n"
        SendCommand(msg)
        
        msg = "M: 300:1, 300:1, 300:1, 300:1, 90:30, 300:1, 300:1, 300:1\n"
        SendCommand(msg)
        time.sleep(0.5)
    
    if time.time()-start>20:
        #doesnt reach interaction position during I See you
        time.sleep(1)
        Sad()
        return 0
    else:
        time.sleep(1)
        MirrorMe()
        return 1

def MirrorMe():
    print("Mirror Me")
  
    resetPos()
    time.sleep(0.5)
    
    start = time.time()
    follows = False
    isthere = True
    
    msg = "L: fg\n"
    SendCommand(msg)
    
    msg = "L: l1b\n"
    SendCommand(msg)
    
    while follows==False and isthere==True and time.time()-start<20:
        AI.AIMode = 'posenet'
        mixer.music.load("/home/pi/Desktop/RAD/Audio 2.mp3")
        mixer.music.play()
        time.sleep(0.5)

        msg = "M: 55:50, 300:1, 55:50, 300:1, 300:1, 112:10, 300:1, 160:10\n"
        SendCommand(msg)
        time.sleep(1.5)
        
        msg = "M: 0:50, 300:1, 0:50, 300:1, 300:1, 108:10, 300:1, 180:10\n"
        SendCommand(msg)
        time.sleep(1)
        
        msg = "M: 55:50, 300:1, 45:50, 300:1, 300:1, 112:10, 300:1, 160:10\n"
        SendCommand(msg)
        time.sleep(1)
        
        mixer.music.stop()
        time.sleep(3)
        print(AI.mov_class)
        if AI.mov_class == 'A':
            follows=True
        else:
            AI.AIMode = 'detect'
            time.sleep(1)
            if AI.centered == False or AI.centered>300 or AI.detected==False:
                isthere=False
            else:
                follows=False
                
        resetPos()
        time.sleep(0.5)
        
    if follows==True:
        msg = "L: l1\n"
        SendCommand(msg)
        time.sleep(1)
        Happy()
        time.sleep(1)
        Stretch(0)
        return 1
    if isthere==False or time.time()-start>15:
        #leaves during MirrorMe
        time.sleep(1)
        Sad()
        return 0
    if follows==False:
        time.sleep(1)
        Sassy()
        time.sleep(1)
        MirrorMe()
        return 2
                
        
def Happy():
    print("Happy")
    resetPos()
    
    msg = 'L: fl\n'
    SendCommand(msg)
    
    mixer.music.load("/home/pi/Desktop/RAD/Audio 5.mp3")
    mixer.music.play()
    
    msg = "M: 55:100, 90:100, 55:100, 90:100, 300:1, 300:1, 300:1, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 0:100, 300:1, 0:100, 300:1, 300:1, 300:1, 300:1, 300:1\n"
    SendCommand(msg)
    time.sleep(0.5)
    
    msg = "M: 55:100, 90:100, 55:100, 90:100, 300:1, 300:1, 300:1, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 0:100, 300:1, 0:100, 300:1, 300:1, 300:1, 300:1, 300:1\n"
    SendCommand(msg)
    
    time.sleep(0.5)
    resetPos()
    mixer.music.stop()


def Sad():
    print("Sad")
    resetPos()
    
    mixer.music.load("/home/pi/Desktop/RAD/Audio 3.mp3")
    mixer.music.play()
    time.sleep(1)
    
    msg = "L: l3s\n"
    SendCommand(msg)
    time.sleep(0.5)
    msg = "L: l2s\n"
    SendCommand(msg)
    time.sleep(0.5)
    msg = "L: l1s\n"
    SendCommand(msg)
    
    msg = 'L: fh\n'
    SendCommand(msg)
    
    time.sleep(0.5)
    msg = "M: 30:100, 300:1, 30:100, 300:1, 300:1, 300:1, 300:1, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 0:100, 300:1, 0:100, 300:1, 300:1, 95:30, 110:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 70:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 110:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 70:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 110:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 70:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 110:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 90:100, 300:1\n"
    SendCommand(msg)
    
    time.sleep(2)
    mixer.music.stop()
    resetPos()
    

def Sassy():
    print("Sassy")
    resetPos()
    
    mixer.music.load("/home/pi/Desktop/RAD/Audio 6.mp3")
    mixer.music.play()
    time.sleep(1)
    
    msg = 'L: fi\n'
    SendCommand(msg)
    time.sleep(0.5)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 70:50, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 110:50, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 70:50, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 110:50, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 90:50, 300:1\n"
    SendCommand(msg)
    
    resetPos()


def Angry():
    print("Angry")
    resetPos()
    
    msg = "L: l3s\n"
    SendCommand(msg)
    time.sleep(0.5)
    msg = "L: l2s\n"
    SendCommand(msg)
    time.sleep(0.5)
    msg = "L: l1s\n"
    SendCommand(msg)
    
    mixer.music.load("/home/pi/Desktop/RAD/Audio 7.mp3")
    mixer.music.play()
    time.sleep(1)
    
    msg = 'L: fn\n'
    SendCommand(msg)
    
    msg = "M: 300:1, 30:100, 300:1, 30:100, 300:1, 300:1, 70:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 0:100, 300:1, 0:100, 300:1, 300:1, 110:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 30:100, 300:1, 30:100, 300:1, 300:1, 70:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 0:100, 300:1, 0:100, 300:1, 300:1, 110:100, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 90:50, 300:1\n"
    SendCommand(msg)
    
    resetPos()
    
def Stretch(chance):
    print("Stretch")
    
    resetPos()
    time.sleep(0.5)

    # extract 2 different moves from the possible classes of movements
    bucket = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'L']
    sequence = []
    for i in range(0, 2):
        rand = random.randint(0, len(bucket)-1)
        sequence.append(bucket.pop(rand))
        
        
    start = time.time()
    follows = False
    isthere = True
    
    msg = "L: fg\n"
    SendCommand(msg)
    
    msg = "L: l2b\n"
    SendCommand(msg)
    
    for c in sequence:
        AI.AIMode = 'posenet'
        mixer.music.load("/home/pi/Desktop/RAD/Audio 2.mp3")
        mixer.music.play()
        time.sleep(1)
        
        MOV_Class(c, 100, 1)
        time.sleep(1)
    
        mixer.music.stop()
        time.sleep(2)
        print(AI.mov_class + ", "+c)
        if AI.mov_class == c:
            follows=True
        else:
            AI.AIMode = 'detect'
            time.sleep(1)
            if AI.centered == False or AI.centered>250 or AI.detected==False:
                isthere=False
            else:
                follows=False
                
        resetPos()
        time.sleep(0.5)
        if follows==True:
            continue
        if chance>2:
            time.sleep(1)
            Angry()
            return 2
        if isthere==False:
            time.sleep(1)
            Sad()
            return 0
        if time.time()-start>15 or follows==False:
            time.sleep(1)
            Sassy()
            #Stretch second chance
            time.sleep(1)
            chance=chance+1
            Stretch(chance)
            return 0
    
    msg = "L: l2\n"
    SendCommand(msg)
    time.sleep(1)
    Happy()
    time.sleep(1)
    Active(0)
    return 1
    

def Active(chance):
    # extract one of the four exercises, which are expressed as a series of class movements
    resetPos()
    bucket = [['A', 'H', 'A', 'M'], ['D', 'E'], ['B', 'C'], ['I', 'L']]
    rand = random.randint(0, len(bucket)-1)
    
    msg = "L: fg\n"
    SendCommand(msg)
    
    msg = "L: l3b\n"
    SendCommand(msg)
    
    follows=True
    isthere=True
    
    AI.AIMode='posenet'
    
    start=time.time()

    #We check only the first and last movement after a certain time of looping on
    # the extracted exercise classes. this allows us faster more meaningful sequences,
    # while being reasonably sure that the user has followed the movements.

    cont=0
    c = bucket[rand][cont]
    MOV_Class(c, 100, 0)
    cont+=1
    
    time.sleep(3)
    print(AI.mov_class + ", "+c)
    if AI.mov_class != c:
        follows=False
        
    AI.AIMode = 'detect'
    time.sleep(2.5)
    
    while time.time()-start<len(bucket[rand])*6 and follows==True and isthere==True:
        c = bucket[rand][cont]
        MOV_Class(c, 100, 0)
        print(cont)
        
        time.sleep(0.6/len(bucket[rand]))   
        
        cont = cont + 1
        if cont==len(bucket[rand]):
            cont=0
        
    if AI.centered == False or AI.distance>250 or AI.detected==False:
            isthere=False
            
    AI.AIMode='posenet'
    
    if isthere==False:
        time.sleep(1)
        Sad()
    elif follows==False and chance<2:
        time.sleep(1)
        Sassy()
        time.sleep(1)
        Active(chance+1)
    elif chance==2:
        time.sleep(1)
        Angry()
    elif isthere==True and follows==True:
        time.sleep(2)
        print(AI.mov_class + ", "+c)
        if AI.mov_class == c:
            msg = "L: l3\n"
            SendCommand(msg)
            time.sleep(1)
            Happy()
            time.sleep(1)
            ByeBye()
        elif chance<2:
            time.sleep(1)
            Sassy()
            time.sleep(1)
            Active(chance+1)
        elif chance==2:
            time.sleep(1)
            Angry()
        
def Salute():
    msg = "L: fg\n"
    SendCommand(msg)
    msg = "M: 300:1, 300:1, 40:10, 180:80, 300:1, 300:1, 300:1, 300:1\n"
    SendCommand(msg)
    
    mixer.music.load("/home/pi/Desktop/RAD/Audio 4.mp3")
    mixer.music.play()
    time.sleep(1.5)
    msg = "L: fo\n"
    SendCommand(msg)
    msg = "M: 300:1, 300:1, 50:100, 300:1, 300:1, 300:1, 300:1, 300:1\n"
    SendCommand(msg)

def Japanese():
    msg = "L: fc\n"
    SendCommand(msg)
    
    mixer.music.load("/home/pi/Desktop/RAD/Audio 4.mp3")
    mixer.music.play()
    time.sleep(1.5)
    
    msg = "L: fg\n"
    SendCommand(msg)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 95:20, 300:1, 300:1\n"
    SendCommand(msg)
    time.sleep(1)
    
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 108:20, 300:1, 300:1\n"
    SendCommand(msg)
 
 
def Evil():
    msg = "L: fc\n"
    SendCommand(msg)
    
    mixer.music.load("/home/pi/Desktop/RAD/Evil.mp3")
    mixer.music.play()
    time.sleep(1.5)
    
    msg = "M: 58:30, 180:80, 58:30, 180:80, 300:1, 112:20, 300:1, 300:1\n"
    SendCommand(msg)
    
    msg = "L: fn\n"
    SendCommand(msg)
    
    time.sleep(1)
    
    msg = "M: 40:50, 300:1, 40:50, 300:1, 300:1, 108:20, 300:1, 300:1\n"
    SendCommand(msg)
    msg = "M: 0:50, 300:1, 0:50, 300:1, 300:1, 108:20, 300:1, 300:1\n"
    SendCommand(msg)
    
    msg = "M: 40:50, 300:1, 40:50, 300:1, 300:1, 108:20, 300:1, 300:1\n"
    SendCommand(msg)
    msg = "M: 0:50, 300:1, 0:50, 300:1, 300:1, 108:20, 300:1, 300:1\n"
    SendCommand(msg)

def Sium():
    msg = "L: fc\n"
    SendCommand(msg)
    
    mixer.music.load("/home/pi/Desktop/RAD/sium.mp3")
    mixer.music.play()
    time.sleep(1.5)
    
    msg = "M: 300:1, 90:50, 300:1, 90:50, 300:1, 300:1, 300:1, 300:1\n"
    SendCommand(msg)
    time.sleep(0.5)
    msg = "M: 30:30, 0:100, 30:30, 0:100, 300:1, 300:1, 300:1, 300:1\n"
    SendCommand(msg)
    
def High():
    msg = "L: fc\n"
    SendCommand(msg)
    
    mixer.music.load("/home/pi/Desktop/RAD/Audio 2.mp3")
    mixer.music.play()
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 160:10\n"
    SendCommand(msg)
    time.sleep(2)
    mixer.music.stop()
    
    msg = "L: l4\n"
    SendCommand(msg)
    mixer.music.load("/home/pi/Desktop/RAD/Audio 4.mp3")
    mixer.music.play()
    
    msg = "L: fp\n"
    SendCommand(msg)
    msg = "M: 30:40, 300:1, 30:40, 300:1, 75:20, 112:20, 100:20, 180:50\n"
    SendCommand(msg)
    
    msg = "L: fq\n"
    SendCommand(msg)
    msg = "M: 300:1, 300:1, 300:1, 300:1, 105:20, 112:20, 80:20, 300:1\n"
    SendCommand(msg)
    
    msg = "L: fr\n"
    SendCommand(msg)
    msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 98:20, 300:1, 300:1\n"
    SendCommand(msg)
    
    msg = "L: fr\n"
    SendCommand(msg)
    msg = "M: 300:1, 300:1, 300:1, 300:1, 75:20, 98:20, 100:20, 300:1\n"
    SendCommand(msg)

def ByeBye():
    mixer.music.load("/home/pi/Desktop/RAD/Audio 8.mp3")
    mixer.music.play()
    time.sleep(1.5)
    resetPos()
    rand = random.randint(0,100)
    if rand<35:
        Salute()
    elif rand>=35 and rand<70:
        Japanese()
    elif rand>=70 and rand<80:
        Sium()
    elif rand>=80 and rand<95:
        Evil()
    elif rand>=95:
        High()
    time.sleep(2)
    resetPos()
    msg = "L: l3s\n"
    SendCommand(msg)
    time.sleep(0.5)
    msg = "L: l2s\n"
    SendCommand(msg)
    time.sleep(0.5)
    msg = "L: l1s\n"
    SendCommand(msg)

def MainLoop():
    AI.AIMode='detect'
    IdleMode()
    
 

Arduino = serial.Serial('/dev/ttyUSB0',9600)
time.sleep(2)

# initial setup for servos and lotus flower LEDs
#M: lsxa:vel: 300:1, lsya:vel: 300:1, rsxa:vel: 300:1, rsya:vel: 300:1, hxa:vel: 300:1, hya:vel: 300:1, hza:vel: 300:1, ba:vel: 300:1\n
msg = "M: 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 300:1, 300:1\n"
SendCommand(msg)
msg = "L: l3s\n"
SendCommand(msg)
time.sleep(0.5)
msg = "L: l2s\n"
SendCommand(msg)
time.sleep(0.5)
msg = "L: l1s\n"
SendCommand(msg)

#AI thread definition
AI = AIModel()
AI.AIMode = 'detect'
AIThread = threading.Thread(target=AILoop, args=(AI,))
AIThread.start()

mixer.init() 
mixer.music.set_volume(100) 

while True:
    MainLoop()
    
    
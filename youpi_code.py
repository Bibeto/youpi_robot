import time
import math
import cv2
import threading
import RPi.GPIO as GPIO



#Pins
sens3 = 37
pas3 = 38
sens2 = 33
pas2 = 35
sens1 = 29    
pas1 = 31  

#Gpio setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False) 

GPIO.setup(sens1,GPIO.OUT)
GPIO.setup(pas1,GPIO.OUT)

GPIO.setup(sens2,GPIO.OUT)
GPIO.setup(pas2,GPIO.OUT)

GPIO.setup(sens3,GPIO.OUT)
GPIO.setup(pas3,GPIO.OUT)

GPIO.output(sens1,GPIO.LOW)
GPIO.output(sens2,GPIO.LOW)
GPIO.output(sens3,GPIO.LOW)

GPIO.output(pas1,GPIO.LOW)
GPIO.output(pas2,GPIO.LOW)
GPIO.output(pas3,GPIO.LOW)

#Length of effector
l1=172
l2=162
l3=162
l4=115

#CurrentAngle
teta = [0 , 52.76, -51.04 ]     #constraint [90 ; -90] == [1440 ; -1440]
                                #constraint [75 ; 0] == [1400 ; 0]
                                #constraint [0  ; -90] == [0 ; -1610]

def frame():
    cap = cv2.VideoCapture('https://192.168.42.129:8080/video') # use Smartphone webcam through USB Tethering
    ret, frame1 = cap.read()
    ret, frame2 = cap.read()
    for i in range(2000):
        diff = cv2.absdiff(frame1, frame2)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            (x, y, w, h) = cv2.boundingRect(contour)
            if cv2.contourArea(contour) < 3000:
                continue
            cv2.rectangle(frame1, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # We move the robot only in case of detection of big objects 
            if x+(w//2)>350 and x+(w//2)<400 and y+(h//2)>150 and y+(h//2)<200 and threading.activeCount()<7:
                myThread = threading.Thread(target=Move, args=(x,y,100,))
                myThread.start() 

        cv2.imshow("feed", frame1)
        frame1 = frame2
        ret, frame2 = cap.read()

        # Close the stream of the webcam 
        if cv2.waitKey(40) == 27:
            break
        
    cv2.destroyAllWindows()
    cap.release()
    

def CalculAng(x,y,z):
    list= []
    if(x>135 and x<290 and y>-200 and y<200 and z<40 and z>-170 and y<math.sqrt(290**2-x**2)):
        Fteta1 = math.atan(y/x)
        alpha = (x**2 +y**2 + (z + l4 )**2 + l3**2 -l2**2 )/ (2 * l3)
        beta = math.atan((z+l4)/(math.sqrt(x**2+y**2)))
        Fteta2 = math.acos( alpha / (math.sqrt(x**2+y**2+(z+l4)**2))) + beta
        if (z>0):
            Fteta3 = math.acos( (math.sqrt(x**2+y**2)- l3*math.cos(Fteta2) )/l2 )
        else :
            Fteta3 = (-1)*math.acos( (math.sqrt(x**2+y**2)- l3*math.cos(Fteta2) )/l2 )

        list.append(Fteta1*180/3.14)
        list.append(Fteta2*180/3.14)
        list.append(Fteta3*180/3.14)
        return list
    else:
        print("ur position if out of bounds")
        return teta
    

def CalculPos(teta1,teta2,teta3):
    teta1=teta1*3.14/180
    teta2=teta2*3.14/180
    teta3=teta3*3.14/180
    Pos= []
    # Direct geomethreshold
    Pos.append(x)
    Pos.append(y)
    Pos.append(z)
    return Pos
    

def Move(x,y,z):
    global teta
    Flist = CalculAng(x,y,z)
    #angle u need to move by
    for i in range(3):
        Flist[i] =Flist[i] - teta[i]
    #Selecting the Direction of steppers
    if Flist[0] > 0 :
        GPIO.output(sens1,GPIO.HIGH)
    else :
        GPIO.output(sens1,GPIO.LOW)
        Flist[0]=-Flist[0]
    if Flist[1] > 0 :
        GPIO.output(sens2,GPIO.HIGH)
    else :
        GPIO.output(sens2,GPIO.LOW)
        Flist[1]=-Flist[1]
    if Flist[2] > 0 :
        GPIO.output(sens3,GPIO.HIGH)
    else :
        GPIO.output(sens3,GPIO.LOW)
        Flist[2]=-Flist[2]

    #Convert Angle into number of steps
    Flist[0] = int((Flist[0]/90)*1440)
    Flist[1] = int((Flist[1]/75)*1400)
    Flist[2] = int((Flist[2]/90)*1610)
    
    #moving steppers    
    for i in range(max(Flist[0],Flist[1],Flist[2])):
        if Flist[0]>0:
            GPIO.output(pas1,GPIO.HIGH)
            Flist[0]-=1
        if Flist[1]>0:
            GPIO.output(pas2,GPIO.HIGH)
            Flist[1]-=1
        if Flist[2]>0:
            GPIO.output(pas3,GPIO.HIGH)
            Flist[2]-=1
        time.sleep(0.002)
        GPIO.output(pas1,GPIO.LOW)
        GPIO.output(pas2,GPIO.LOW)
        GPIO.output(pas3,GPIO.LOW)
        time.sleep(0.002)
    teta = CalculAng(x,y,z)

# Move the robot to the initial position
def Resetter():
    Move(220,0,-140)
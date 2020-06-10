from controller import DistanceSensor, Motor,Receiver,Emitter,GPS, Supervisor,Keyboard,LED
import random
encoding = 'utf-8'
TIME_STEP = 64
MOVE = 2.0
STOP = 0.0
STUCK_D = 0.004
ENCODE = 'utf-8'
#states
#  1 = car is okay
#  2 = car is on fire
#  3 = car is stuck
#  4 = car is stuck and on fire
OKAY = 1
FIRE = 2
STUCK = 3
REKT = 4

dim = [0.1,0.05,0.2]

state = OKAY

#get supervisor mode for translation randomization
supervisor = Supervisor()
#randomize translation
#random seed initialize
random.seed()
#get robot from supervisor
robo_node = supervisor.getFromDef('Robot_dummy')
#get field from robot
trans_field = robo_node.getField('translation')
#randomize the x and z coordinates to change position of dummy car
x = random.uniform(-2,2)
z = random.uniform(-2,2)
#trans_field.setSFVec3f([x,0,z])

#Enabling keyboard to control different funcitons
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

#initializing distanse sensors
ds = []
dsNames = ['ds_right', 'ds_left','ds_back_left','ds_back_right']
for i in range(4):
    ds.append(supervisor.getDistanceSensor(dsNames[i]))
    ds[i].enable(TIME_STEP)
#initializing wheels
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(supervisor.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
avoidObstacleCounter = 0

#initializing communications
fasheqi=supervisor.getEmitter('emitter')
fasheqi.setChannel(1)
fasheqi.setRange(-1)

#initializing IMU
imu=supervisor.getInertialUnit('imu_angle')
imu.enable(TIME_STEP)
count = 0
#imu stuff
pI=3.14159265
curr_angle = 0


#initialize GPS
gps = supervisor.getGPS('dummy_gps')
gps.enable(TIME_STEP)
prev_data = [0,0,0]

# for getting average distance change
n = 0
avg = 0
cumulative = 0

#LED stuff
ledR = [None] * 5
ledR[0] = supervisor.getLED('fire_1')
ledR[1] = supervisor.getLED('fire_2')
ledR[2] = supervisor.getLED('fire_3')
ledR[3] = supervisor.getLED('fire_4')
ledR[4] = supervisor.getLED('fire_5')
led_on = 0
led_num = random.randint(0,4)



def isStuck(new_data, prev_data):
    xd = (new_data[0] - prev_data[0])
    yd = (new_data[1] - prev_data[1])
    zd = (new_data[2] - prev_data[2])
    if xd < STUCK_D and yd < STUCK_D and zd < STUCK_D:
        return 1
    else:
        return 0
def move(wheels, speed):
    wheels[0].setVelocity(speed)
    wheels[1].setVelocity(speed)
    wheels[2].setVelocity(speed)
    wheels[3].setVelocity(speed)      
def sendStuck(coord, server):
    print('im stuck!')
    coord = "REQ HELP COORDS " + coord + " DIM " + str(curr_angle) + " " + str(dim[0]) + " " + str(dim[2]) + " TOW"
    message = bytes(coord,ENCODE)
    server.send(message)
    
def isFire(led_on):
    #print('here 5')
    if led_on == 1:
        return 1
    return 0

def sendFire(coord, server):
    coord = "REQ HELP COORDS " + coord + " DIM " + str(curr_angle) + " " + str(dim[0]) + " " + str(dim[2]) + " FIRE"
    message = bytes(coord, ENCODE)
    server.send(message)
    
def sendRekt(coord,server):
    coord = "REQ HELP COORDS " + coord + " REKT"
    message = bytes(coord, ENCODE)
    server.send(message)
    
    
connector = supervisor.getConnector("DUMMY_BACK_LATCH")
connector.enablePresence(TIME_STEP)

# main loop
while supervisor.step(TIME_STEP) != -1:

    data = gps.getValues()
    coord = str(data[0]) + " " + str(data[1]) + " " + str(data[2])
    key = keyboard.getKey()
    move(wheels,STOP)
    
    xyz = imu.getRollPitchYaw()
    curr_angle = xyz[2]/pI*180
    #print(curr_angle)
    #convert the curr_angle to accomadate for fire cars
    if curr_angle < 1 and curr_angle > -1:
        curr_angle = 0
    elif curr_angle < -179 and curr_angle > 179:
        curr_angle = 0
    elif curr_angle > 89 and curr_angle < 91:
        curr_angle = 90
    elif curr_angle >-91 and curr_angle < -89:
        curr_angle = 90
    elif curr_angle > 91 and curr_angle < 179:
        curr_angle = curr_angle - 180
    elif curr_angle < -91 and curr_angle > -179:
        curr_angle = curr_angle + 180
    #print(curr_angle)
    #press left to toggle car on fake fire    
    if key == keyboard.LEFT:
        if led_on == 0:
            led_on = 1
            ledR[led_num].set(led_on)
            #led_num = randint(0,4)
        else:
            led_on = 0
            ledR[led_num].set(led_on)
            led_num = random.randint(0,4)
    if key == keyboard.UP:
            move(wheels,MOVE)
    #print(led_on)        
    ########################
    # car state diagnosis  #
    ########################  
    #car is okay
    if state == OKAY:
        # if up is being pressed on keyboard, move car forward       
            #if stuck -> input in wheels but no change in gps
            #press up to go forward
        if key == keyboard.UP:
            #move(wheels,MOVE)
            if isStuck(data,prev_data):
                sendStuck(coord, fasheqi)
                state = STUCK
        elif isFire(led_on):
            print("i'm on fire")
            state = FIRE
            sendFire(coord, fasheqi)
            
            #print(data)
            #get average coordinate change
            #n += 1
            #cumulative += data[2] - prev_data[2]
            #avg = cumulative / n
            #print(avg)
    #car is on fire
    elif state == FIRE:
        #can go to fire and stuck
        if isStuck(data,prev_data):
            #sendStuck(coord,fasheqi)
            state = REKT
            sendStuck(coord, fasheqi)
            print("i'm suffering")
        #can go to okay
        elif not isFire(led_on):
            state = OKAY
            print("i'm okay")
    #car is stuck
    elif state == STUCK:
        #can go to fire and stuck
        if isFire(led_on):
            state = REKT
            sendFire(coord, fasheqi)
            print("i'm suffering")
        #can go to okay
        elif not isStuck(data,prev_data):
            state = OKAY
            print("i'm okay")
    #car is stuck and on fire
    #elif state == REKT:
        #can go to stuck
    #    if not isFire(led_on):
    #        state = STUCK
        #can go to fire
    #    if not isStuck(data,prev_data):
    #        state = FIRE
        
    if key == keyboard.UP:
        prev_data = data
    #print(state)

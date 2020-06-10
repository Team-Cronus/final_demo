from controller import Robot, DistanceSensor, Motor,Receiver,Emitter,Keyboard,Camera,Connector, CameraRecognitionObject, GPS
import struct
import RRT2
import math
import sys
import my_parser
import IdealPos
EN = "utf-8"
pI=3.14159265359
moving = 0


# the dimension of the hook car
#hook_length = 0.2 
#hook_width = 0.1

#car dimensions
# x,y,z in meters 
dim = [0.1,0.05,0.2]
CAR = "TOW"
TH = 950
TIME_STEP = 64

#dummy car location
#goal_x = -1
#goal_y = -1

##########globals for rrt##############################
delta_time=11.93662 #when speed is 3.14 rad/s, go 1.5 meters need 11.93662s
currTime=0
nextTime=0
i=1
mode=2
my_turn = False
lock=0
rrt_count=0
prio = 2
do_job_now = False
##########################################################
######## globals for info parsing and ideal loc ############
com = r_message =  None                                    #
at_ideal = going_ideal = False                             #
coords = []
ccoords =  []
############################################################


robot = Robot()
ds = []
dsNames = ['ds_right', 'ds_left','ds_back_left','ds_back_right']
for i in range(4):
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(TIME_STEP)
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
avoidObstacleCounter = 0
############################################################initiation of controllers and sensors
tuoluoyi=robot.getInertialUnit('imu_angle')
tuoluoyi.enable(TIME_STEP)
jieshouqi=robot.getReceiver('receiver')
jieshouqi.enable(TIME_STEP)
jieshouqi.setChannel(2)

server_sock=robot.getEmitter('emitter')
server_sock.setChannel(1)
server_sock.setRange(-1)

count=0
gps = robot.getGPS('hook_gps')
gps.enable(TIME_STEP)
#for manually controlling the car
CONTROL_MODE = 1
#Enabling keyboard to control different funcitons
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

#initialize camera
cam1 = robot.getCamera("camera1")
cam1.enable(TIME_STEP)
cam1.recognitionEnable(TIME_STEP)
cam1m = robot.getMotor("camera_motor")


cam1m.setPosition(float("inf"))
cam1m.setVelocity(0.0)
connector = robot.getConnector('DUMMY_BACK_LATCH')
connector.enablePresence(TIME_STEP)


################################################ for sequence global variable
start_find = 0 # set to 1 when rrt is finished and is starting to find latch
start_rrt = 0 # set to 1 when hook car received command from server and find path to a nearby location of dummy car

################################################ global variable for move_to_latch()
goal_x = -1 #dummy car location in term of x axis
goal_y = -1 #dummy car location in term of y axis
dum_loc = [] # the 4 location around the dummy car
dum_angle = -361 # the orientation of dum car
dum_loc.append((-100,-100)) #top
dum_loc.append((-100,-100)) #right
dum_loc.append((-100,-100)) #bot
dum_loc.append((-100,-100)) #left
######################################################### locks
#move_to_fire variables
stop_index = -1
index = -1
max_light_index = -1
min_dist_index = -1
first_time = 0
#LOCKS
hook_lock_0 = 0
hook_lock_1 = 0
hook_lock_2 = 0
hook_lock_3 = 0
hook_lock_4 = 0
hook_lock_5 = 0
hook_lock_6 = 0 
hook_lock_7 = 0

move_lock_4_1 = 0
move_lock_4_2 = 0
move_lock_4_3 = 0
move_lock_4_4 = 0
move_lock_4_5 = 0
move_lock_4_6 = 0
move_lock_1 = 0
move_lock_2 = 0
move_lock_3 = 0
move_lock_4 = 0
move_thresh_angle = -361

scan_lock_1 = 0
scan_lock_2 = 0
scan_lock_3 = 0
scan_lock_4 = 0
scan_lock = 0

#convert python coordinates to webots
def conv_coords(coords):
    x = (coords[0] + 3) * 10
    y = (coords[1] + 3) * 10 
    z = (-1*coords[2] + 3) * 10
    return [x,y,z]

############## create rrt path
rrt_theta=[]
len_rrt = 0
my_rrt = None
in_transit = False
obstacles_list = [
         (19,20,2,40),
        (37,0,2,40)
        ]
show_ani = False
#initializing the rrt
def rrt_create(start, dest):
   global my_rrt
   #rrt_theta = []
   beg_x = float(start[0])
   beg_y = float(start[2])
   goal_x=float(dest[0])
   goal_y=float(dest[2])
   #convert radius to RRT coordinates
   car_bound = dim[2] * 10 + 2
   #print(car_radius)
   my_rrt = RRT2.RRT(start=[beg_x, beg_y],goal=[goal_x,goal_y],rand_area=[0,60],\
       obstacle_list=obstacles_list,car_radi=car_bound,show=show_ani)
   #rrt_path=RRT2.rrt_main(beg_x, beg_y,goal_x,goal_y,car_bound)
   rrt_new(start,dest)
   #print(rrt_theta)

#ALWAYS! rrt_reset right before rrt_new
#resetting parameters for rrt
def rrt_reset():
    global rrt_theta, len_rrt, i, lock, angleArray
    rrt_theta = []
    len_rrt = 0
    i = 1
    #print(i)
    angleArray=rrt_theta
    lock = 0

#change start and goal and create path
def rrt_new(start, dest):
    global rrt_theta,len_rrt, mode,my_rrt
    rrt_reset()
    my_rrt.set_start_end([float(start[0]),float(start[2])],[float(dest[0]),float(dest[2])])
    rrt_path = my_rrt.planning() 
    len_path=len(rrt_path)
    for data in rrt_path:
       rrt_theta.append(float(data[2]))
    rrt_theta.reverse()
    len_rrt=len(rrt_theta)
    mode = 1
    print(rrt_theta)

#########################################for move rrt

def go_straight():
        wheels[0].setVelocity(3)
        wheels[1].setVelocity(3)
        wheels[2].setVelocity(3)         
        wheels[3].setVelocity(3)
def go_back():
        wheels[0].setVelocity(-3)
        wheels[1].setVelocity(-3)
        wheels[2].setVelocity(-3)         
        wheels[3].setVelocity(-3)
def right_turn():
         wheels[0].setVelocity(1)
         wheels[1].setVelocity(-1)
         wheels[2].setVelocity(1)         
         wheels[3].setVelocity(-1) 

def left_turn():
         wheels[0].setVelocity(-1)
         wheels[1].setVelocity(1)
         wheels[2].setVelocity(-1)         
         wheels[3].setVelocity(1) 
         
def no_move():  
         wheels[0].setVelocity(0)
         wheels[1].setVelocity(0)
         wheels[2].setVelocity(0)         
         wheels[3].setVelocity(0)  
         
def set_global_angle(angle):
    global dum_angle
    dum_angle = angle


def move_car_on_rrt():
    global mode,lock,i,delta_time,currTime,nextTime,rrt_theta, at_ideal, going_ideal
    angleArray = rrt_theta
    #print(angleArray[i])
    if mode==0:
         #print(lock)
         xyz = tuoluoyi.getRollPitchYaw()
         for a in xyz:
             a = float(a)
         curr_angle = xyz[2]/pI*180
      #decide which way should rotate
         if lock ==0:
            if angleArray[i]<=curr_angle and  curr_angle >=0 and angleArray[i] >=0:    #right turn
                lock=1            
            elif angleArray[i] > curr_angle and  curr_angle >=0 and angleArray[i] >=0:   #left  turn 
                lock=2                   
            elif angleArray[i]<=curr_angle and  curr_angle <=0 and angleArray[i] <=0: #right turn
                lock=3                                  
            elif angleArray[i]>curr_angle and  curr_angle <=0 and angleArray[i] <=0: #left turn
                lock=4                 
            elif angleArray[i]<90 and  curr_angle <=0 and angleArray[i] >=0:  #left turn
                lock=5                
            elif angleArray[i]>=90 and  curr_angle <=0 and angleArray[i] >=0: #right turn
                lock=6
            elif angleArray[i]>-90 and  curr_angle >=0 and angleArray[i] <=0:  #left turn
                lock=7                         
            elif angleArray[i]<=-90 and  curr_angle >=0 and angleArray[i] <=0:   #right turn
                lock=8
            else:
                print('error')
                
            #print(lock)              
         if lock==1:    #right turn
              right_turn()
              xyz=tuoluoyi.getRollPitchYaw()
              if xyz[2]/pI*180 <=angleArray[i]:
                    no_move()
                    mode=1
                    currTime=robot.getTime()              
         elif lock==2:   #left  turn 
              left_turn()
              xyz=tuoluoyi.getRollPitchYaw()
              if xyz[2]/pI*180 >=angleArray[i] or (xyz[2]/pI*180 <=-175 and xyz[2]/pI*180>=-180):
                    no_move()
                    mode=1 
                    currTime=robot.getTime()                    
         elif lock==3: #right turn
              right_turn()
              xyz=tuoluoyi.getRollPitchYaw()
              if xyz[2]/pI*180 <=angleArray[i]:
                    no_move()
                    mode=1
                    currTime=robot.getTime()                                  
         elif lock==4: #left turn
              left_turn()
              xyz=tuoluoyi.getRollPitchYaw()
              if xyz[2]/pI*180 >=angleArray[i]:
                    no_move()
                    mode=1
                    currTime=robot.getTime()                  
         elif lock==5:  #left turn
              left_turn()
              xyz=tuoluoyi.getRollPitchYaw()
              if xyz[2]/pI*180 >=angleArray[i]:
                    no_move()
                    mode=1
                    currTime=robot.getTime()                 
         elif lock==6: #right turn
              right_turn()
              xyz=tuoluoyi.getRollPitchYaw()
              if xyz[2]/pI*180 <=angleArray[i] and xyz[2]/pI*180>=0:
                    no_move()
                    mode=1
                    currTime=robot.getTime()                          
         elif lock==7:  #left turn
              right_turn()
              xyz=tuoluoyi.getRollPitchYaw()
              if xyz[2]/pI*180 <=angleArray[i] and xyz[2]/pI*180<=0:
                    no_move()
                    mode=1
                    currTime=robot.getTime()                          
         elif lock==8:   #right turn
              left_turn()
              xyz=tuoluoyi.getRollPitchYaw()
              if xyz[2]/pI*180 >=angleArray[i] and xyz[2]/pI*180<=0:
                    no_move()
                    mode=1 
                    currTime=robot.getTime()                                 
    
    elif mode==1:                 
       if robot.getTime()-currTime <= 1.55:#3.978733:
              #count+=1
              #print(count,robot.getTime())
              leftSpeed = pI
              rightSpeed = pI
              wheels[0].setVelocity(leftSpeed)
              wheels[1].setVelocity(rightSpeed)
              wheels[2].setVelocity(leftSpeed)         
              wheels[3].setVelocity(rightSpeed)              
       else:
              #print(lock,angleArray[i])
              i+=1
              lock=0 
              no_move()
              
              if going_ideal:
                  #if at the end of RRT, switch modes
                  if i==len_rrt-1:                         
                      mode=3
                      #if heading to ideal, set at_ideal to be true
                      at_ideal = True
                      going_ideal = False
                  else:
                      mode = 0
              elif do_job_now:
                  #print("from do_job_now")
                  if i==len_rrt-1:                         
                      mode = 2
                      #mode=5
                  else:
                      mode = 0
                  
              elif i == len_rrt-2:
                  mode = 3
                  #rrt_theta = []
              else:
                 mode=0
       #moving = 0
    return mode #,lock,i,delta_time,currTime,nextTime
              
#######################
# ideal waiting stuff #
#######################


def check_queue(rec_sock):
    server_sock.send(bytes('REQ JOBS NONE '+CAR,EN))
    return False

#ideal position does not work well with u shaped obstacles since the car will wait
#on the outside of the wall    
def go_ideal():
    global gps, ccoords, going_ideal, mode,my_rrt
    #test ideal_pos
    my_pos = gps.getValues()
    ideal = IdealPos.Ideal_Pos(r1=12,r2=16,show=True)
    my_pos = conv_coords(my_pos)
    #pos is a list of all reported positions (x,z)
    #ind is a sorted index by smallest distance
    pos, ind = ideal.get_ideal(my_pos[0],my_pos[2],ccoords[0],ccoords[2])
    #get coordinates of ideal waiting spot from positions and index
    ind_no_obs = 0
    #while position has an obstacle, update index to next shortest distance
    while my_rrt.collision_check_no_path(pos[ind[ind_no_obs]],my_rrt.obstacleList) == 0:
        ind_no_obs = ind_no_obs + 1
    print("obs index "+str(ind_no_obs))    
    print("position: "+str(pos[ind[ind_no_obs]]))    
            
    
    #change to fit coordinate convention 
    ideal_wait = [pos[ind[ind_no_obs]][0],0,pos[ind[ind_no_obs]][1]]
    rrt_new(my_pos,ideal_wait)
    #mode = 2
    going_ideal = True
    return True
#############################
# collision avoidance stuff #
#############################

##TODO!!!!: finish implementing collision avoidance,
# need to use function to add obstacles
# need update communications implementation

def query_coll():
    data = gps.getValues()
    coord = str(data[0]) + " " + str(data[1]) + " " + str(data[2])
    out_message = bytes("REQ COLL COORDS "+ str(coords[0]) +" "+ str(coords[1]) +" "+ str(coords[2]) + " "+CAR,EN)
    server_sock.send(out_message)
    
def resp_coll():
    data = gps.getValues()
    coord = str(data[0]) + " " + str(data[1]) + " " + str(data[2])
    out_message = bytes("RESP COLL COORDS "+ coords[0] +" "+ coords[1] +" "+ coords[2] + " "+CAR,EN)
    server_sock.send(out_message)


def handle_collision():
    return False

def send_fin_task():
    data = "RESP HELP DONE "+CAR
    out_message = bytes(data,EN)
    server_sock.send(out_message)

def add_obstacle(ox,oz,size):
    global my_rrt
    obs_x = ox-0.5*size
    obs_z = oz-0.5*size
    my_rrt.add_obstacle([obs_x,obs_z],size)

def add_obstacles(obs_list):
    for (ox,oz,size) in obs_list:
        add_obstacle(ox,oz,size)    
    
def distance(pos1, pos2):
    return math.sqrt((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)

def move_straight(pos2, index):
    data = gps.getValues()
    angle = -1000
    if index == 1:  
        if data[0] < pos2[0]:
            go_straight()
            data = gps.getValues()
        else:
            no_move()
            angle = 180
    elif index == 2:
        if data[2] > pos2[1]:
            go_straight()
            data = gps.getValues()
        else:
            no_move()
            angle = -90
    elif index == 3:
        if data[0] > pos2[0]:
            go_straight()
            data = gps.getValues()
        else:
            no_move()
            angle = 0
    elif index == 0:
        if data[2] < pos2[1]:
            print(data[2])
            print(pos2[1])
            go_straight()
            data = gps.getValues()
            print("here")
        else:
            no_move()
            angle = 90
    return angle
 

def move(pos1, pos2, index):
    global move_lock_4_1, move_lock_4_2, move_lock_4_3, move_lock_4_4, move_lock_4_5, move_lock_4_6, move_lock_1, move_lock_2, move_lock_3, move_lock_4
    global move_thresh_angle
    move_return = -1
    
    if index == 4:
        #print("here?")       
        xyz = tuoluoyi.getRollPitchYaw()
        curr_angle = xyz[2]/pI*180
        # get thresh angle, the angle to stop turning
        if pos1[0] < pos2[0]:
            thresh_angle = 90
        elif pos1[0] > pos2[0]:
            thresh_angle = -90
        # turn to that angle
        if move_lock_4_1 == 0:
            left_turn()
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == thresh_angle:
                no_move()
                move_lock_4_1 = 1
                print("here!!!!!!!!!!!")    
        elif move_lock_4_1 == 1 and move_lock_4_2 == 0:
            data = gps.getValues()
            if pos1[0] > pos2[0] and data[0] > pos2[0]:
                go_straight()
                data = gps.getValues()
                #print(str(data[0]))
                #print(str(pos2[0]))
            elif pos1[0] < pos2[0] and data[0] < pos2[0]:
                go_straight()
                data = gps.getValues()
            else:
                move_lock_4_2 = 1
                no_move()
                print("finish moving left or right")
        elif move_lock_4_2 == 1:
            data = gps.getValues()
            if pos2[1] > data[2] and pos2[1] > pos1[1]:
                #thresh_angle = 180
                if move_lock_4_3==0:
                    left_turn()
                    xyz=tuoluoyi.getRollPitchYaw()
                    curr_angle = xyz[2]/pI*180
                    print(curr_angle)
                    if  -1 < round(curr_angle) < 1:
                        print("here")
                        move_lock_4_3 = 1
                        no_move()
                elif move_lock_4_5 == 0:
                    go_straight()
                    data = gps.getValues()
                    #print(str(data[2]))
                    #print(str(pos2[1]))
                    #if data[2] >= pos2[1]:
                        #print("here!!!!")
                        #move_lock_4_5 = 1
                        #move_return = 1
                        #no_move()
            elif pos2[1] < data[2] and pos2[1] < pos1[1]:
                #thresh_angle = 0
                print("hello")
                if move_lock_4_4==0:
                    right_turn()
                    xyz=tuoluoyi.getRollPitchYaw()
                    curr_angle = xyz[2]/pI*180
                    if abs(curr_angle) > 179:
                        move_lock_4_4 = 1
                        no_move()
                elif move_lock_4_6 == 0:
                    print("hello2")
                    go_straight()
                    data = gps.getValues()
                    #if data[2] < pos2[1]:
                        #move_return = 1
                        #no_move()
            else:
                 print("I am fking here!")
                 no_move()
                 move_return = 1
    else:
        xyz=tuoluoyi.getRollPitchYaw()
        curr_angle = xyz[2]/pI*180
        if move_lock_1 == 0:
            print("at lock 1")                     
            move_thresh_angle = move_straight(pos2, index)
            if move_thresh_angle != -1000:
                print(move_thresh_angle)
                move_lock_1 = 1
        elif curr_angle < (move_thresh_angle-2)  or curr_angle > (move_thresh_angle +2) and move_lock_2 == 0:
             print("at lock 2")
             left_turn()
             xyz=tuoluoyi.getRollPitchYaw()
             curr_angle = xyz[2]/pI*180
             if curr_angle > (move_thresh_angle-2) and curr_angle < (move_thresh_angle + 2):
                 move_lock_2 = 1
                 no_move()
        elif move_lock_3 == 0:
            print("at lock 3")
            index = (index+1)%4
            move_lock_3 = 1
        elif move_lock_4 == 0:
            print("at lock 4")
            print(index)
            print(pos2)
            index_temp = (index+1)%4
            temp = move_straight(pos2, index_temp)
            if temp != -1000:
                move_lock_4 = 1
                move_return = 1
        else:
            move_return = 1        
    if move_return == 1:
        no_move()
        move_lock_4_1 = 0
        move_lock_4_2 = 0
        move_lock_4_3 = 0
        move_lock_4_4 = 0
        move_lock_1 = 0
        move_lock_2 = 0
        move_lock_3 = 0
        move_lock_4 = 0
        move_thresh_angle = -361
        return 1
    return move_return

def find_hook(index):
    global scan_lock, scan_lock_1, scan_lock_2, scan_lock_3, scan_lock_4
    global dum_angle
    ret = -1
    xyz = tuoluoyi.getRollPitchYaw()
    curr_angle = xyz[2]/pI*180 
    if index == 1 or index == 3: #thresh angle for scanning right and left side
        thresh_angle_1 = -180+dum_angle
        if thresh_angle_1 == -180:
            thresh_angle_1 += 1
        if thresh_angle_1 < -180:
            thresh_angle_1 = 360 + thresh_angle_1
        thresh_angle_2 = 0 + dum_angle
        if thresh_angle_2 == 0:
            thresh_angle_2 += 1
    elif index == 0 or index == 2: #thresh angle for scanning top and bot side
        thresh_angle_1 = 90+dum_angle
        thresh_angle_2 = -90+dum_angle
    if index == 1 and scan_lock == 0:    #scan right
        if scan_lock_1 == 0:
            left_turn()
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(dum_angle-90):
                scan_lock_1 = 1
                no_move()
        if scan_lock_1 == 1:
            print("scan lock 1!")
            for object in cam1.getRecognitionObjects():
                #print(object.get_model())
                if object.get_model().decode(EN) == 'TOW':
                    print("I SEE IT")
                    location = object.get_position()
                    scan_lock_1 = 0
                    no_move()
                    return 1
            right_turn()
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_1):
                print("finished rotating!")
                scan_lock_1 = 0
                scan_lock = 1
                no_move()
    elif index == 2 and scan_lock == 0:    #scan bot (which is shown top)
        if scan_lock_2 == 0:
            left_turn()
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            print(curr_angle)
            if dum_angle == 0:
                dum_angle = dum_angle + 1
            if round(curr_angle) == int(dum_angle):
                scan_lock_2 = 1
                no_move()   
        if scan_lock_2 == 1:
            print("scan lock 2!")
            for object in cam1.getRecognitionObjects():
                #print(object.get_model())
                if object.get_model().decode(EN) == 'TOW':
                    print("I SEE IT")
                    location = object.get_position()
                    scan_lock_2 = 0
                    no_move()
                    return 1
            right_turn()
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_2):
                print("finished rotating!")
                scan_lock_2 = 0
                scan_lock = 1
                no_move()
    elif index == 3 and scan_lock == 0:  # scan left
        if scan_lock_3 == 0:
            left_turn()
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(dum_angle+90):
                scan_lock_3 = 1
                no_move()
        if scan_lock_3 == 1:
            print("scan lock 3!")
            for object in cam1.getRecognitionObjects():
                #print(object.get_model())
                if object.get_model().decode(EN) == 'TOW':
                    print("I SEE IT")
                    location = object.get_position()
                    scan_lock_3 = 0
                    no_move()
                    return 1
            right_turn()
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_2):
                print("finished rotating!")
                scan_lock_3 = 0
                scan_lock = 1
                no_move()
    elif index == 0 and scan_lock == 0:  #scan top (which is shown bot)
        if scan_lock_4 == 0:
            left_turn()
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            t_angle = -179
            if dum_angle < 0:
                t_angle = dum_angle + 180
            elif dum_angle > 0:
                t_angle = dum_angle - 180
            print("t_angle is: " + str(t_angle))
            print("curr_angle is: " + str(curr_angle))
            if round(curr_angle) == int(t_angle):
                scan_lock_4 = 1
                no_move()
        elif scan_lock_4 == 1:
            print("scan lock 4!")
            for object in cam1.getRecognitionObjects():
                #print(object.get_model())
                if object.get_model().decode(EN) == 'TOW':
                    print("I SEE IT")
                    location = object.get_position()
                    scan_lock_4 = 0
                    no_move()
                    return 1
            right_turn()
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_1):
                print("finished rotating!")
                scan_lock_4 = 0
                scan_lock = 1
                no_move()
    if scan_lock == 1:
        print("scan_lock is set to 1")
        scan_lock = 0
        return 2
    return ret

def get_to_operation():
    data2 = connector.getPresence()
    if data2==1:
        no_move()
        connector.lock()
        return 1
    else:
        for object in cam1.getRecognitionObjects():
            #print(object.get_model())
            if object.get_model().decode(EN) == 'TOW':
                print("I SEE IT")
                location = object.get_position()
                print("The latch location is : "+ str(location))
                if location[0] < 0.03 and location[0] > -0.03:
                    go_straight()
                elif location[0] > -0.03:
                    print("turning right")
                    right_turn()
                    go_back()
                    if robot.step(100) == -1:
                         pass
                    right_turn()
                elif location[0] < 0.03:
                    print("turning left")
                    left_turn()
                    go_back()
                    if robot.step(100) == -1:
                         pass
                    left_turn()
                return -1
            else:
                print("seeing nothing, turning right")
                right_turn()
                return -1
        return -1          
def move_to_latch():
    global hook_lock_0, hook_lock_1, hook_lock_2, hook_lock_3, hook_lock_4, hook_lock_5, hook_lock_6, hook_lock_7, hook_lock_8, hook_lock_9
    global arm_loc, stop_index, index, min_dist_index
    global first_time
    ret = -1
    #data = gps.getValues()
    if hook_lock_0 == 0:
        #print("at hook lock 0")
        data = gps.getValues()
        arm_loc = (data[0], data[2])
        #finding closest point of interest
        dist = []
        #print("here?")
        dist.append(distance(arm_loc, dum_loc[0])) #top
        dist.append(distance(arm_loc, dum_loc[1])) #right
        dist.append(distance(arm_loc, dum_loc[2])) #bot
        dist.append(distance(arm_loc, dum_loc[3])) #left
        min_dist_index = dist.index(min(dist[0], dist[1], dist[2], dist[3]))
        print("The min dist index is: " + str(min_dist_index))
        hook_lock_0 = 1
    #move to point of interest
    elif hook_lock_1 == 0:
        #print("at hook lock 1")
        #print("Now the min dist index is: " + str(min_dist_index))
        temp = move(arm_loc, dum_loc[min_dist_index], 4)
        #print(str(dum_loc[min_dist_index]))
        #print(str(arm_loc))
        if temp == 1:
            hook_lock_1 = 1
            print("moved to point of interest")
    
    #scan the area
    elif hook_lock_1 == 1 and hook_lock_2 == 0:
        #print("at hook lock 2")
        temp = find_hook(min_dist_index)
        if temp == 1:
            print("found hook!")
            hook_lock_2 = 1
            hook_lock_3 = 1
            hook_lock_6 = 1
        elif temp == 2:
            index = (min_dist_index+1)%4
            hook_lock_2 = 1
    #move to the other sides
    elif hook_lock_2 == 1 and hook_lock_3 == 0: 
        #print("at hook lock 3")
        if index != min_dist_index:
            #print("try to move to other side")
            data = gps.getValues()
            arm_loc = (data[0], data[2])
            #if dum_loc[index] != (-100,-100):
            if hook_lock_4 == 0:
                #print("At 4")
                temp1 = move(arm_loc, dum_loc[index], index)
                if temp1 == 1:
                    hook_lock_4 = 1
            elif hook_lock_4 == 1 and hook_lock_5 ==0:
                #print("At 5")
                temp2 = find_hook(index)
                if temp2 == 1:
                    hook_lock_5 = 1
                    hook_lock_3 = 1
                    hook_lock_6 = 1
                elif temp2 == 2:
                    hook_lock_5 = 1
            #index = (index+1)%4
            elif hook_lock_5 == 1:
                #print("At 6")
                index = (index+1)%4
                #print("index is:" + str(index))
                #print("min_dist_index" + str(min_dist_index))
                hook_lock_4 = 0
                hook_lock_5 = 0
                if index == min_dist_index:
                    hook_lock_3 = 1
                    #max_light_index = dum_light.index(max(dum_light[0],dum_light[1],dum_light[2],dum_light[3]))
                    #stop_index = min_dist_index-1
                    #if stop_index < 0:
                        #stop_index = 3

    #rotate to the final angle        
    elif hook_lock_6 == 1:
        #print("at hook lock 6")
        #print("here!!!!")
        temp = get_to_operation()
        if temp == 1:
            hook_lock_6 = 0
            hook_lock_7 = 1            
    elif hook_lock_7 == 1:
        hook_lock_1 = 0
        hook_lock_2 = 0
        hook_lock_3 = 0
        hook_lock_4 = 0
        hook_lock_5 = 0
        hook_lock_6 = 0
        hook_lock_7 = 0
        ret = 1
    return ret
        
    #get_to_operation(dum_imu[max_light_index]) #rotate to the angle and move closer
    #print("top value is "+ str(top_light_value))
    #print("bot value is "+ str(bot_light_value))
    
while robot.step(TIME_STEP) != -1:
   
    data2 = connector.getPresence()
    
    if data2==1:
        connector.lock()
      
    if jieshouqi.getQueueLength() > 0:
        if jieshouqi.getDataSize() > 0:
            message=jieshouqi.getData()
            jieshouqi.nextPacket()
            #parse message
            r_message = message.decode(EN)
            #statement to go to a position via RRT
            if "GO" in r_message:
                #rrt_reset()
                print(r_message)
                coords, dinfo = my_parser.coords(r_message)
                start = gps.getValues()
                
                #for a in start:
                #    a = float(a)
                #print(coords)
                for a in coords:
                    ccoords.append(float(a))
                start = conv_coords(start)
                ccoords = conv_coords(ccoords)
                mode = 0
                
                #TODO: set up actual task
                angle = math.radians(float(dinfo[0])) # imu of dum car                
                length = float(dinfo[2]) # length of dum car
                width = float(dinfo[1]) # width of dum car
                set_global_angle(float(dinfo[0]))
                #set the four location to traverse.
                dum_loc[0] = (float(coords[0]) + (length+dim[2]/2)*math.sin(angle),float(coords[2])+(length+dim[2]/2)*math.cos(angle)) #top
                dum_loc[1] = (float(coords[0])+ (width+dim[2])*math.cos(angle),float(coords[2])-(width+dim[2])*math.sin(angle)) #right
                dum_loc[2] = (float(coords[0]) - (length+dim[2]/2)*math.sin(angle),float(coords[2])-(length+dim[2]/2)*math.cos(angle)) #bot
                dum_loc[3] = (float(coords[0]) - (width+dim[2])*math.cos(angle),float(coords[2])+(width+dim[2])*math.sin(angle)) #left
                
                

                if my_rrt is None:
                    rrt_create(start,ccoords)
                else:
                    rrt_new(start,ccoords)
                in_transit = True
                #mode = 10 #for testing purposes!!!!
            elif "RESP" in r_message:
                #statement to check message whose turn it is for going to ideal position
                if "JOBS" in r_message:
                    #its FIRE car's turn to go, go into specific task mode
                    if (CAR+"T") in r_message:
                        print("my turn!")
                        my_turn = True
                        mode = 5
                    else:
                        print(r_message)
                        print("not my turn")
                        #go to ideal position mode
                        mode = 4
                #statement to check if its their turn to go for collision protocol
                elif "COLL" in r_message:
                    print(r_message)
                    #TODO: implement add obstacles,implement go to node function
                    #add obstacles to rrt planner list
                    #find new node to traverse to
                    #rewire rrt to go to that node
                    #send message back to server for confirmation
                    #continue on rrt
                    
                    
            #condition to respond to collision protocol steo for server's query        
            elif "REQ" in r_message:
                if "COLL" in r_message:
                    resp_coll()
    #RRT is still processing         
    elif 0 <= mode and mode < 2: 
        print(mode)
        #check if car senses another car
        #if it does, go into collision avoidance mode
        #else, move car on rrt
        #if in_transit:
        #    for sensor in ds:
        #        if sensor.getValue() < 300:
        #            mode = 6
        if 0 <= mode and mode < 2: 
            move_car_on_rrt()
        
    #RRT is completed and check to go to ideal
    elif mode == 3:
        in_transit = False
        #rrt_reset()
        #if not at ideal position, then check queue
        check_queue(jieshouqi)
        print(mode)              
    #if needs to go to ideal position    
    elif mode == 4:
        in_transit = False
        print(mode)
        #not car's turn and not at ideal, then go to ideal
        if not my_turn and not at_ideal:
            print("GOING IDEAL")
            go_ideal()
            #at_ideal = True    
        #if not car's turn and is at ideal, then stay still and check queue
        elif not my_turn and not going_ideal:
            no_move()
            mode = 3
            #testing purpose only
            #my_turn = True
        #it is this car's turn, go back in front of car, then mode 5 which does its job 
        else: 
            mode = 0
            my_pos = gps.getValues()
            my_pos = conv_coords(my_pos)
            do_job_now = True
            rrt_new(my_pos,ccoords)        
            at_ideal = False      
    #do specific task    
    elif mode == 5:
        #no_move()
        #print(mode)
        
        print("finding latch...")
        #should prepare to operate on car
        #then do its specific task
        #from there, send fin
        task_done = False
        temp = move_to_latch()
        if data2==1:
            #start_find = 0
            no_move()
            task_done = True
            print("done!!!!")   
        if temp == 1:
            #start_find = 0
            no_move()
            task_done = True
            print("done!!!!") 
        
        if task_done == True:
            send_fin_task()
            mode = 2
    #collision avoidance protocol
    elif mode == 6:
        print(mode)
        no_move()
        #request server for obstacles
        query_coll()
        
        #mode = 7  
    elif mode == 7:
        print(mode)
        #wait until further notice to run new RRT
        #mode = 6
        
    #test mode
    elif mode == 10:
        
        go_ideal()
        print(mode)
             
     #'''     
     #s = 1
     #z = -1
     
     #if CONTROL_MODE == 1:
     #    key = keyboard.getKey()
     #    if key == keyboard.UP:               #move backwards
     #        wheels[0].setVelocity(z*leftSpeed)
     #        wheels[1].setVelocity(z*rightSpeed)
     #        wheels[2].setVelocity(z*leftSpeed)         
     #        wheels[3].setVelocity(z*rightSpeed)
     #    elif key == keyboard.DOWN:           #move forwards
     #        wheels[0].setVelocity(s*leftSpeed)
     #        wheels[1].setVelocity(s*rightSpeed)
     #        wheels[2].setVelocity(s*leftSpeed)         
     #        wheels[3].setVelocity(s*rightSpeed)
     #    elif key == keyboard.LEFT:                #turn left
     #        wheels[0].setVelocity(z*leftSpeed)
     #        wheels[1].setVelocity(s*rightSpeed)
     #        wheels[2].setVelocity(z*leftSpeed)         
     #        wheels[3].setVelocity(s*rightSpeed)
     #    elif key == keyboard.RIGHT:               #turn right
     #        wheels[0].setVelocity(s*leftSpeed)
     #        wheels[1].setVelocity(z*rightSpeed)
     #        wheels[2].setVelocity(s*leftSpeed)         
     #        wheels[3].setVelocity(z*rightSpeed)
     #    else:                                    #stop
     #        wheels[0].setVelocity(0)
     #        wheels[1].setVelocity(0)
     #        wheels[2].setVelocity(0)
     #        wheels[3].setVelocity(0)
    
     #    '''
     
     
#     data2 = connector.getPresence()
    
#     if data2==1:
#        connector.lock()
    

#     if start_find == 1:
#         temp = move_to_latch()
#         if data2==1:
#             start_find = 0
#             no_move()
#             print("done!!!!")   
#         if temp == 1:
#             start_find = 0
#             no_move()
#             print("done!!!!") 

#define goal location.
#             goal_x = float(coord[1])*10+30
#             goal_y = 60-(float(coord[5])*10+30)

#             
#             start_rrt=1
     #print(data2)
#     if start_rrt == 1:
#         temp = move_car_on_rrt()
         #print(temp[0])
#         print(temp[0])
#         if(temp[0] == 2):
#             start_rrt = 0
#             start_find = 1

                #old notation
                #angle = math.radians(float(coord[7])) # imu of dum car
#               length = float(coord[9]) # length of dum car
#               set_global_angle(float(coord[7])) # set global variable dum_angle for future use
#               width = float(coord[11]) # width of dum car
#               dum_loc[0] = (float(coord[1]) + (length+hook_length/2)*math.sin(angle),float(coord[5])+(length+hook_length/2)*math.cos(angle)) #top
#               dum_loc[1] = (float(coord[1])+ (width+hook_length)*math.cos(angle),float(coord[5])-(width+hook_length)*math.sin(angle)) #right
#               dum_loc[2] = (float(coord[1]) - (length+hook_length/2)*math.sin(angle),float(coord[5])-(length+hook_length/2)*math.cos(angle)) #bot
#               dum_loc[3] = (float(coord[1]) - (width+hook_length)*math.cos(angle),float(coord[5])+(width+hook_length)*math.sin(angle)) #left
#               print(str(dum_loc[0]))
#               print(str(dum_loc[1]))
#               print(str(dum_loc[2]))
#               print(str(dum_loc[3]))
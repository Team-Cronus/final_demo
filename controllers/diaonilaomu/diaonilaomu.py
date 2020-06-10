from controller import Robot, DistanceSensor, Motor,Receiver,Emitter,LightSensor,LED,Keyboard, GPS
import struct
#import RRT
import math
import RRT2
import sys
import my_parser
import IdealPos
EN = "utf-8"
pI=3.14159265359

#dummy car location
#goal_x = -1
#goal_y = -1

#fire car dimension
#fire_length = 0.2
#fire_width = 0.1

#car dimensions
# x,y,z in meters 
dim = [0.1,0.05,0.2]
CAR = "FIRE"
TH = 950
TIME_STEP = 64
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
task_done = False
#testing purposes
mode = 2


##########################################################
######## globals for info parsing and ideal loc ############
com = r_message =  None                  #
at_ideal = going_ideal = False  
coords = []
ccoords =  []                         #
############################################################

robot = Robot()

ds = []
dsNames = ['ds_right', 'ds_left','ds_back_left','ds_back_right']
for i in range(4):
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(TIME_STEP)
'''
ds_l = robot.getDistanceSensor('ds_left')
ds_r = robot.getDistanceSensor('ds_right')
ds_l.enable(TIME_STEP)
ds_r.enable(TIME_STEP)
'''
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4','arm_motor1','arm_motor2','arm_motor4','arm_motor6']
for i in range(8):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
avoidObstacleCounter = 0

tuoluoyi=robot.getInertialUnit('imu_angle')
tuoluoyi.enable(TIME_STEP)
jieshouqi=robot.getReceiver('receiver')
jieshouqi.enable(TIME_STEP)
jieshouqi.setChannel(3)
#set up emitter to send back to server
server_sock=robot.getEmitter('emitter')
server_sock.setChannel(1)
server_sock.setRange(-1)

count =0
jianpan=robot.getKeyboard()
jianpan.enable(TIME_STEP)
light_sensor_1=robot.getLightSensor('light_sensor1')
light_sensor_1.enable(TIME_STEP)
light_sensor_2=robot.getLightSensor('light_sensor2')
light_sensor_2.enable(TIME_STEP)
light_sensor_3=robot.getLightSensor('light_sensor3')
light_sensor_3.enable(TIME_STEP)
light_sensor_lu_left=robot.getLightSensor('light_sensor_lu_left')
light_sensor_lu_left.enable(TIME_STEP)
light_sensor_lu_right=robot.getLightSensor('light_sensor_lu_right')
light_sensor_lu_right.enable(TIME_STEP)
motor4_imu=robot.getInertialUnit('motor4_imu')
motor4_imu.enable(TIME_STEP)
motor2_imu=robot.getInertialUnit('motor2_imu')
motor2_imu.enable(TIME_STEP)
gps = robot.getGPS('arm_gps')
gps.enable(TIME_STEP)
#start_rrt = 0
#start_find = 0
#start_arm = 0
ls_top = robot.getLightSensor("light_sensor_top")
ls_bot = robot.getLightSensor("light_sensor_bot")
ls_top.enable(TIME_STEP)
ls_bot.enable(TIME_STEP)


#to go to the closest side global variables
#detect fire
#Top, Bot, Left, and Right position of the dummy car
dum_loc = []
arm_loc = []
dum_angle = -361
dum_loc.append((-100,-100)) #top
dum_loc.append((-100,-100)) #right
dum_loc.append((-100,-100)) #bot
dum_loc.append((-100,-100)) #left

dum_light =[-1, -1, -1, -1] #top, right, bot, left max light intensity

dum_imu = [-1, -1, -1, -1] #top, right, bot, left angle with the highest intensity

#move_to_fire variables
stop_index = -1
index = -1
max_light_index = -1
min_dist_index = -1
first_time = 0
#LOCKS
fire_lock_0 = 0
fire_lock_1 = 0
fire_lock_2 = 0
fire_lock_3 = 0
fire_lock_4 = 0
fire_lock_5 = 0
fire_lock_6 = 0 
fire_lock_7 = 0
fire_lock_8 = 0
fire_lock_9 = 0

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
ending = False
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
           obstacle_list=obstacles_list,car_radi=car_bound,show = show_ani)
   #rrt_path=RRT2.rrt_main(beg_x, beg_y,goal_x,goal_y,car_bound)
   rrt_new(start,dest)
   #print(rrt_theta)

#ALWAYS! rrt_reset right before rrt_new
#resetting parameters for rrt
def rrt_reset():
    global rrt_theta, len_rrt, i, lock
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
################################################# for move RRT

def go_straight():
        wheels[0].setVelocity(1)
        wheels[1].setVelocity(1)
        wheels[2].setVelocity(1)         
        wheels[3].setVelocity(1)
        
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
       if robot.getTime()-currTime <=1.55:#3.978733:
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
              elif ending:
                  if i==len_rrt-1:                         
                      mode = 2
                      #mode=5
                  else:
                      mode = 0
              elif do_job_now:
                  #print("from do_job_now")
                  if i==len_rrt-1:                         
                      mode = 5
                      #mode=5
                  else:
                      mode = 0
              elif i == len_rrt-4:
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
    ideal = IdealPos.Ideal_Pos(r1=15,r2=20,show=True)
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
    out_message = bytes("RESP COLL COORDS "+ str(coords[0]) +" "+ str(coords[1]) +" "+ str(coords[2]) + " "+CAR,EN)
    server_sock.send(out_message)


def handle_collision():
    return False

def send_fin_task():
    data = "RESP HELP DONE "+CAR
    out_message = bytes(data,EN)
    server_sock.send(out_message)

def add_obstacle(c,size):
    global my_rrt
    obs_x = c[0]-0.5*size
    obs_z = c[2]-0.5*size
    my_rrt.add_obstacle([obs_x,obs_z],size)

def add_obstacles(obs_list):
    for (ox,oz,size) in obs_list:
        add_obstacle(ox,oz,size)    
    
    
###############################################fire car move
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
                #print("hello")
                if move_lock_4_4==0:
                    right_turn()
                    xyz=tuoluoyi.getRollPitchYaw()
                    curr_angle = xyz[2]/pI*180
                    if abs(curr_angle) > 179:
                        move_lock_4_4 = 1
                        no_move()
                elif data[2] > pos2[1] and move_lock_4_6 == 1:
                    go_straight()
                    data = gps.getValues()
                    if data[2] < pos2[1]:
                        move_return = 1
                        no_move()
            else:
                 print("I am fking here!")
                 no_move()
                 move_return = 1
    else:
        xyz=tuoluoyi.getRollPitchYaw()
        curr_angle = xyz[2]/pI*180
        if move_lock_1 == 0:
            print("at lock 1")
            ds_l = ds[0].getValue()
            ds_r = ds[1].getValue()
            if ds_l < 100 or ds_r < 100:
                no_move()
                #move_thresh_angle = t_angle[index]
            else:                     
                move_thresh_angle = move_straight(pos2, index)
            if move_thresh_angle != -1000:
                print(move_thresh_angle)
                move_lock_1 = 1
        elif curr_angle < (move_thresh_angle-0.7)  or curr_angle > (move_thresh_angle +0.7) and move_lock_2 == 0:
             print("at lock 2")
             left_turn()
             xyz=tuoluoyi.getRollPitchYaw()
             curr_angle = xyz[2]/pI*180
             if curr_angle > (move_thresh_angle-0.7) and curr_angle < (move_thresh_angle +0.7):
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

def scan(index):
    global scan_lock, scan_lock_1, scan_lock_2, scan_lock_3, scan_lock_4
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
            lt= (ls_top.getValue() + ls_bot.getValue())/2
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_2):
                scan_lock_1 = 1
                no_move()
            if lt > dum_light[index]:
                dum_light[index] = lt
                dum_imu[index] = curr_angle
        if scan_lock_1 == 1:
            right_turn()
            lt= (ls_top.getValue() + ls_bot.getValue())/2
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_1):
                print("finished rotating!")
                scan_lock_1 = 0
                scan_lock = 1
                no_move()
            if lt > dum_light[index]:
                dum_light[index] = lt
                dum_imu[index] = curr_angle
    elif index == 2 and scan_lock == 0:    #scan bot (which is shown top)
        if scan_lock_2 == 0:
            left_turn()
            lt= (ls_top.getValue() + ls_bot.getValue())/2
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_1):
                scan_lock_2 = 1
                no_move()
            if lt > dum_light[index]:
                dum_light[index] = lt
                dum_imu[index] = curr_angle   
        if scan_lock_2 == 1:
            right_turn()
            lt= (ls_top.getValue() + ls_bot.getValue())/2
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_2):
                print("finished rotating!")
                scan_lock_2 = 0
                scan_lock = 1
                no_move()
            if lt > dum_light[index]:
                dum_light[index] = lt
                dum_imu[index] = curr_angle
    elif index == 3 and scan_lock == 0:  # scan left
        if scan_lock_3 == 0:
            left_turn()
            lt= (ls_top.getValue() + ls_bot.getValue())/2
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_1):
                scan_lock_3 = 1
                no_move()
            if lt > dum_light[index]:
                dum_light[index] = lt
                dum_imu[index] = curr_angle
        if scan_lock_3 == 1:
            right_turn()
            lt= (ls_top.getValue() + ls_bot.getValue())/2
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_2):
                print("finished rotating!")
                scan_lock_3 = 0
                scan_lock = 1
                no_move()
            if lt > dum_light[index]:
                dum_light[index] = lt
                dum_imu[index] = curr_angle 
    elif index == 0 and scan_lock == 0:  #scan top (which is shown bot)
        if scan_lock_4 == 0:
            left_turn()
            lt= (ls_top.getValue() + ls_bot.getValue())/2
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_2):
                scan_lock_4 = 1
                no_move()
            if lt > dum_light[index]:
                dum_light[index] = lt
                dum_imu[index] = curr_angle
        elif scan_lock_4 == 1:
            right_turn()
            lt= (ls_top.getValue() + ls_bot.getValue())/2
            xyz=tuoluoyi.getRollPitchYaw()
            curr_angle = xyz[2]/pI*180
            if round(curr_angle) == int(thresh_angle_1):
                print("finished rotating!")
                scan_lock_4 = 0
                scan_lock = 1
                no_move()
            if lt > dum_light[index]:
                dum_light[index] = lt
                dum_imu[index] = curr_angle
    if scan_lock == 1:
        print("scan_lock is set to 1")
        scan_lock = 0
        return 1
    return ret

def get_to_operation(angle):
    left_turn()
    xyz=tuoluoyi.getRollPitchYaw()
    curr_angle = xyz[2]/pI*180
    if round(curr_angle) == int(angle):
        no_move()
        return 1
    else:
        return -1
        
def move_to_fire():
    global fire_lock_0, fire_lock_1, fire_lock_2, fire_lock_3, fire_lock_4, fire_lock_5, fire_lock_6, fire_lock_7, fire_lock_8, fire_lock_9
    global arm_loc, stop_index, index, min_dist_index, max_light_index
    global first_time
    ret = -1
    #data = gps.getValues()
    if fire_lock_0 == 0:
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
        fire_lock_0 = 1
    #move to point of interest
    elif fire_lock_1 == 0:
        #print("Now the min dist index is: " + str(min_dist_index))
        temp = move(arm_loc, dum_loc[min_dist_index], 4)
        #print(str(dum_loc[min_dist_index]))
        #print(str(arm_loc))
        if temp == 1:
            fire_lock_1 = 1
            print("moved to point of interest")
    
    #scan the area
    elif fire_lock_1 == 1 and fire_lock_2 == 0:
        temp = scan(min_dist_index)
        if temp == 1:
            print("finished scanning")
            fire_lock_2 = 1
            index = (min_dist_index+1)%4
    #move to the other sides
    elif fire_lock_2 == 1 and fire_lock_3 == 0: 
        if index != min_dist_index:
            #print("try to move to other side")
            data = gps.getValues()
            arm_loc = (data[0], data[2])
            #if dum_loc[index] != (-100,-100):
            if fire_lock_4 == 0:
                print("At 4")
                temp1 = move(arm_loc, dum_loc[index], index)
                if temp1 == 1:
                    fire_lock_4 = 1
            elif fire_lock_4 == 1 and fire_lock_5 ==0:
                print("At 5")
                temp2 = scan(index)
                if temp2 == 1:
                    fire_lock_5 = 1
            #index = (index+1)%4
            elif fire_lock_5 == 1:
                print("At 6")
                index = (index+1)%4
                print("index is:" + str(index))
                print("min_dist_index" + str(min_dist_index))
                fire_lock_4 = 0
                fire_lock_5 = 0
                if index == min_dist_index:
                    fire_lock_3 = 1
                    max_light_index = dum_light.index(max(dum_light[0],dum_light[1],dum_light[2],dum_light[3]))
                    stop_index = min_dist_index-1
                    if stop_index < 0:
                        stop_index = 3
    
    #find which side to go
    elif fire_lock_3 == 1 and fire_lock_6 == 0:
        print("max_light_index is:" + str(max_light_index))
        print("current index is: " + str(stop_index))     
        if stop_index != (max_light_index+1)%4 or first_time == 0:
            if first_time == 0:
                first_time = 1
            if fire_lock_7 == 0 and fire_lock_8 == 0:
                stop_index = (stop_index+1)%4
                data = gps.getValues()
                arm_loc = (data[0], data[2])
                fire_lock_7 = 1
            elif fire_lock_7 == 1:
                temp = move(arm_loc, dum_loc[stop_index], stop_index)
                #stop_index = (stop_index-1)%4
                if temp == 1:
                    fire_lock_7 = 0
        else:
            fire_lock_8 = 1
            fire_lock_6 = 1
    #rotate to the final angle        
    elif fire_lock_8 == 1:
        print("here!!!!")
        temp = get_to_operation(dum_imu[max_light_index])
        if temp == 1:
            fire_lock_8 = 0
            fire_lock_9 = 1            
    elif fire_lock_9 == 1:
        fire_lock_1 = 0
        fire_lock_2 = 0
        fire_lock_3 = 0
        fire_lock_4 = 0
        fire_lock_5 = 0
        fire_lock_6 = 0
        fire_lock_7 = 0
        fire_lock_8 = 0
        fire_lock_9 = 0
        ret = 1
    return ret

###############################################motor_move
motor1_direction=0
motor1_stop=0
motor2_stop=0
motor3_stop=0
motor4_stop=0  
max_fire1 = 0
temp_fire1 = 0
max_fire2 = 0
max_fire3 = 0
temp_fire2 = 0
temp_fire3 = 0
theta_1=0
distance_fire=0
theta_0=0
max_2motors=0
arm_mode = False
def robot_arm_moving():
   global motor1_stop, motor2_stop, motor3_stop, motor4_stop,temp_fire1,\
           max_fire1,temp_fire2,max_fire2,theta_1,distance_fire,theta_0,\
           temp_fire3,max_fire3,motor1_direction,max_2motors,arm_mode,task_done 
#movement for motor 1
   if(motor1_stop==0):
       #print('1')
       
       if motor1_direction == 0:
           if(light_sensor_lu_left.getValue()>light_sensor_lu_right.getValue()):
                motor1_direction=1
           else:
                motor1_direction=2                   
       elif(motor1_direction==1):                     #turn left
            temp_fire1 = light_sensor_1.getValue()
            print("fire readings: " + str(temp_fire1))   
            if(temp_fire1>=max_fire1):
                wheels[4].setVelocity(0.5)
                max_fire1=temp_fire1
            else:
                wheels[4].setVelocity(0)
                motor1_stop=1
       elif(motor1_direction==2):                      #turn right
            temp_fire1 = light_sensor_1.getValue()  
            print("fire readings: " + str(temp_fire1)) 
            if(temp_fire1>=max_fire1):
                wheels[4].setVelocity(-0.5)
                max_fire1=temp_fire1
            else:
                wheels[4].setVelocity(0)
                motor1_stop=1                

#movement for motor 3 to test fire location          
   elif(motor3_stop==0):
       #print('2')
       temp_fire2 = light_sensor_2.getValue()
       if(temp_fire2>=max_fire2):
           wheels[6].setVelocity(0.5)
           max_fire2=temp_fire2
       else:
           wheels[6].setVelocity(0)
           motor3_stop=1
           theta_1 = motor4_imu.getRollPitchYaw() 
           distance_fire=0.18/math.cos(theta_1[1])#+0.065*math.tan(theta_1[1])
           if((distance_fire-0.27)/0.18>=0.72):
                theta_0=0.785
           else:
                theta_0=math.acos((distance_fire-0.27)/0.18) 
                     
#get the theta1 angle
   elif(motor3_stop==1):
       #print(motor3_stop)
       temp_theta_0=motor2_imu.getRollPitchYaw()
       if((pI/2-math.fabs(temp_theta_0[1]))>theta_0):
           wheels[5].setVelocity(0.5)
       else:
           motor3_stop=2
           #print(motor3_stop)
           wheels[5].setVelocity(0)
           motor3_stop==2
           motor2_stop=1
           
#move motor3 to correct postition depend on angle thta0          
   elif(motor2_stop==1):
       #print('4')
       #print((motor4_imu.getRollPitchYaw())[1]/pI*180)
       if((motor4_imu.getRollPitchYaw())[1]>0):
           wheels[6].setVelocity(0.5)
       else:
           wheels[6].setVelocity(0)
           motor2_stop=2

#         
   elif(motor2_stop==2):
       temp_fire3 = light_sensor_3.getValue()
       if(temp_fire3>=max_fire3):
           wheels[7].setVelocity(0.5)
           max_fire3=temp_fire3
       else:
           wheels[7].setVelocity(0)          
           motor2_stop=3
           
#         
   elif(motor2_stop==3):
       if(light_sensor_3.getValue() < 500):
                temp_fire_s=light_sensor_3.getValue()
                if(temp_fire_s>=max_2motors):
                   wheels[7].setVelocity(-0.5)
                   wheels[6].setVelocity(0.3)
                   max_2motors=temp_fire_s
                else:
                   wheels[6].setVelocity(0)
                   wheels[7].setVelocity(0)
                   motor2_stop=4         
       else:
                motor2_stop=4
                wheels[6].setVelocity(0)
                wheels[7].setVelocity(0)
                task_done = True
                arm_mode = False
      
######################################################

while robot.step(TIME_STEP) != -1:
   # xyz = []
    #xyz=tuoluoyi.getRollPitchYaw()
   # print('x=', xyz[0]/3.14*180,' ','y=', xyz[1]/3.14*180,' ','z=', xyz[2]/3.14*180)
     #print(jieshouqi.getDataSize()) 
    leftSpeed = 10.0
    rightSpeed = 10.0
    
    v1 = ds[0].getValue()
    v2 = ds[1].getValue()
    #print("ds_l is: " + str(v1))
    #print("ds_r is: " + str(v2))
     #if there is a message to be recieved
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
                coords,dinfo = my_parser.coords(r_message)

                start = gps.getValues()

                for a in coords:
                    ccoords.append(float(a))
                start = conv_coords(start)
                ccoords = conv_coords(ccoords)
                mode = 0
                
                #TODO: set up actual task
                angle = math.radians(float(dinfo[0])) # imu of dum car                
                width = float(dinfo[1]) # length of dum car
                length = float(dinfo[2]) # width of dum car
                set_global_angle(float(dinfo[0]))
                #print(dum_angle)
                #set the four location to traverse.
                dum_loc[0] = (float(coords[0]) + (length+dim[2]/2)*math.sin(angle),float(coords[2])+(length+dim[2]/2)*math.cos(angle)) #top
                dum_loc[1] = (float(coords[0])+ (width+dim[2])*math.cos(angle),float(coords[2])-(width+dim[2])*math.sin(angle)) #right
                dum_loc[2] = (float(coords[0]) - (length+dim[2]/2)*math.sin(angle),float(coords[2])-(length+dim[2]/2)*math.cos(angle)) #bot
                dum_loc[3] = (float(coords[0]) - (width+dim[2])*math.cos(angle),float(coords[2])+(width+dim[2])*math.sin(angle)) #left
                print(dum_loc[0])
                print(dum_loc[1])
                print(dum_loc[2])
                print(dum_loc[3])
                if my_rrt is None:
                    rrt_create(start,ccoords)
                else:
                    rrt_new(start,ccoords)
                in_transit = True
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
        
        print("finding fire...")
        #should prepare to operate on car
        #then do its specific task
        #from there, send fin
        task_done = False
        temp = move_to_fire()
        do_job_now = False
  
        if temp == 1:
            #start_find = 0
            no_move()
            arm_mode = True
            print("done finding fire!") 
            #go to arm mode
            mode = 8
        
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
    elif mode == 8:
        robot_arm_moving()
        if task_done == True:
            send_fin_task()
            my_pos = gps.getValues()
            my_pos = conv_coords(my_pos)
            add_obstacle(ccoords,(dinfo[2]*10-1))
            end_pos = conv_coords([2.5,0.04,2.0])
            rrt_new(my_pos,end_pos)
            ending = True
            mode = 0
            print('done with arm')
    #test mode
    elif mode == 10:
        
        go_ideal()
        print(mode)

             
             
             



##franks code i think, to go around car
#define goal location.
#             goal_x = float(coord[1])*10+30
#             goal_y = 60-(float(coord[5])*10+30)
#             angle = math.radians(float(coord[7])) # imu of dum car
#             length = float(coord[9]) # length of dum car
#             width = float(coord[11]) # width of dum car
             #dum_len = length
             #dum_wid = width
             #dum_gps = (float(coord[1]),float(coord[5]))
#             set_global_angle(float(coord[7])) # set global variable dum_angle for future use
             #set the four location to traverse.
#             dum_loc[0] = (float(coord[1]) + (length+dim[2]/2)*math.sin(angle),float(coord[5])+(length+dim[2]/2)*math.cos(angle)) #top
#             dum_loc[1] = (float(coord[1])+ (width+dim[2])*math.cos(angle),float(coord[5])-(width+dim[2])*math.sin(angle)) #right
#             dum_loc[2] = (float(coord[1]) - (length+dim[2]/2)*math.sin(angle),float(coord[5])-(length+dim[2]/2)*math.cos(angle)) #bot
#             dum_loc[3] = (float(coord[1]) - (width+dim[2])*math.cos(angle),float(coord[5])+(width+dim[2])*math.sin(angle)) #left
#             print(str(dum_loc[0]))
#             print(str(dum_loc[1]))
#             print(str(dum_loc[2]))
#             print(str(dum_loc[3]))


##############specific task, i think
#    if start_find == 1:
#         temp = move_to_fire()
#         if temp == 1:
#             start_find = 0
#             print("done!!!!")

#############doing rrt then after go to specific task <- lu's code i think

#     if start_rrt == 1:
#         temp = move_car_on_rrt()
         #print(temp[0])
#         print(temp[0])
#         if(temp[0] == 2):
#             start_rrt = 0
#             start_find = 1
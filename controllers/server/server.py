from controller import Robot, DistanceSensor, Motor,Receiver,Emitter
import struct
import my_parser
import queue
import math
TIME_STEP = 64
robot = Robot()
EN = 'utf=8'
MAX_ITEMS = 6
#queue for jobs
jobs_list = queue.PriorityQueue(maxsize=MAX_ITEMS)

#jobs_list.put((1,'hello'))
#jobs_list.put((2,'hi'))
#a = jobs_list.get()
#b = jobs_list.get()
#print(a)
#print(b)

#dictionary of car name,priority,channel, (maybe:car's dimensions in radius)
helper_dict = {
    'FIRE': (2,3,2.5),
    'TOW': (3,2,2.5) 
}

#list of registered helper cars
helper_list = ['FIRE','TOW']

#for debugging ideal pos
#jobs_list.put((helper_dict['TOW'][0],'TOW'))

client_sender=robot.getEmitter('emitter')
client_sender.setChannel(1)
client_sender.setRange(-1)

client_receiver = robot.getReceiver('receiver')
client_receiver.enable(TIME_STEP)
client_receiver.setChannel(1)

helper_sender=robot.getEmitter('emitter2')
helper_sender.setChannel(2)
helper_sender.setRange(-1)

helper_receiver = robot.getReceiver('receiver2')
helper_receiver.enable(TIME_STEP)
helper_receiver.setChannel(2)

#index to keep track of which cars have been queried, used for collision avoidance
coll_iter = 0
#list that holds tuple of car and car information: (name,prio,coordinates,size)
cars_in_reg = queue.PriorityQueue(maxsize=MAX_ITEMS)
region_th = 1
coll_coords = None

send_m = False
mode = ""

def is_in_coll_region(coords_reg,coord_car):
    x_r = coords_reg[0]
    z_r = coords_reg[2]
    x_c = coords_car[0]
    z_c = coords_car[2]
    #region is a circle equation, we can tell if a car is in the region
        #if their car's coordinates are less than or equal to the equation 
    d = math.sqrt((x_r-x_c) ** 2 + (z_r-z_c) ** 2)
    #if distance is less than region threshold, then car is in the vicinity
    if d < region_th:
        return True
    return False
def query_for_reg():
    car_name = helper_list[coll_iter]
    car_info = helper_dict[car_name]
    ch = car_info[1]
    helper_sender.setChannel(ch)
    helper_sender.send("REQ COLL")
    coll_iter = coll_iter + 1   
def send_reg():
    info = []
    items = []
    obstacles = ""
    first = cars_in_reg.get()
    #while queue not empty, append info to string to prep for sending
    while not cars_in_reg.empty():
        item = cars_in_reg.get()
        items.append(item)
        #format of out_message:
            #<x y z coordinates>, <car radius>
        obstacles = obstacles + " " + item[1][0] + " " + item[1][1] + " " + item[1][2] + " " + item[2]
    i = 0
    #restore queue
    for item in items:
        cars_in_reg.put(items)
    #send to highest prio car
    out_message = bytes("RESP COLL" + obstacles, EN)
    helper_sender.setChannel(helper_dict[first[0]][0])
    helper_sender.send(out_message)
    
    
         

#TODO: implement the collision avoidance protocol
        #fix the messaging formats 


while robot.step(TIME_STEP) != -1:
    
    #this mode is reserved to send the highest priority car the obstacles
    if mode == "COLL PROTOCOL: 1":
        #query all cars
        if coll_iter < len(helper_list):
            query_for_reg()
            #reset mode?
            #coll_iter = 0
            #mode = ""
            #send highest prio car the
        #all known cars have been queried, move to next step
        else:
            mode = "COLL PROTOCOL 2"
            send_reg()    
                   
    elif client_receiver.getQueueLength() > 0: 
        if client_receiver.getDataSize()>1:
             #print(jieshouqi.getData())
             message=client_receiver.getData()
             #decode from bytes to string
             whole_message = message.decode("utf-8")
             
             #print(rec_message)
             #r_message = rec_message.split()
             #car_name = r_message[len(r_message)-1]
             out_message = bytes(' ',EN)
             
             #generate general message
             type,comm,cinfo,car = my_parser.gen(whole_message)
             
             
             
             #if "FIRE" in r_message:  
             #    helper_sender.setChannel(3)             
             
             #if in step 2 of collision protocol
             if mode == "COLL PROTOCOL 2":
                 #if received confirmation of node met
                 print(comm)
                 
             
             elif "REQ" in type:
                 if "HELP" in comm:
                     #to prevent duplicate jobs
                     # currently innefficient implementation, but works for now 
                     jobs = []
                     #get all jobs
                     nondp = True
                     while not jobs_list.empty():
                         jobs.append( jobs_list.get())
                     #check if job already exists
                     for job in jobs:
                         print("jobs: " + job[1])
                         if job[1] == car:
                             nondp = False
                     #restore jobs_list
                     for job in jobs:
                         jobs_list.put(job)
                     if nondp: 
                         print("inserting " + str(helper_dict[car][0]) + " " + car)    
                         coords,dinfo = my_parser.coords(cinfo)
                         jobs_list.put((helper_dict[car][0],car))
                         
                         #print(helper_dict[car][0])
                         out_message = "GO COORDS "+ str(coords[0]) +" "+ str(coords[1]) +" "+ str(coords[2]) + " "\
                                       + "DIM " + str(dinfo[0]) +" "+ str(dinfo[1]) +" "+ str(dinfo[2]) +" "+ car
                         print(out_message)
                         send_m = True
                     
                     else:
                         print('duplicate job')
                 if "JOBS" in comm:
                     job = jobs_list.get()
                     jobs_list.put(job)
                     out_message = "RESP JOBS " + str(job[1]) + "T " + car
                     send_m = True
                 #request for collision avoidance at this coordinate
                 elif "COLL" in comm:
                     coords = my_parser.coords(cinfo)
                     coll_coords = coords
                     mode = "COLL PROTOCOL"
             elif "RESP" in type:
                 #case for when job is completed
                 if "HELP" in comm:
                     job = jobs_list.get()
                     print(job)
                     #print(job[1] + " completed task")
                 if "COLL" in comm:
                     coords = my_parser.coords(cinfo)
                     in_reg = is_in_coll_region(coll_coords,coords)
                     if in_reg:
                         cars_in_reg.put(helper_dict[car][0],[car,coords,str(helper_dict[car][2])])
                             
            
                       
                 
             #elif "TOW" in r_message:
             #    helper_sender.setChannel(2)
             #    if "COORDS" in r_message:
             #        coords,car = my_parser.server(r_message)
             #        jobs_list.put((3,'TOW'))
             #        out_message = bytes("GO "+ coords[0] +" "+ coords[1] +" "+ coords[2],EN)
             #        send_m = True
             #    elif "REQ" in r_message:
             #        if "JOBS" in r_message"
             #            job = jobs_list.get()
             #            jobs_list.put(job)
             #            out_message = bytes("REQ " + str(job[1]),EN)
             #            send_m = True
             #elif "REKT" in r_message:
             #    coords,car = my_parser.server(r_message)
             #    print(r_message)
             
                 
                 #query_for_reg()
                 #send to the highest prio car
                     #region, positions of cars:size, 
             #set channel for appropriate car
             
             #print(car)
             #print(helper_dict[car][1])
             if send_m:
                 out_b = bytes(out_message, EN)
                 helper_sender.setChannel(helper_dict[car][1])
                 helper_sender.send(out_b)
             send_m = False    
             client_receiver.nextPacket()

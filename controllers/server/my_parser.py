
def gen(message):
    str = message.split()
    type = str[0]
    comm = str[1]
    i = 2
    info = []
    while i < len(str)-1:
        #print(str[i])
        info.append(str[i])
        i = i + 1
    car = str[len(str)-1]
    return type,comm,info,car

def coords(message):
    info = []
    
    if isinstance(message,str):
        mes = message.split()
    else:
        mes = message
    #command = str[0]
    i = 0
    for word in mes:
        if word == "COORDS":
            i = i + 1
            break;
        i = i + 1
    coord_x = float(mes[i])
    coord_y = float(mes[i+1])
    coord_z = float(mes[i+2])
    
    if "DIM" in mes:
        info.append(float(mes[i+4]))
        info.append(float(mes[i+5]))
        info.append(float(mes[i+6]))
        
    #car = str[len(str)-1]
    return [coord_x,coord_y,coord_z],info

def car(message):
    str = message.split()
    command = str[0]
    coord_x = str[1]
    coord_y = str[2]
    coord_z = str[3]
    return (command,(coord_x,coord_y,coord_z))
    
        
#algorithm to find the ideal spot

#need parameter for where car is
#need variables for the workinging area perimeter
#need parameter for the ideal perimeter

import math
import matplotlib.pyplot as plt
import numpy as np
class Ideal_Pos:
    

    def __init__(self,r1=1,r2=2,t=30,show=False):
        #working radius around car
        self.working_rad = r1
        #ideal radius should be larger than working_radius
        self.ideal_rad = r2
        #increments of theta to search for
        self.theta_it = t
        #holds values of the ideal positions
        self.positions = []
        #holds values of the distances to the ideal positions
        self.dists = []
        #holds the indexes of the distance values after being sorted ascending
        self.indexes = []
        self.draw_ani = show

    def get_pos(self,x2,y2):
        theta = 0
        pos = []
        while theta < 360:  
            rad = math.radians(theta)
            x = x2 + ((self.ideal_rad+self.working_rad)/2) * math.cos(rad)
            y = y2 + ((self.ideal_rad+self.working_rad)/2) * math.sin(rad)
            pos.append((x,y))
            theta = theta + self.theta_it
        return pos
    def get_dists(self,x,y):
        diss = []
        for pos in self.positions:
            dist = math.sqrt((x-pos[0])**2 + (y-pos[1])**2)
            diss.append(dist)
        return diss
    def get_ideal(self,x1,y1,x2,y2):
        #x,y is the center, 
        #split circle into segments
        #find the x,y for each segment
        self.positions = self.get_pos(x2,y2)
        #print(self.positions)
        if self.draw_ani:
            self.my_plot(x1,y1)
        #find the distance from you
        self.dists = self.get_dists(x1,y1)
        #print(self.dists)
        #sort the shortest distance
        #return the index according to the shortest distance
        self.indexes = sorted(range(len(self.dists)), key=lambda k: self.dists[k])
        #print(self.indexes)
        return self.positions, self.indexes
    #get the equation of a line
    def get_line(self,y1,y2,x1,x2):
        beg = x1
        end = x2
        if x1 > x2:
            beg = x2
            end = x1
        x = np.arange(beg,end,0.01)
        m = (y1-y2)/(x1-x2)
        y = m*(x - x1) + y1
        return(x,y)
    def my_plot(self,x1,y1):
        xs = [ x for (x,y) in self.positions]
        ys = [ y for (x,y) in self.positions]
        #print(xs)
        #print(ys)
        #eqs = []
        for pos in self.positions:
            x,y = self.get_line(y1,pos[1],x1,pos[0])
            #eqs.append(y)
            plt.plot(x,y)
        plt.plot(xs,ys,marker='o')
        plt.show()
        
        
def main(cur_x=-2,cur_z=4, car_x=2,car_y=2,show_plot = True):
    
    pos = Ideal_Pos(show=show_plot)
    pos, ind = pos.get_ideal(cur_x,cur_z,car_x,car_y)
    print(pos)
    print(ind)

if __name__ == '__main__':
    main()

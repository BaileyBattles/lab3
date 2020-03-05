import matplotlib.pyplot as plt
import numpy
import matplotlib.patches as patch
import random
from matplotlib.animation import FuncAnimation
import matplotlib as mpl

#Starting point
X_START = 0
Y_START = 0

#Ending point
X_END = 40
Y_END = 40

#Length of the field
X_LENGTH = 100
Y_LENGTH = 100

ROBOT_RADIUS = 3
ITERATION_SIZE = 1
END_PRB_SAMPLE = 5
MAX_ITERATIONS = 2000

OMEGA_LEFT = 1
OMEGA_RIGHT = 1
V_FORWARD = 1

THRESHOLD = 1
FRAME_RATE = 50 #milliseconds

COUNTERCLOCKWISE_SPEED = 69.23 #Degrees per second
CLOCKWISE_SPEED  = 79.23 #Degrees per second
FORWARD_SPEED = 1.188 #CM per second 

class point():
    def __init__(self,x,y):
        self.x = x
        self.y = y
    
    def distanceTo(self, otherPoint):
        return numpy.sqrt(pow((float(self.x) - float(otherPoint.x)), 2) + \
                          pow((float(self.y) - float(otherPoint.y)), 2))

    def angleTo(self, end):
        dx = float(end.x) - self.x
        dy = float(end.y) - self.y
        #Check Edge Cases then check each quadrant
        if(dx == 0 and dy == 0):
            theta = 0 
        elif(dx > 0 and dy == 0):
            theta = 0
        elif(dx == 0 and dy > 0):
            theta = 90
        elif(dx < 0 and dy == 0 ):
            theta = 180
        elif(dx == 0 and dy < 0):
            theta = 270
        elif(dx>0 and dy>0):
            theta = numpy.arctan(dy/dx) * (180/numpy.pi)
        elif(dx<0 and dy>0):
            theta = 180 - (numpy.arctan(-dy/dx)*(180/numpy.pi))
        elif(dx<0 and dy<0):
            theta = 180 + (numpy.arctan(dy/dx)*(180/numpy.pi))
        elif(dx>0 and dy<0):
            theta = 360 - (numpy.arctan(-dy/dx)*(180/numpy.pi))
        return theta

class obstacle():
    def __init__(self,point,width,height):
        self.point = point #Bottom left corner of box
        self.height = height
        self.width = width 

class vertex():
    def __init__(self,point,index_prev):
        self.point = point
        self.index_prev = index_prev

class RRT():
    def __init__ (self, map):
        self.map = map
        self.iteration_size = ITERATION_SIZE
        self.end_prb_sample = END_PRB_SAMPLE
        #First vertex does not have a previous 
        self.vertices = [vertex(point=map.start,index_prev=-1)]
        self.path = []    
    def return_path(self):
        return self.path

    def RRT_plan(self):
        success = 0
        for i in range(0,MAX_ITERATIONS):
            if(random.randint(0,100)> self.end_prb_sample):
                random_x = random.uniform(0,X_LENGTH)
                random_y = random.uniform(0,Y_LENGTH)
            else:
                random_x = self.map.end.x
                random_y = self.map.end.y

            nearest_index = self.calc_closest_vertex(point(random_x,random_y)) 
            nearest_vertex = self.vertices[nearest_index]

            theta = self.calc_angle_to_point(nearest_vertex.point,point(random_x,random_y))
            if(not self.is_collision(point(nearest_vertex.point.x+ITERATION_SIZE*numpy.cos(theta*numpy.pi/180),nearest_vertex.point.y+ITERATION_SIZE*numpy.sin(theta*numpy.pi/180)))):
                pt = point(nearest_vertex.point.x+ITERATION_SIZE*numpy.cos(theta*numpy.pi/180),nearest_vertex.point.y+ITERATION_SIZE*numpy.sin(theta*numpy.pi/180))
                new_vertex = vertex(pt,nearest_index)
                self.vertices.append(new_vertex)
                #plt.plot(new_vertex.point.x,new_vertex.point.y,"g.")
                dx = self.map.end.x-new_vertex.point.x
                dy = self.map.end.y-new_vertex.point.y
                dist = numpy.sqrt(dx*dx + dy*dy)
                if(dist<=THRESHOLD):
                    success=1
                    #plt.show()
                    break
        
        if(success == 1):
            vt = self.vertices[len(self.vertices)-1]
            while(vt.index_prev!=-1):
                self.path.insert(len(self.vertices),vt.point)
                vt = self.vertices[vt.index_prev] 
            for pt in self.path:
                plt.plot(pt.x,pt.y,"r.")
            return self.path
            #plt.show()
                




    def is_collision(self, pt):
        for obs in self.map.obstacles:
            if(pt.x>obs.point.x and pt.x<(obs.point.x+obs.width) and pt.y>obs.point.y and pt.y<(obs.point.y+obs.height)):
                return True
        for obs in self.map.config_obstacles:
            if(pt.x>obs.point.x and pt.x<(obs.point.x+obs.width) and pt.y>obs.point.y and pt.y<(obs.point.y+obs.height)):
                return True
        return False   


    def calc_closest_vertex(self,point):
        #Used squared distance as the dist metric
        min_dist = -1
        min_index = -1
        index = 0 
        for v in self.vertices:
            if(min_dist == -1):
                dx = v.point.x-point.x
                dy = v.point.y-point.y
                dist = (dx*dx + dy*dy)
                min_dist = dist
                min_index = 0
            else:
                dx = v.point.x-point.x
                dy = v.point.y-point.y
                dist = (dx*dx + dy*dy)
                if(dist<min_dist):
                    min_dist = dist
                    min_index = index
            index = index+1 
        return min_index
    
    def calc_angle_to_point(self,start,end):
        dx = float(end.x) - start.x
        dy = float(end.y) - start.y
        #Check Edge Cases then check each quadrant
        if(dx == 0 and dy == 0):
            theta = 0 
        elif(dx > 0 and dy == 0):
            theta = 0
        elif(dx == 0 and dy > 0):
            theta = 90
        elif(dx < 0 and dy == 0 ):
            theta = 180
        elif(dx == 0 and dy < 0):
            theta = 270
        elif(dx>0 and dy>0):
            theta = numpy.arctan(dy/dx) * (180/numpy.pi)
        elif(dx<0 and dy>0):
            theta = 180 - (numpy.arctan(-dy/dx)*(180/numpy.pi))
        elif(dx<0 and dy<0):
            theta = 180 + (numpy.arctan(dy/dx)*(180/numpy.pi))
        elif(dx>0 and dy<0):
            theta = 360 - (numpy.arctan(-dy/dx)*(180/numpy.pi))
        return theta





class map:
    def __init__(self,start,end,obstacles):
        self.obstacles = obstacles
        self.start = start
        self.end = end
        self.config_obstacles = [] #Obstacles added because of the robot's radius
    
    def plot_map(self):
        plt.clf()
        #Plot the points
        ax = plt.gca() 
        for obs in self.obstacles: 
            rect = patch.Rectangle((obs.point.x,obs.point.y),obs.width,obs.height)
            ax.add_patch(rect)
        plt.plot(self.start.x,self.start.y,"r*")
        plt.plot(self.end.x,self.end.y,"r*")
        plt.axis([0,X_LENGTH,0,Y_LENGTH])
        plt.grid(True)
        #plt.show()
        #Plot the obstacles

    def plot_config_space(self):
        #plt.clf()
        ax = plt.gca() 
        plt.plot(self.start.x,self.start.y,"r*")
        plt.plot(self.end.x,self.end.y,"r*")
        for obs in self.obstacles: 
            # RIGHT
            x = obs.point.x + obs.width 
            y = obs.point.y - ROBOT_RADIUS
            width = ROBOT_RADIUS
            height = obs.height + 2*ROBOT_RADIUS
            self.config_obstacles.append(obstacle(point(x,y),width=width, height=height))

            #TOP
            x = obs.point.x - ROBOT_RADIUS
            y = obs.point.y + obs.height
            width = obs.width + 2*ROBOT_RADIUS
            height = ROBOT_RADIUS
            self.config_obstacles.append(obstacle(point(x,y),width=width, height=height))

            #LEFT
            x = obs.point.x - ROBOT_RADIUS
            y = obs.point.y - ROBOT_RADIUS
            width = ROBOT_RADIUS
            height = obs.height + 2*ROBOT_RADIUS
            self.config_obstacles.append(obstacle(point(x,y),width=width, height=height))


            #BOTTOM
            x = obs.point.x - ROBOT_RADIUS
            y = obs.point.y - ROBOT_RADIUS
            width = obs.width + 2*ROBOT_RADIUS
            height = ROBOT_RADIUS
            self.config_obstacles.append(obstacle(point(x,y),width=width, height=height))
        
        for obs in self.config_obstacles:
            rect = patch.Rectangle((obs.point.x,obs.point.y),obs.width,obs.height,color='purple')
            ax.add_patch(rect)
        plt.axis([0,X_LENGTH,0,Y_LENGTH])
        plt.grid(True)
        #plt.show()



class Controller:
    def __init__(self, path, start, end):
        """
        Path is a list as given from RRT_plan
        """
        path.append(start)
        path = path[::-1]
        path.append(end)
        self.path = path
    
    def calculate_trajectories(self):
        trajectory = []
        theta = 0
        for i in range(0, len(self.path) - 1):
            start = self.path[i]
            end = self.path[i + 1]
            new_theta = start.angleTo(end)
            
            dTheta = new_theta - theta
            if dTheta > 180:
                dTheta -= 360
            elif dTheta < -180:
                dTheta += 180

            dist = start.distanceTo(end)

            theta = new_theta
            trajectory.append((dTheta, dist))
        return trajectory
    
    def plot_trajectory(self, trajectory):
        plt.figure()
        for v in self.path:
            print(v.x, v.y)
        print("\n\n\n\n\n")
        for x in trajectory:
            print(x)
        for i in range(len(trajectory)):
            v = self.path[i]
            dx = numpy.cos(numpy.radians(trajectory[i][1]))
            dy = numpy.sin(numpy.radians(trajectory[i][1]))
            print(v.x, v.y)
            plt.plot([v.x, v.x + dx], [v.y, v.y + dy])
        #plt.show()
        
    def calculate_times(self, trajectory): 
        for segment in trajectory:
            #t_theta = float(segment[0])/ROTATION_SPEED     #ROTATION_TIME
            t_forward = float(segment[1])/FORWARD_SPEED   #FORWARD_TIME
            if(segment[0] < 0 ):
                t_theta = float(segment[0])/CLOCKWISE_SPEED
                print("%.2f,f,%.2f" % (-t_theta, t_forward))
            elif(segment[0] >= 0):
                t_theta = float(segment[0])/COUNTERCLOCKWISE_SPEED
                print("%.2f,l,%.2f" % (t_theta,t_forward))
  

        
        
        
class point_plotter():
    def __init__(self,path,trajectories,my_map):
        plt.clf()
        self.map = my_map 
        self.Acc_11 = []
        self.Acc_12 = []
        self.theta = []
        for segment in path:
            self.Acc_11.insert(0,segment.x)
            self.Acc_12.insert(0,segment.y)
        for angle in trajectories:
            self.theta.insert(0,angle[0])
        self.fig = plt.figure(figsize = (5,5))
        self.axes = self.fig.add_subplot(111)
        self.patch = plt.Rectangle((5,5),ROBOT_RADIUS,ROBOT_RADIUS) 
        self.i=0

        # self.Acc_11 = [1,2,3,4,6,7]
        # self.Acc_12 = [2,2,2,2,2,2]
    def plot_obstacles(self):
        for obs in self.map.obstacles: 
            rect = patch.Rectangle((obs.point.x,obs.point.y),obs.width,obs.height)
            self.axes.add_patch(rect)
        self.axes.plot(self.map.start.x,self.map.start.y,"r*")
        self.axes.plot(self.map.end.x,self.map.end.y,"r*")
    
    def plot(self):
        # Scatter plot
        def frames():
            for acc_11_pos, acc_12_pos, theta in zip(self.Acc_11, self.Acc_12, self.theta):
                yield acc_11_pos, acc_12_pos, theta 

        def ani(coords):
            x,y = self.patch.xy
            x = coords[0]
            y = coords[1]
            theta = coords[2]
            self.patch.angle= theta
            self.patch.set_xy([x,y]) 
            #point.set_data([coords[0]],[coords[1]])
            return self.patch,point
        
        self.axes.add_patch(self.patch)
        self.axes.set_xlim(0, 100)
        self.axes.set_ylim(0, 100)

        point, = self.axes.plot([self.Acc_11[0]],[self.Acc_12[0]], 'go')
        
        ani = FuncAnimation(self.fig, ani, frames=frames, interval=50)
        #interval = millisecond delay
        plt.show()





def main():
    start = point(X_START,Y_START)
    end = point(X_END,Y_END)
    obstacles = [
        obstacle(point(15,15),width=7,height=5),
        obstacle(point(24,24),width=8,height=5)
    ]
    
    my_map = map(start,end,obstacles)
    my_map.plot_map()
    my_map.plot_config_space()
    rrt = RRT(my_map)
    path = rrt.RRT_plan()
    controller = Controller(path, start, end)
    trajectory = controller.calculate_trajectories()
    controller.calculate_times(trajectory)
    path = rrt.return_path()
    pp = point_plotter(path,trajectory,my_map)
    pp.plot_obstacles()
    pp.plot()
    #controller.plot_trajectory(trajectory)

if __name__ == '__main__':
    main()
import numpy as np
import matplotlib as plt
import cv2 as cv
import time
import heapq

s_time=time.time()
# Start node
Startnode=input("Please enter the Start node coordinates")
Start_x,Start_y=Startnode.split()
Start_x=int(Start_x)
Start_y=int(Start_y)
Startnode=(Start_x,Start_y)
# Goal node
Goalnode=input("Please enter the Goal node coordinates")
Goal_x,Goal_y=Goalnode.split()
Goal_x=int(Goal_x)
Goal_y=int(Goal_y)
Goalnode=(Goal_x,Goal_y)
# validnode(Start_x,Start_y)

map = np.full((250, 600), 255, dtype=np.uint8)
# validnode(Start_x,Start_y)

# Defining obstacle with clearance for point bot
def totalobstacle():
    
    # Hexagon
    center = (300, 125)
    side_length = 80
    x_coords = center[0] + side_length*np.array([1, np.cos(np.pi/3), -np.cos(np.pi/3), -1, -np.cos(np.pi/3), np.cos(np.pi/3)])
    y_coords = center[1] + side_length*np.array([0, np.sin(np.pi/3), np.sin(np.pi/3), 0, -np.sin(np.pi/3), -np.sin(np.pi/3)])
    pts1= np.array([(x, y) for x, y in zip(x_coords, y_coords)], np.int32)
    pts = pts1.reshape((-1,1,2))
    cv.polylines(map, [pts1], True, (0, 255, 0), thickness=1)
    cv.fillPoly(map, [pts1], color=(0, 0, 0))

    # Rectangles
    cv.rectangle(map,(95,0),(155,105),0,-1)
    cv.rectangle(map,(94,145),(155,250),0,-1)
    # Triangle
    pts2= np.array([[405,10], [405, 240], [465, 125]], np.int32)
    cv.polylines(map, [pts2], True, (0,255,0), thickness=1)
    cv.fillPoly(map, [pts2], color=(0, 0, 0))
    cv.imshow('Map',map)
    cv.waitKey(0)

# Defining actualobstacle
def actualobstacle():

    # Hexagon
    center = (300, 125)
    side_length = 75
    x_coords = center[0] + side_length*np.array([1, np.cos(np.pi/3), -np.cos(np.pi/3), -1, -np.cos(np.pi/3), np.cos(np.pi/3)])
    y_coords = center[1] + side_length*np.array([0, np.sin(np.pi/3), np.sin(np.pi/3), 0, -np.sin(np.pi/3), -np.sin(np.pi/3)])
    pts3= np.array([(x, y) for x, y in zip(x_coords, y_coords)], np.int32)
    pts3 = pts3.reshape((-1,1,2))
    cv.fillPoly(map, [pts3], (127, 127, 127))
    # Rectangles
    cv.rectangle(map,(100,0),(150,100),(127,127,127),-1)
    cv.rectangle(map,(100,150),(150,250),(127,127,127),-1)
    # Triangle
    pts4 = np.array([[410, 25], [410, 225], [460, 125]], np.int32)
    cv.polylines(map, [pts4], isClosed=True,color=(0,0,0), thickness=1)
    cv.fillPoly(map, [pts4], (127, 127, 127))

    cv.imshow('Map',map)
    cv.waitKey(0)


   
# Actions
moves=[]
def move_north(x,y,ctc):
    if y>0: 
        x=x
        y-=1
        ctc+=1
        return (x,y),ctc
    else:
        return(None),None

def move_south(x,y,ctc):
    if y<599:
        x=x
        y+=1
        ctc+=1
        return (x,y),ctc
    else:
        return(None),None

   
def move_east(x,y,ctc):
    if x<249:
        x+=1
        y=y
        ctc+=1
        return (x,y),ctc
    else:
        return(None),None

def move_west(x,y,ctc):

    if x>0:
        x-=1
        y=y
        ctc+=1
        return (x,y),ctc
    else:
        return(None),None
    

def move_northeast(x,y,ctc):
    if x>0 and y<599: 
        x-=1
        y+=1
        ctc+=1.4
        return (x,y),ctc
    else:
        return(None),None
    

def move_northwest(x,y,ctc):
    if y>0 and x>0:
        x-=1
        y-=1
        ctc+=1.4
        return (x,y),ctc
    else:
        return(None),None
    

def move_southeast(x,y,ctc):
    if y<599 and x<249:
        x+=1
        y+=1
        ctc+=1.4
        return (x,y),ctc
    else:
        return(None),None
    

def move_southwest(x,y,ctc):
    if y<599 and x>0:

        x-=1
        y+=1
        ctc+=1.4
        return (x,y),ctc
    else:
        return(None),None

def generate_path(final_dict,node):
    path=[]
    path.append(node)
    parent_node=final_dict[node]
    while parent_node is not None and parent_node != Startnode:
        path.append(parent_node)
        parent_node=final_dict[parent_node]
    path.append(Startnode)
    path.reverse()
    return path

# Dijkstra 
def dijkstra(Startnode,Goalnode):

    totalobstacle()
    actualobstacle()

    final_dict={}
    initial_list=[]
    final_list=set()
    explored_list=set() #final_dict
    # ctc=0
    # startnode=(Start_x,Start_y)
    # goalnode=(Goal_x,Goal_y)
    heapq.heappush(initial_list,(0,Startnode))
   
    while initial_list:
        cost,node=heapq.heappop(initial_list)
       
        if node not in explored_list:
            explored_list.add(node)
        final_list.add(node)
        if not validnode(node[0],node[1]):
            continue

        if node[1] == Goalnode[0] and node[0]==Goalnode[1]:
            print('total cost',cost)
            return generate_path(final_dict,node),final_dict

        for action in [move_east,move_north,move_south,move_west,
                       move_southeast,move_northeast,move_northwest,move_southwest]:
            new_node,c2c=action(node[0],node[1],cost) 
            # print(new_node,'nn',action)
            if new_node not in final_list and new_node is not None:
                if new_node not in explored_list:
                    heapq.heappush(initial_list,(c2c,new_node))
                    explored_list.add(new_node)
                    final_dict.update({new_node:node})
                else:
                    # print("repeated")
                    for i in range(len(initial_list)):
                        if initial_list[i][1]==new_node:
                            if initial_list[i][0]>c2c:
                                initial_list[i]=(c2c,new_node)
                                final_dict.update({new_node:node})
    
def visualize_map(final_dict):
    for i in final_dict.keys():
        cv.circle(map,(i[1],250-i[0]),1,(79,79,79),-1)
        cv.imshow('map',map)
        cv.waitKey(1)

    for i in range(len(path)):
        cv.circle(map,(path[i][1],250-path[i][0]),1,(255,255,255),-1)
        cv.imshow('map',map)
        cv.waitKey(1)


                        
def validnode(Start_x,Start_y):
    i=Start_x
    j=Start_y
    if map[i][j]==255:
        return True
    else:
        return False

print(map.shape)
path,final_dict=dijkstra(Startnode,Goalnode)
print(path)
visualize_map(final_dict)
cv.imshow('map',map)
cv.waitKey(0)
e_time=time.time()
print('Time: ',e_time-s_time)
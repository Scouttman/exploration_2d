#!/usr/bin/env python3
import os
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.etree import ElementTree
from xml.dom import minidom
import numpy as np
import random

e_length = 10
e_width = 10
seg_length = 1
w_length = seg_length*e_length
w_width = seg_length*e_width
grid = np.zeros([w_length,w_width],dtype=int)
visited = np.zeros([w_length,w_width],dtype=int)

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def addWall(model,x,y,r,length,i):
    name = "Wall_%d"%i
    pose = "%f %f 0 0 0 %f"%(x,y,r)
    # wite to xml
    link = SubElement(model, 'link',{"name":name})
    #collsion
    coll = SubElement(link, 'collision',{"name":(name+"_Collision")})
    geom = SubElement(coll, 'geometry')
    box = SubElement(geom,"box")
    sizeEle = SubElement(box,"size")
    sizeEle.text = "%f 0.15 0.5"%length
    poseEle = SubElement(coll,"pose",{"frame":""})
    poseEle.text = "0 0 0.25 0 0 0" # position
    # visual
    visual = SubElement(link, 'visual',{"name":(name+"_Visual")})
    geom = SubElement(visual, 'geometry')
    box = SubElement(geom,"box")
    sizeEle = SubElement(box,"size")
    sizeEle.text = "%f 0.15 0.5"%length
    poseEle = SubElement(visual,"pose",{"frame":""})
    poseEle.text = "0 0 0.25 0 -0 0" # position
    mat = SubElement(visual, 'material')
    script = SubElement(mat,"script")
    uri = SubElement(script,"uri")
    nameEle = SubElement(script,"name")
    uri.text = "file://media/materials/scripts/gazebo.material"
    nameEle.text = "Gazebo/Wood"
    ambient = SubElement(mat,"ambient")
    ambient.text = "1 1 1 1"
    poseEle = SubElement(link,"pose",{"frame":""})
    poseEle.text = pose # position

def addCylinder(model,x,y,radius,i):
    length = 0.25
    name = "Obstical_%d"%i
    pose = "%f %f 0 0 0 0"%(x,y)
    # wite to xml
    link = SubElement(model, 'link',{"name":name})
    #collsion
    coll = SubElement(link, 'collision',{"name":(name+"_Collision")})
    geom = SubElement(coll, 'geometry')
    cylinder = SubElement(geom,"cylinder")
    radiusEle = SubElement(cylinder,"radius")
    radiusEle.text = "%f"%radius
    lengthEle = SubElement(cylinder,"length")
    lengthEle.text = "%f"%length
    # visual
    visual = SubElement(link, 'visual',{"name":(name+"_Visual")})
    geom = SubElement(visual, 'geometry')
    cylinder = SubElement(geom,"cylinder")
    radiusEle = SubElement(cylinder,"radius")
    radiusEle.text = "%f"%radius
    lengthEle = SubElement(cylinder,"length")
    lengthEle.text = "%f"%length
    #mat = SubElement(visual, 'material')
    #script = SubElement(mat,"script")
    #uri = SubElement(script,"uri")
    #nameEle = SubElement(script,"name")
    #uri.text = "file://media/materials/scripts/gazebo.material"
    #nameEle.text = "Gazebo/Wood"
    #ambient = SubElement(mat,"ambient")
    #ambient.text = "1 1 1 1"
    poseEle = SubElement(link,"pose",{"frame":""})
    poseEle.text = pose # position

def getSurround(pos):
    U = 0b0001
    L = 0b0010
    D = 0b0100
    R = 0b1000
    V = 0b10000 #visted
    eles = []
    #grid[pos[0],pos[1]] += U
    #if(grid[pos[0]-1,pos[1]] == 0 or grid[pos[0]-1,pos[1]] == U):
    #    grid[pos[0]-1,pos[1]] += D
    #up ?
    if(visited[pos[0],pos[1]]==0):
        if(pos[0]-1>0):
            if(grid[pos[0]-1,pos[1]]==0):
                eles.append([pos[0]-1,pos[1]])
                if(grid[pos[0],pos[1]]&U):
                    print("WTF")
                grid[pos[0],pos[1]] += U
                grid[pos[0]-1,pos[1]] += D
        # down 
        if(pos[0]+1<w_length):
            if(grid[pos[0]+1,pos[1]]==0):
                eles.append([pos[0]+1,pos[1]])
                if(grid[pos[0],pos[1]]&D):
                    print("WTF")
                grid[pos[0],pos[1]] += D
                grid[pos[0]+1,pos[1]] += U
        # left 
        if(pos[1]-1>0):
            if(grid[pos[0],pos[1]-1]==0):
                eles.append([pos[0],pos[1]-1])
                if(grid[pos[0],pos[1]]&L):
                    print("WTF")
                grid[pos[0],pos[1]] += L
                grid[pos[0],pos[1]-1] += R
        # right
        if(pos[1]+1<w_width):
            if(grid[pos[0],pos[1]+1]==0):
                eles.append([pos[0],pos[1]+1])
                if(grid[pos[0],pos[1]]&R):
                    print("WTF")
                grid[pos[0],pos[1]] += R
                grid[pos[0],pos[1]+1] += L
    else:
        print("ERROR:Been here before")
    return eles

        

root = Element('sdf')
root.set('version','1.6')
comment = Comment('Python Testing')
root.append(comment)
model = SubElement(root,'model')
model.set("name","random")
global_pose = SubElement(model, 'pose',{"frame":""})
global_pose.text = "0 0 0 0 0 0"




obs = False
if(obs):
    # build bounding box
    perimeter = 10
    elements = [[0,perimeter/2,0],[perimeter/2,0,np.pi/2],[0,-perimeter/2,np.pi],[-perimeter/2,0,3*np.pi/2]]
    print("Build walls")
    for i, ele in enumerate(elements):
        addWall(model,ele[0],ele[1],ele[2],perimeter,i)
    # place internal components
    length = 0.5
    min_radius = 0.12#0.12
    max_radius = 1
    gap = 0.5 # for robot to go through
    numOfObs = 30
    poss = np.zeros([numOfObs,3]) # note this will cause 0,0 to be avoided
    print("Placing Objects")
    for i in range(numOfObs):
        print(i)
        success = False
        for e in range(10):
            radius = random.uniform(min_radius,max_radius)
            x = random.uniform(-perimeter/2+radius, perimeter/2-radius)
            y = random.uniform(-perimeter/2+radius, perimeter/2-radius)
            ranges = np.hypot(poss[:,0]-x,poss[:,1]-y)-poss[:,2]   
            # check for collision with other obsticles
            if(ranges.min()>(radius+gap)):
                # Keep away from edges
                #if(x-gap-radius>perimeter/2 and x+gap+radius<perimeter/2 and y-gap-radius>perimeter/2 and y+gap+radius<perimeter/2):
                success = True
                break
        if(success):
            addCylinder(model,x,y,radius,i)
            poss[i,:] = [x,y,radius]
        else:
            print("ERROR: Unable to place")

maze = True
if(maze):
    toVisit = [[0,0]]
    i = 0
    while len(toVisit)>0:
        #random.shuffle(toVisit)
        pos = toVisit.pop()
        eles = getSurround(pos)
        visited[pos[0],pos[1]] = i
        if(len(eles)>0):
            random.shuffle(eles)
            for e in eles:
                toVisit.append(e)
                #toVisit.insert(0,e)
        else:
            print("%d dead end"%(i))
        i+=1
    if(i != w_length*w_length):
        print("ERROR %d != %d"%(i,w_length*w_length))
    print(grid)
    print(visited)
    U = 0b0001
    L = 0b0010
    D = 0b0100
    R = 0b1000
    l = 1
    ll = l #-0.2
    
    # this apears to work 
    for int_x in range(e_length):
        for int_y in range(e_width):
            count = (int_x*e_width+int_y)*4+4
            val = grid[int_x,int_y]
            x = int_x*seg_length+0.5
            y = int_y*seg_length+0.5   
            #addCylinder(model,x-w_length/2,y-w_width/2,0.1,count)
            numWalls = 0
            if(not val&D):
                addWall(model,x+l/2-w_length/2,y-w_width/2,np.pi/2,l,count)
                numWalls += 1
            if(not val&U):
                addWall(model,x-l/2-w_length/2,y-w_width/2,np.pi/2,l,count+1)
                numWalls += 1
            if(not val&R):
                addWall(model,x-w_length/2,y+l/2-w_width/2,0,ll,count+2)
                numWalls += 1
            if(not val&L):
                addWall(model,x-w_length/2,y-l/2-w_width/2,0,ll,count+3)
                numWalls += 1
            if(numWalls==4):
                print("ERROR: did not visit cell %d,%d"%(x,y))
            if(numWalls==0):
                print("ERROR: no walls for cell %d,%d"%(x,y))
    

#print(prettify(root).replace('"', "'"))
save_path = "/home/scouttman/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_maze"
modelFile = os.path.join(save_path, "model.sdf")
print(modelFile)
f = open(modelFile,"w+")
f.write(prettify(root).replace('"', "'"))
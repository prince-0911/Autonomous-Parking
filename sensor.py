
import pybullet as p
import time
import numpy as np
import pybullet_data
import math
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
c= [0,0,0.1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#obs1 = p.loadURDF("r2d2.urdf",[2,0,0.5], cubeStartOrientation)
#obs2= p.loadURDF("r2d2.urdf",[0,2,0.5], cubeStartOrientation)
#obs3 = p.loadURDF("r2d2.urdf",[2,2,0.5], cubeStartOrientation)
obs4 = p.loadURDF("r2d2.urdf",[-2,0,0.5], cubeStartOrientation)
#obs5 = p.loadURDF("r2d2.urdf",[2*math.cos(math.pi/6),0+2*math.sin(math.pi/6),0.5], cubeStartOrientation)
car=p.loadURDF("husky/husky.urdf",c,cubeStartOrientation)

numjoints=p.getNumJoints(car)
#for j in range(numjoints):
    #print(p.getJointInfo(car,j))
value = 8 #rad/s
maxForce = 100 #Newton

#print (position)
def fun(pos1,pos2,pos3,thetha):
    r=5
    x=["x1","x2","x3","x4","x5","x6","x7","x8","x9","x10","x11","x12"]
    t=[0 for i in range(12)]
    dist=np.zeros(12)
    for i in range(12):
        x[i]=p.rayTest([pos1,pos2,pos3+0.7],[pos1+r*math.cos(math.pi*i/6+thetha),pos2+r*math.sin(math.pi*i/6+thetha),pos3+0.7])
        #print(x[i])
        x[i]=x[i][0]
        x[i]=list(x[i])
        #print(x[i])
        t[i]=list(x[i][3])
        #print(t[i])
        #p.addUserDebugLine([pos1,pos2,pos3+0.2],[pos1+r*math.cos(0+math.pi*i/6),pos2+r*math.sin(0+math.pi*i/6),pos3],[1,0,1])
        if(x[i][0]==-1 or 0):
            #p.addUserDebugLine([pos1,pos2,pos3+0.7],[pos1+r*math.cos(0+math.pi*i/6+thetha),pos2+r*math.sin(0+math.pi*i/6+thetha),pos3],[1,0,1])
            dist[i]=1000
            #print(dist[i])
        else:
            #p.addUserDebugLine([pos1,pos2,pos3+0.7],t[i],[1,0,0])
            dist[i]=np.sqrt(pow(t[i][0]-pos1,2)+pow(t[i][1]-pos2,2)+pow(t[i][2]-pos3,2))
    return dist       
     
def forward():
    targetVel = value
    for joint in range(2, 6):
        p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
        p.stepSimulation()
def backward():
    targetVel = -value
    for joint in range(2, 6):
        p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
        p.stepSimulation()
def left():
    targetVel=value
    for joint in range(2, 6):
        if joint%2!=0:
            p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()
def right():
    targetVel=value
    for joint in range(2, 6):
        if joint%2==0:
            p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()
n=1  
pos3=[0,0,0]
thetha=0
while(n>0):
    pos=p.getBasePositionAndOrientation(car)
    pos1=pos[0]
    dist=np.zeros(12)
    pos2=p.getEulerFromQuaternion(pos[1])
    thetha=thetha+pos2[2]-pos3[2]
    pos3=pos2
    dist=fun(pos1[0],pos1[1],pos1[2],thetha)
    if(n==2):
        print(thetha)
        print(dist)
    if(n%500==0):
        print(thetha)
        print(dist)
    
    keys = p.getKeyboardEvents()
    
    for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = value
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
           
            p.stepSimulation()
            #time.sleep(1./240.)
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
            #time.sleep(1./240.)
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = -value
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
            #time.sleep(1./240.)

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
            #time.sleep(1./240.)

        if(k== p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel=value
            for joint in range(2, 6):
                if joint%2==0:
                    p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()
            #time.sleep(1./240.)

        if(k== p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel=0
            for joint in range(2, 6):
                if joint%2==0:
                    p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()
            #time.sleep(1./240.)

        if(k== p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel=value
            for joint in range(2, 6):
                if joint%2!=0:
                    p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()
            #time.sleep(1./240.)

        if(k== p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel=0
            for joint in range(2, 6):
                if joint%2!=0:
                    p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()
            #time.sleep(1./240.)
    p.stepSimulation()
    n=n+1
p.disconnect()
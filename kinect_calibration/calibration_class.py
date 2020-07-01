#!/usr/bin/env python

import sys
import cv2
import math
import numpy as np
from numpy import *
from numpy.linalg import inv, qr,det
from math import sqrt




IMG_COL=960
IMG_ROW=540

x_temp=0.376881479525+0.376058926447+0.375573234153+0.377304446614
print("11111111111111111",x_temp/4)
y_temp=-0.24092599437-0.239538246504-0.240592261432-0.241169592054
print("2222222222222222222222222",y_temp/4)
z_temp=-0.408356295716-0.411946083386-0.408041069928-0.409714356289
print("33333333333333333333",z_temp/4)
rx_temp=-0.0029812649842-0.00377016278047-0.0033301030781 -0.00295350192309
print("4444444444444444444444444",rx_temp/4)
ry_temp=-0.138667835363-0.137850204936-0.138141149828-0.139093099171
print("55555555555555555555555555",ry_temp/4)
rz_temp=-0.990311378571-0.990428310334-0.990394708333-0.990255442752
print("66666666666666666",rz_temp/4)
rw_temp=0.00676143242007+0.0059049946062+0.00492492070344+0.00620843687553
print("7777777777777",rw_temp/4)

class point_transformation:

    def __init__(self):
        self.position=0
        self.Extra_Params_Quat=[np.array([ -0.0007117477497294999,-0.1392747698175,-0.9902379727733334, 0.004664718343659999]),
                                    np.array([-0.0032587581914649997,-0.1384380723245,-0.9903474599975, 0.00594994615131]),
                                    np.array([0,0,0, 0]),
                                    np.array([0,0,0, 0]),
                                    np.array([0,0,0, 0])]
        self.Extra_Params_Translation=[np.array([[-0.025292478616583333],[-0.24338054059199998],[-0.41234705825633333]]),
                                    np.array([[0.37645452168475],[-0.24055652359],[-0.40951445132975]]),
                                    np.array([[0.0],[-0.0],[0]]),
                                    np.array([[0.0],[-0.0],[0]]),
                                    np.array([[0.0],[-0.0],[0]])]

        

        self.Quat=np.array([0,0,0,0])
        self.Translation=np.array([[0.0],[-0.0],[0]])
        self.Camera_Internal_Mat=np.array([[528.1534105235548, 0.0, 475.32604300255264],[0.0, 527.6570775979337, 262.17997301714524]
                        ,[0.000000, 0.000000, 1.000000]])
    def quat_to_Rot(self):
        #print(self.Quat)
        qx=self.Quat[0]
        qy=self.Quat[1]
        qz=self.Quat[2]
        qw=self.Quat[3]
        rot=np.zeros([3,3])
        rot[0,0]=1-2*qy*qy-2*qz*qz
        rot[0,1]=2*qx*qy - 2*qz*qw
        rot[0,2]=2*qx*qz + 2*qy*qw
        rot[1,0]=2*qx*qy + 2*qz*qw
        rot[1,1]=1 - 2*qx*qx - 2*qz*qz
        rot[1,2]=2*qy*qz - 2*qx*qw
        rot[2,0]=2*qx*qz - 2*qy*qw
        rot[2,1]=2*qy*qz + 2*qx*qw
        rot[2,2]=1 - 2*qx*qx - 2*qy*qy 

        return rot


    def load_position(self,position):
        self.Quat=self.Extra_Params_Quat[position]
        #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",position)
       # print("Quat",self.Quat)
        self.Translation=self.Extra_Params_Translation[position]
    
    def find_nonzeros_pixelvalue(self,img,v,u):
        count=0
        sums=0
        if (v>IMG_ROW-2)|(u>IMG_COL-2)|(u<2)|(v<2):
            print(u)
            print(v)
            print("out img range")
            return(0)
        else:
            if (img[v,u]!=0):
                return(img[v,u])  
                print(img[v,u])
            else:
                for i in range(-1,2,1):
                    for j in range(-1,2,1):
                        if img[v+i,u+j]!=0:
                            count=count+1
                            sums=sums+img[v+i,u+j]
                if count==0:
                    print("3*3 pixels all zeros,bad point")
                    return(0)
                else:
                    return(sums/count)     
     
    def Pix2baselink(self,depth_img,u,v):
       # print("x,y",x,y)


        u=int(u)
        v=int(v)


        if u<0:
            u=0
        if u>IMG_COL-1:
            u=IMG_COL-1
        if v<0:
            v=0
        if v>IMG_ROW-1:
            v=IMG_ROW-1
        
        #print(self.depth_img)
        pixposition=np.ones((3,1))
        pixposition[0]=u
        pixposition[1]=v

        Camera_External_RotMat  = self.quat_to_Rot()
        baselink2base=np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        #s=self.find_nonzeros_pixelvalue(depth_img,v,u)
        s=depth_img[v,u]*0.001 
        if s==0:
            s=1.01
        print("sssssssssssssssssss",s)
        Camera_Mat_inv=np.linalg.inv(self.Camera_Internal_Mat)
        cameraPoint=(s*Camera_Mat_inv).dot(pixposition)
        worldPoint_3d = Camera_External_RotMat.dot(cameraPoint)+self.Translation
        worldPoint_3d=baselink2base.dot(worldPoint_3d)
       # print(",,,worldPoint_3d,,,,,,",worldPoint_3d)
        return worldPoint_3d[0],worldPoint_3d[1],worldPoint_3d[2]



    def rpy2rotvec(Rx,Ry,Rz):
        Txyz=np.array([[math.cos(Rz)*math.cos(Ry),math.cos(Rz)*math.sin(Ry)*math.sin(Rx)-math.sin(Rz)*math.cos(Rx),math.cos(Rz)*math.sin(Ry)*math.cos(Rx)+math.sin(Rz)*math.sin(Rx)],[math.sin(Rz)*math.cos(Ry),math.sin(Rz)*math.sin(Ry)*math.sin(Rx)+math.cos(Rz)*math.cos(Rx),math.sin(Rz)*math.sin(Ry)*math.cos(Rx)-math.cos(Rz)*math.sin(Rx)],[-math.sin(Ry),math.cos(Ry)*math.sin(Rx),math.cos(Ry)*math.cos(Rx)]])
        p=math.acos((Txyz[0,0]+Txyz[1,1]+Txyz[2,2]-1)/2)
        kx=(1/(2*math.sin(p)))*(Txyz[2,1]-Txyz[1,2])*p
        ky=(1/(2*math.sin(p)))*(Txyz[0,2]-Txyz[2,0])*p
        kz=(1/(2*math.sin(p)))*(Txyz[1,0]-Txyz[0,1])*p

        return kx,ky,kz

if __name__ == '__main__':
    
    calibration = point_transformation()

    #out.release()
    cv2.destroyAllWindows()

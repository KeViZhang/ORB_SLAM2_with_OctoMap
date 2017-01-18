#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d
import argparse

def plot_traj(ax,f,axis,style,color,label):
    x = []
    y = []
    for line in f:
        if line[0] == '#':
            continue
        data = line.split()
        x.append( float(data[0] ) )
        y.append( float(data[axis] ) )
    ax.plot(x,y,style,color=color,label=label)

parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
parser.add_argument('first_file', help='camera trajectory (format: timestamp tx ty tz qx qy qz qw)')
parser.add_argument('second_file', help='MAVpose trajectory (format: timestamp tx ty tz qx qy qz qw)')
args = parser.parse_args()

dpi = 300;


for i in range(1,4):
    f_Cam = open(args.first_file)
    f_MAV = open(args.second_file)
    fig = plt.figure()
    ax = plt.subplot( 111 )
    plot_traj(ax,f_MAV,i,'-',"blue","MAV_" + chr(119+i))
    plot_traj(ax,f_Cam,i,'-',"red", "Cam_" + chr(119+i))
    ax.legend()
    ax.set_xlabel('t [s]')
    ax.set_ylabel(chr(119+i)+' [m]')
    plt.savefig(str(i)+'-t' + chr(119+i), dpi = dpi)
    plt.show()

f_MAV = open(args.second_file)
x = []
y = []
z = []
for line in f_MAV:
    if line[0] == '#':
        continue
    data = line.split()
    x.append( float(data[1] ) )
    y.append( float(data[2] ) )
    z.append( float(data[3] ) )

fig = plt.figure()
ax = plt.subplot( 111 )
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.plot(x,y)
plt.savefig('4-xy',dpi = dpi)
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.plot(x,y,z)
plt.savefig('5-xyz',dpi = dpi)
plt.show()


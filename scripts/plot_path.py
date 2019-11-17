#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import csv
import sys


#ustructued max = 34355
max_cells = 35000/100

if  len(sys.argv) == 2:
    log_file_str = sys.argv[1]
else:
    log_file_str = "log.csv"

history = np.loadtxt(open(log_file_str, "rb"), delimiter=",", skiprows=1)
history[history[:,3] == 0,3 ] = np.nan

tmp = [history[:,3] == np.nan]
#print("nan at {}".format(tmp))
ignor_upto = 0
history = history[ignor_upto:,:]

#print(history)
limit = np.argmax(history[:,3]>(max_cells*95))
#limit = history.shape[0]-6
print(limit)

def calc_dis(path):
    return np.sum(np.hypot(path[:,0],path[:,1]))

img = plt.imread("/home/scouttman/catkin_ws/src/exploration_2d/scripts/obs_background.png")
img = plt.imread("/home/scouttman/catkin_ws/src/exploration_2d/scripts/background_old.png")

show_exploration_rate = True
if(show_exploration_rate):
    plt.plot((history[:limit,0]-history[0,0])/60, history[:limit,3]/max_cells, 'o-',zorder=1)
    axes = plt.gca()
    axes.set_ylabel('explored (%)')
    axes.set_xlabel('time (min)')
    plt.show()

#plt.subplot(2, 1, 1)
no_sub = True
fig, ax = plt.subplots()
if(not no_sub):
    plt.subplots_adjust(left=0.25, bottom=0.25)
nudge = 0.5
shift = [0,0]
limits = [-5-nudge-shift[0],5+nudge-shift[0],-5-nudge+shift[1],5+nudge+shift[1]] #[min(history[:,1])-nudge,max(history[:,2])+nudge,min(history[:,2])-nudge,max(history[:,2])+nudge]
maxArg =  history.shape[0]-1
ax.imshow(img, zorder=0, extent=limits)
dis = calc_dis(history[:limit,1:3])
title = plt.title('Exploration time {:.1f}min(s) {:.2f}m explored {:.0f}%'.format((history[limit,0]-history[0,0])/60,dis,history[limit,3]/max_cells))
l, = plt.plot(- history[:limit,2], history[:limit,1], 'o-',zorder=1)
axes = plt.gca()
axes.set_xlim([-5,5])
axes.set_ylim([-5,5])

if(not no_sub):
    axcolor = 'lightgoldenrodyellow'
    ahis = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
    shis = Slider(ahis, 'his', 1, limit, valinit=limit)



def update(val):
    limit = int(shis.val)
    if(limit>maxArg):
        limit = maxArg
    l.set_xdata(-history[:limit,2])
    l.set_ydata(history[:limit,1])
    fig.canvas.draw_idle()
    dis = calc_dis(history[:limit,1:3])
    title.set_text(u'Exploration time {:.1f}min(s) {:.2f}m explored {:.0f}%'.format((history[limit,0]-history[0,0])/60,dis,history[limit,3]/max_cells))

if(not no_sub):
    shis.on_changed(update)

plt.show()
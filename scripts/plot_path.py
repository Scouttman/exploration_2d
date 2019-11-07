#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import csv


history = np.loadtxt(open("log_3.txt", "rb"), delimiter=",", skiprows=1)
limit = 10


img = plt.imread("scripts/obs_background.png")

#plt.subplot(2, 1, 1)
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)
nudge = 0.5
limits = [min(history[:,1])-nudge,max(history[:,2])+nudge,min(history[:,2])-nudge,max(history[:,2])+nudge]
ax.imshow(img, zorder=0, extent=limits)
l, = plt.plot(history[:limit,1], history[:limit,2], 'o-',zorder=1)

axcolor = 'lightgoldenrodyellow'
ahis = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
shis = Slider(ahis, 'his', 1, 30.0, valinit=limit)


def update(val):
    limit = int(shis.val)
    l.set_xdata(history[:limit,1])
    l.set_ydata(history[:limit,2])
    fig.canvas.draw_idle()

shis.on_changed(update)

plt.show()
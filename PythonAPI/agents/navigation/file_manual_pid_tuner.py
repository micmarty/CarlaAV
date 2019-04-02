import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons

fig, ax = plt.subplots()
# plt.subplots_adjust(left=0.25, bottom=0.25)
# t = np.arange(0.0, 1.0, 0.001)
# a0 = 5
# f0 = 3
# delta_f = 5.0
# s = a0*np.sin(2*np.pi*f0*t)
# l, = plt.plot(t, s, lw=2, color='red')
# plt.axis([0, 1, -10, 10])

axcolor = 'lightgoldenrodyellow'
axp = plt.axes([0.35, 0.50, 0.35, 0.03], facecolor=axcolor)
axd = plt.axes([0.35, 0.40, 0.35, 0.03], facecolor=axcolor)
axi = plt.axes([0.35, 0.30, 0.35, 0.03], facecolor=axcolor)

import json
# try:
#     with open('/home/miczi/simulators/094/pid.txt', 'r') as json_file:
#         data = json.load(json_file)
#         P, I, D = data['P'], data['I'], data['D']
# except FileNotFoundError:
#     pass


slider_P = Slider(axp, 'Proportional', 0.0, 1.0, valinit=1.0, valstep=0.005)
slider_D = Slider(axd, 'Derivative', 0.0, 1.0, valinit=0.0, valstep=0.005)
slider_I = Slider(axi, 'Integral', 0.0, 1.0, valinit=1.0, valstep=0.005)


def update(val):
    with open('/home/miczi/simulators/094/pid.txt', 'w') as json_file:
        json.dump({
            'P': slider_P.val,
            'I': slider_I.val,
            'D': slider_D.val
        }, json_file)
    fig.canvas.draw_idle()

slider_P.on_changed(update)
slider_I.on_changed(update)
slider_D.on_changed(update)
#resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
#button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')


# def reset(event):
#     sfreq.reset()
#     samp.reset()
# button.on_clicked(reset)

# rax = plt.axes([0.025, 0.5, 0.15, 0.15], facecolor=axcolor)
# radio = RadioButtons(rax, ('red', 'blue', 'green'), active=0)


# def colorfunc(label):
#     l.set_color(label)
#     fig.canvas.draw_idle()
# radio.on_clicked(colorfunc)

plt.show()
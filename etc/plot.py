import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('output.csv',delimiter=',', dtype = float)
bg_color = 'black'
fg_color = 'white'

a = [row[0] for row in data]
b = [row[1] for row in data]
c = [row[2] for row in data]
d = [row[3] for row in data]
e = [row[4] for row in data]


# fig = plt.figure()
fig = plt.figure(facecolor=bg_color, edgecolor=fg_color)
axes = fig.add_subplot(111)
axes.patch.set_facecolor(bg_color)
axes.xaxis.set_tick_params(color=fg_color, labelcolor=fg_color)
axes.yaxis.set_tick_params(color=fg_color, labelcolor=fg_color)
# for spine in axes.spines.values():
#     spine.set_color(fg_color)

plt.plot(a,fg_color)
plt.plot(b,fg_color)
plt.plot(c,fg_color)
plt.plot(d,fg_color)
plt.plot(e,'g',linewidth=4.0)



plt.show()
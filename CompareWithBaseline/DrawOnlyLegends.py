import matplotlib.pyplot as plt
from GlobalVariables import *
import seaborn as sns
from matplotlib.transforms import Bbox
import numpy as np

fig, ax = plt.subplots()
method_num = len(baseline_method_names)
element=1
row=method_num
col=5
lane_data =[[element] * col for _ in range(row)]

# Display the legend
ax.legend( loc='lower center', ncol=method_num)
ax.axis('off')

method_names = baseline_method_names
for i in range(method_num):
    splot = sns.lineplot( x=lane_data[i], y=lane_data[i], marker=marker_map[method_names[i]],
                         color=color_map[method_names[i]
                         ], label=baseline_method_labels[method_names[i]],
                         markersize=marker_size_map[method_names[i]], markeredgewidth=0)

plt.legend( loc='lower center',  ncol=method_num, fontsize=14)
# x0,x1 = 0.86124, 1.138
x0,x1 = 0.845, 1.155
y0,y1 = 0.9485, 0.9575
bbox = Bbox([[x0,y0],[x1,y1]])
bbox = bbox.transformed(ax.transData).transformed(fig.dpi_scale_trans.inverted())
plt.savefig(ROOT_CompareWithBaseline_PATH+"legends.pdf", format='pdf', bbox_inches=bbox)
plt.show(block=False)
plt.show()
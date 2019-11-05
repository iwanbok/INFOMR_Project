import matplotlib.pyplot as plt
import numpy as np
import os

'''
    For the given path, get the List of all files in the directory tree 
'''
def getListOfFiles(dirName):
    # create a list of file and sub directories 
    # names in the given directory 
    listOfFile = os.listdir(dirName)
    listOfFile.sort(reverse=True)
    allFiles = list()
    # Iterate over all the entries
    for entry in listOfFile:
        # Create full path
        fullPath = os.path.join(dirName, entry)
        # If entry is a directory then get the list of files in this directory 
        if os.path.isdir(fullPath):
            allFiles = allFiles + getListOfFiles(fullPath)
        else:
            allFiles.append(entry)
                
    return allFiles

tnse_points = np.loadtxt('data/tsne.txt')
classes = os.listdir('data/normalized')
meshes = getListOfFiles('data/normalized')
norm = plt.Normalize(0, 19)
c = np.fromfunction(lambda i: i / 20, (tnse_points.shape[0],))
c = np.floor(c)
cmap = plt.cm.tab20
fig, ax = plt.subplots()
sc = plt.scatter(tnse_points[:,0], tnse_points[:,1], s=20, c=c, cmap=cmap, norm=norm)
annot = ax.annotate("", xy=(0,0), xytext=(20,20),textcoords="offset points",
                    bbox=dict(boxstyle="round", fc="w"),
                    arrowprops=dict(arrowstyle="->"))

def update_annot(ind):
    index = ind['ind'][0]

    pos = sc.get_offsets()[index]
    annot.xy = pos
    text = "{}, {}".format(meshes[index], classes[int(index / 20)])
    annot.set_text(text)
    annot.get_bbox_patch().set_facecolor(cmap(norm(c[index])))
    annot.get_bbox_patch().set_alpha(0.4)

def hover(event):
    vis = annot.get_visible()
    if event.inaxes == ax:
        cont, ind = sc.contains(event)
        if cont:
            update_annot(ind)
            annot.set_visible(True)
            fig.canvas.draw_idle()
        else:
            if vis:
                annot.set_visible(False)
                fig.canvas.draw_idle()

fig.canvas.mpl_connect("motion_notify_event", hover)

plt.show()
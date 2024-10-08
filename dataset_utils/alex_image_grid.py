import tkinter
from tkinter import *
from PIL import Image, ImageTk
import pickle
import os
import glob
import time
import sys

import faulthandler

faulthandler.enable()

query = "street level view"

path = os.path.join(os.path.curdir, "dataset", query)
image_nums = list(os.walk(path))[0][2]

root = Tk()

toRemove = set()
im_nums = []
time_left = time.time()
def removeImage(i):
    global time_left
    time_left = time.time()
    def remove():
        imageLabels[i].config(text="Removed", image=None)
        global toRemove
        toRemove.add(im_nums[i])
        
    return remove

def chosen():
    global decided
    decided = True

rows = 4
cols = 4

imageLabels = [tkinter.Button(root, text="Removed", command = removeImage(i)) for i in range(rows*cols)]
for n,x in enumerate(imageLabels):
    x.grid(row=n//cols, column=n%cols)

viewed = 0
while viewed < len(image_nums):
    
    im_nums = image_nums[viewed:viewed+16]
    root.update()
    toRemove.clear()
    images = []
    
    for j, imgname in enumerate(im_nums):
        if imgname == '.DS_Store':
            continue
        name = os.path.join(os.path.curdir,
                            'dataset', query, imgname)
        image = Image.open(name)
        temp = ImageTk.PhotoImage(image)
        images.append(temp)
        imageLabels[j].config(text = query, image=temp)
        image.close()
    
    decided = False
    while not decided:
        time.sleep(1/30)
        root.update()
        if time.time() >= time_left + 5:
            chosen()
            time_left = time.time()
        
    for imgname in toRemove:
        name = os.path.join(os.path.curdir,
                            'dataset', query, imgname)
        os.remove(name)
        
    viewed+=16

root.destroy()

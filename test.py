from tkinter import S
import cv2
import numpy as np


a = [[1,2],[3,4],[15,6],[4,8],[10,12]]

b= np.gradient(a,axis = 0)

a = np.array(a)
a1 =  a[:,0]
b1 = np.gradient(a1)
print(b1)
print(5**2)
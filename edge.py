import cv2
import numpy as np

vessel = cv2.imread('vessel_mask.jpg')
vessel = cv2.cvtColor(vessel, cv2.COLOR_BGR2GRAY)

catheter

_, catheter_contour, hierarchy = cv2.findContours(vessel,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

from asyncore import write
from tkinter import S
from turtle import position
import cv2
import numpy as np
import csv
import os

def findIntersection(point, contour, radius, height, length):
    blank = np.zeros_like(contour)
    cv2.circle(blank, (point[1], point[0]), radius, 255, -1) 
    
    intersection = []
    for i in range(np.max([0,point[0] - radius]), np.min([height, point[0] + radius])):
        for j in range(np.max([0,point[1] - radius]), np.min([length, point[1] + radius])):
            if(blank[i][j] ==255 and contour[i][j] ==255):
                intersection.append([i,j])
                contour[i][j] =0
    return intersection, contour

def furthestPoint(point, edges):
    distances = []
    for i in edges:
        distances.append((point[0] - i[0]) ** 2 + (point[1]- i[1]) ** 2)
    
    index = np.argmax(distances)
    return edges[index]

def contactPoint(catheter_point, contour_image, threshold):
    height = np.shape(contour_image)[0]
    length = np.shape(contour_image)[1]
    contact_points = []
    distance_to_vessel = []
    contact_index = []
    for point in catheter_point:
        blank = np.zeros((height, length))
        radius = 2
        contact = False
        while True:
            if radius >=20 or contact:
                break
            cv2.circle(blank, (point[1], point[0]), radius, 255, -1) 
            for i in range(np.max([0,point[0] - radius]), np.min([height, point[0] + radius])):
                for j in range(np.max([0,point[1] - radius]), np.min([length, point[1] + radius])):
                    if(blank[i][j] == 255 and contour_image[i][j] == 255):
                        contact = True
            radius +=1
        distance_to_vessel.append(radius)
        print(radius)

    # print(np.shape(points))
    # print(np.shape(distance_to_vessel))

    for i in range(np.shape(distance_to_vessel)[0]):
        if(distance_to_vessel[i] <=threshold):
            contact_points.append(catheter_point[i])
            contact_index.append(1)
        else:
            contact_index.append(0)
    
    print(np.shape(contact_index))
    print(contact_index)
    return contact_points, contact_index

def curvature(points):
    points_m = np.array(points)
    x = points_m[:, 0]
    y = points_m[:, 1]
    
    x_t = np.gradient(x)
    y_t = np.gradient(y)
    xx_t = np.gradient(x_t)
    yy_t = np.gradient(y_t)
    
    curvature_list = np.abs(xx_t * y_t - x_t * yy_t)/(x_t * x_t + y_t * y_t) ** 1.5
    return curvature_list

def catheterExtraction(filename, mask, threshold):
    insert = cv2.imread(filename)
    insert_mask = cv2.bitwise_and(insert, mask)
    
    height = np.shape(insert_mask)[0]
    length = np.shape(insert_mask)[1]
    
    for i in range(height):
        for j in range(length):
            if(insert_mask[i][j][2] < threshold):
                insert_mask[i][j] = [0,0,0]
    
    insert_mask = cv2.cvtColor(insert_mask, cv2.COLOR_BGR2GRAY)
    
    kernel = np.ones((2,5),np.uint8)
    opening = cv2.morphologyEx(insert_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    rec, img2 = cv2.threshold(opening, 20,255, cv2.THRESH_BINARY)
    _, catheter_contour, hierarchy = cv2.findContours(img2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    index = 0
    size = 0
    for i in range(np.shape(catheter_contour)[0]) :
        if np.size(catheter_contour[i]) > size:
            index  = i
            size = np.size(catheter_contour[i])

    contour = np.zeros_like(insert_mask)
    cv2.drawContours(contour, catheter_contour, index, (255,255,255), -1)
    contour=cv2.ximgproc.thinning(contour)
    return contour
    
def catheterPoints(contour, mask, radius):
    height = np.shape(contour)[0]
    length = np.shape(contour)[1]
    contour[:][0] = 0
    for i in reversed(range(np.shape(contour)[0])):
        if contour[i][1] == 255:
            start = [i,1]
            break     

    next_point = start
    sections = []
    points = []
    points.append(start)
    
    while True:
        next_edge, contour = findIntersection(next_point, contour, radius, height, length)
        if next_edge ==[]:
            break
        next_point = furthestPoint(next_point, next_edge)
        sections.append(next_edge)
        points.append(next_point)
        print(next_point)

    return points

def saveData(filename):    
    mask =cv2.imread('vessel_mask.jpg')
    # filename = 'insert4.jpg'
    contour= catheterExtraction(filename, mask, threshold=75)

    points = catheterPoints(contour, mask, radius=5)
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    mask = 255 - mask
    contacted_point, contacted_point_index = contactPoint(points, mask,10)
    curvatures = curvature(points)
    insert = cv2.imread(filename)

    for i in points:
        insert[i[0]][i[1]] = [0,0,0]

    for i in contacted_point:
        insert[i[0]][i[1]] = [255,255,255]


    print(points)
    print(curvatures)
    print(contacted_point_index)

    position = np.asarray(points)
    x_position = position[:,0]
    y_position = position[:,1]

    print(x_position)

    return x_position, y_position, curvatures, insert





with open('x_position.csv', 'a') as x_list:
    write_x = csv.writer(x_list)
with open('y_position.csv', 'a') as y_list:
    write_y = csv.writer(y_list)
with open('curvature.csv', 'a') as c_list:
    write_c = csv.writer(c_list)

x_positions = []
y_positions =[]
allCurvatures = []

folderPath  = '/Users/yifan_li/Library/Mobile Documents/com~apple~CloudDocs/Documents/Code/Python/CV/data1'
pic_list = os.listdir(folderPath)
pic_list = np.sort(pic_list)
for i in pic_list[2:10]:
    filename = folderPath + '/' + i
    print(filename)
    x,y,c,image = saveData(filename)
    x_positions.append(x)
    y_positions.append(x)
    allCurvatures.append(c)
    newImage_name = '/Users/yifan_li/Library/Mobile Documents/com~apple~CloudDocs/Documents/Code/Python/CV/modified_image/' + i
    print(newImage_name)
    cv2.imwrite(newImage_name, image)
    
    
    
with open('x_position.csv', 'a') as x_list:
    write_x = csv.writer(x_list)
    write_x.writerows(x_positions)
with open('y_position.csv', 'a') as y_list:
    write_y = csv.writer(y_list)
    write_y.writerows(y_positions)
with open('curvature.csv', 'a') as c_list:
    write_c = csv.writer(c_list)
    write_c.writerows(allCurvatures)


    













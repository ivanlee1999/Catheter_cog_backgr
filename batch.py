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
    contact_list = np.zeros_like(catheter_point)
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
    contour = cv2.GaussianBlur(contour, (5,5), cv2.BORDER_DEFAULT)
    contour=cv2.ximgproc.thinning(contour)
    return contour
    
def catheterPoints(contour, mask, radius):
    height = np.shape(contour)[0]
    length = np.shape(contour)[1]
    contour[:,0] = 0
    start = [0,0]
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


def curvature(originalPoint):
    num = np.shape(originalPoint)[0]
    sample = np.arange(0,num,4)
    originalPoint = np.asarray(originalPoint)
    # sampllPoints = originalPoint[sample]
    sampllPoints = originalPoint
    # print(sampllPoints)
    # print(sampllPoints)
    print(originalPoint[:,0])
    fitCurve = np.polyfit(sampllPoints[:,1], sampllPoints[:,0], 10)
    fitEqu = np.poly1d(fitCurve)
    fit_fd = fitEqu.deriv()    #first derivative
    fit_sd = fit_fd.deriv()
    y_fd = fit_fd(originalPoint[:, 1])
    y_sd = fit_sd(originalPoint[:, 1])
    curvatures = np.abs(y_sd)/ ((1 + y_fd**2))**1.5
    print(fitEqu(originalPoint[:,1]))
    print(curvatures)
    return curvatures

def saveData(filename):    
    mask =cv2.imread('vessel_mask.jpg')
    # filename = 'insert4.jpg'
    contour= catheterExtraction(filename, mask, threshold=80)

    points = catheterPoints(contour, mask, radius=5)
    curvatures = curvature(points) 
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    mask = 255 - mask
    contacted_point, contacted_point_index = contactPoint(points, mask,10)
    # curvatures = curvature(points)
    insert = cv2.imread(filename)

    for i in points:
        insert[i[0]][i[1]] = [0,0,0]

    for i in contacted_point:
        insert[i[0]][i[1]] = [255,255,255]


    print(points)
    # print(curvatures)
    print(contacted_point_index)

    position = np.asarray(points)
    x_position = position[:,0]
    y_position = position[:,1]

    print(x_position)

    return x_position, y_position, insert, contacted_point_index, curvatures





with open('x_position.csv', 'a') as x_list:
    write_x = csv.writer(x_list)
with open('y_position.csv', 'a') as y_list:
    write_y = csv.writer(y_list)
with open('curvature.csv', 'a') as c_list:
    write_c = csv.writer(c_list)

x_positions = []
y_positions =[]
allCurvatures = []

folderPath  = './images'
# a = cv2.imread('~/Workspace/prosilica_driver//prosilica/recorded/raw_2022-02-08-14-16-28')
pic_list = os.listdir(folderPath)
pic_list = np.sort(pic_list)
for i in pic_list[2:]:
    filename = folderPath + '/' + i
    print(filename)
    x,y,image,con, cur = saveData(filename)
    # x_positions.append(x)
    # y_positions.append(x)
    # allCurvatures.append(c)
    newImage_name = './modified_image/' + i
    print(newImage_name)
    cv2.imwrite(newImage_name, image)
    
    print(x)
    with open('x_position.csv', 'a') as x_list:
        write_x = csv.writer(x_list)
        write_x.writerow(x)
    with open('y_position.csv', 'a') as y_list:
        write_y = csv.writer(y_list)
        write_y.writerow(y)
    with open('contact.csv', 'a') as con_list:
        write_con = csv.writer(con_list)
        write_con.writerow(con)
    with open('curvature.csv', 'a') as cur_list:
        write_cur = csv.writer(cur_list)
        write_cur.writerow(cur)
    

    
    
# with open('x_position.csv', 'a') as x_list:
#     write_x = csv.writer(x_list)
#     write_x.writerows(x_positions)
# with open('y_position.csv', 'a') as y_list:
#     write_y = csv.writer(y_list)
#     write_y.writerows(y_positions)
# with open('curvature.csv', 'a') as c_list:
#     write_c = csv.writer(c_list)
#     write_c.writerows(allCurvatures)


    













from tkinter import S
import cv2
import numpy as np

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
    contact_points = []
    distance_to_vessel = []
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
            contact_points.append(points[i])
    
    return contact_points

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
    
    

mask =cv2.imread('vessel_mask.jpg')
insert = cv2.imread('insert3.jpg')
insert_mask = cv2.bitwise_and(insert, mask)

height = np.shape(insert_mask)[0]
length = np.shape(insert_mask)[1]

for i in range(height):
    for j in range(length):
        if(insert_mask[i][j][2] < 75):
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

# cv2.drawContours(insert, catheter_contour, index, (255,255,255), -1)

# cv2.imshow('contour', contour)
# cv2.waitKey(0)

thickness = 0
for i in range(height):
    if contour[i][300] ==255:
        thickness +=1
print('thickness of vessel' +  str(thickness))

contour=cv2.ximgproc.thinning(contour)
cv2.imshow('contour', contour)
cv2.waitKey(0)

# for i in range(height):
#     for j in range(length):
#         if contour[i][j] ==255:
#             insert[i][j] =[255, 255, 255]


# cv2.imwrite('thin.jpg', insert)

contour[:][0] = 0


for i in reversed(range(height)):
    if contour[i][1] == 255:
        start = [i,1]
        break       

radius = 5
next_point = start
sections = []
points = []
points.append(start)

while True:
    next_edge, contour = findIntersection(next_point, contour, radius, height, length)
    if next_edge ==[]:
        break
    next_point = furthestPoint(next_point, next_edge)
    # print(next_point)
    sections.append(next_edge)
    points.append(next_point)


# print(points)
# blank = np.zeros_like(insert_mask)
# for i in points:
#     insert[i[0]][i[1]] = [255, 255, 255]

mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
mask = 255 - mask
# cv2.imshow('image', mask)
# cv2.waitKey(0)


contacted_point = contactPoint(points, mask,10)
print(contacted_point)

for i in contacted_point:
    insert[i[0]][i[1]] = [255,255,255]


curv = curvature(points)

print(np.shape(curv))





cv2.imshow('image', insert)
cv2.waitKey(0)







# contact_points = []
# distance_to_vessel = []
# for point in points:
#     blank = np.zeros((height, length))
#     radius = 2
#     contact = False
#     while True:
#         if radius >=20 or contact:
#             break
#         cv2.circle(blank, (point[1], point[0]), radius, 255, -1) 
#         for i in range(np.max([0,point[0] - radius]), np.min([height, point[0] + radius])):
#             for j in range(np.max([0,point[1] - radius]), np.min([length, point[1] + radius])):
#                 if(blank[i][j] == 255 and mask[i][j] == 255):
#                     contact_points.append([point])
#                     contact = True
#         radius +=1
#     distance_to_vessel.append(radius)
#     print(radius)

# print(np.shape(points))
# print(np.shape(distance_to_vessel))

# for i in range(np.shape(distance_to_vessel)[0]):
#     if(distance_to_vessel[i] <=10):
#         insert[points[i][0]][points[i][1]] = [255, 255, 255]







# cv2.imshow('image', insert)
# cv2.waitKey(0)



# filename = 'points.jpg'
# cv2.imwrite(filename, insert)
# cv2.imshow('image', insert)
# cv2.waitKey(0)








# while True:
#     next_edge = findIntersection(sections[-1], contour, radius, height, length)
#     next_intersection, next_intersection_dis = furthestPoint(sections[-2], next_edge)
#     if(next_intersection<=)
#     print(next_intersection)
#     sections.append(next_intersection)








# third = findIntersection(second_int, contour, radius, height,length)
# third_furthest = furthestPoint(start, third)

# print(start)
# print(second_int)
# print("third_furthest: " + str(third_furthest))
# print(third)


# radius = 60
# blank = np.zeros_like(insert_mask)
# cv2.circle(blank, (0, start[0]), radius, 255, 0)

# for i in range(np.max([0,start[0] - radius]), np.min([height, start[0] + radius])):
#     for j in range(np.max([0,start[1] - radius]), np.min([length, start[1] + radius])):
#         if(blank[i][j] ==255 and contour[i][j] ==255):
#             print([i,j])

    

# cv2.imshow('image', blank)
# cv2.waitKey(0)





# thin=cv2.ximgproc.thinning(img2)
# _, catheter, hierarchy = cv2.findContours(thin,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# print(catheter[1][:100])

# blank = np.zeros_like(insert)
# cv2.drawContours(blank, catheter, -1, (255,255,255), 1)
# cv2.imshow('image', img2)
# cv2.waitKey(0)




# _, contours, hierarchy = cv2.findContours(opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# blank = np.zeros_like(insert)
# cv2.drawContours(blank, contours, 1, (255,255,255), -1)

# blank= cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)

# thin=cv2.ximgproc.thinning(blank)




# cv2.drawContours(insert, thin, 1, (255,255,255), 1)



# cv2.imshow('image', insert)
# cv2.waitKey(0)
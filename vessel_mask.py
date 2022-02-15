import cv2
import numpy as np

img = cv2.imread('original.jpg')

img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

rec, img2 = cv2.threshold(img, 70,255, cv2.THRESH_BINARY)

kernel = np.ones((6,6),np.uint8)
opening = cv2.morphologyEx(img2, cv2.MORPH_OPEN, kernel)

height = np.shape(opening)[0]
length = np.shape(opening)[1]


for i in range(length):
    if opening[height-20][i] == 255:
        left_lower_rec_ver = i
        break



for i in reversed(range(height)) :
    if opening[i][left_lower_rec_ver-10] == 255:
        left_lower_rec_hori = i
        break
    
print('left_lower_rec_ver: ' +  str(left_lower_rec_ver)  )
print('left_lower_rec_hori: ' +  str(left_lower_rec_hori)  )

for i in range(left_lower_rec_hori-10, height):
    for j in range(left_lower_rec_ver+10):
        opening[i][j] = 255
    

for i in range(length):
    if opening[height-1][i] == 0:
        right_lower_rec_ver = i
        break

print('right_lower_rec_ver: ' +  str(right_lower_rec_ver)  )

for i in range(right_lower_rec_ver, length):
    for j in range(height):
        opening[j][i] = 255

for i in range(10):
    for j in range(length):
        opening[i][j] = 255


opening = 255 - opening

insert = cv2.imread('insert.jpg')

opening_mask = np.zeros((height,length,3))
opening_mask = opening_mask + 255

cv2.imwrite('vessel_mask.jpg', opening)









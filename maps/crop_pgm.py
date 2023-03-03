import cv2
import numpy as np

img = cv2.imread('map.pgm', cv2.IMREAD_GRAYSCALE)
img = img[1900:2100, 1900:2100]
img = img.astype(np.uint8)
cv2.imwrite('new_map.pgm', img)

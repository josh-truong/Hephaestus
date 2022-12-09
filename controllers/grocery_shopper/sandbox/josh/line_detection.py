import cv2
import numpy as np
import matplotlib.pyplot as plt

img = np.load("saved/map.npy")
# img = np.load("filter_map.npy")
img = np.uint8(img*255) # Add a filter method here
edges = cv2.Canny(img, 50, 150)
line_image = np.copy(img) * 0  # creating a blank to draw lines on
# Run Hough on edge detected image
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, np.array([]), 50, 20)
lines = np.row_stack(lines)



# Detect only outliers in the x coordinates
lines_length = np.array([[abs(x1-x2),abs(y1-y2)] for x1,y1,x2,y2 in lines])
xIQR = np.subtract(*np.percentile(lines_length[:,0],[75,25]))
lines = lines[np.where(lines_length[:,0] > xIQR)]

for x1,y1,x2,y2 in lines:
    cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
cv2.imshow('Line Detection', line_image)
cv2.waitKey(0)
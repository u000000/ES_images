import cv2
import numpy as np

# Specify input and output coordinates that is used to calculate the transformation matrix
input_pts = np.float32([[590,265],[850,203],[838,443],[400,544]])
output_pts = np.float32([[-45.3,41],[-62,24.7],[-71.6,43.3],[-45,69.5]])

# Compute the perspective transform M
M = cv2.getPerspectiveTransform(input_pts,output_pts)
print(M)
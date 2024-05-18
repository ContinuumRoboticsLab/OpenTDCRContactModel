import cv2
import numpy as np
import sys

blank_image = np.ones((500,500,3), np.uint8)

def nothing(x):
    pass

def generate_superellipse_points(a, b, n, num_points=1000):
    """Generate points for a superellipse."""
    t = np.linspace(0, 2 * np.pi, num_points)
    x = a * np.sign(np.cos(t)) * (np.abs(np.cos(t))**(2/n))
    y = b * np.sign(np.sin(t)) * (np.abs(np.sin(t))**(2/n))
    points = np.vstack((x, y)).T
    return points

def draw_superellipse(image, center, a, b, n, thickness=1):
    """Draw a superellipse on an image."""
    points = generate_superellipse_points(a, b, n)
    # Translate points to the center
    points[:, 0] += center[0]
    points[:, 1] += center[1]
    # Convert points to integer
    points = points.astype(np.int32)
    # Draw superellipse
    cv2.polylines(image, [points], isClosed=True, color=(0, 255, 0), thickness=thickness)

cv2.namedWindow('Taskspace')
ob_num = 0

cv2.createTrackbar('major', 'Taskspace', 0, 400, nothing)
cv2.createTrackbar('minor', 'Taskspace', 0, 400, nothing)
cv2.createTrackbar('n', 'Taskspace', 0, 20, nothing)

cv2.setTrackbarPos('major', "Taskspace", 50)
cv2.setTrackbarPos('minor', "Taskspace", 50)
cv2.setTrackbarPos('n', "Taskspace", 2)

while True:
    copy_im = blank_image.copy()
    
    major = cv2.getTrackbarPos('major', 'Taskspace')
    minor = cv2.getTrackbarPos('minor', 'Taskspace')
    n = cv2.getTrackbarPos('n', 'Taskspace')
    
    #cv2.ellipse(copy_im,(256,256),(major,minor),0,0,360,255,-1)
    draw_superellipse(copy_im, (256, 256), major, minor, n)
    cv2.putText(copy_im, f"({major / 10000},{minor / 10000},{n})", (0, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 1)
    
    cv2.imshow("Taskspace", copy_im)
    if cv2.waitKey(1) == ord('q'):
        # Write to file
        break
    
    
cv2.destroyAllWindows()
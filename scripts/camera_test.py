import cv2
import numpy as np

# Set camera parameters
cap = cv2.VideoCapture(0)


# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# cap.set(cv2.CAP_PROP_FPS, 60)
# cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
# # set the focus to infinity
# cap.set(cv2.CAP_PROP_FOCUS, 0)
# # make sure auto exposure and auto white balance are turned off
# cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)



while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of orange color in HSV, with much tighter bounds for better detection
    lower_orange = np.array([10, 80, 230])
    upper_orange = np.array([40, 255, 255])
    # upper_orange = np.array([40, 128, 255])


    # Threshold the HSV image to get only orange colors
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours on original frame
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

    # Display the resulting frame
    cv2.imshow('frame', frame)


    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything when done
cap.release()
# out.release()
cv2.destroyAllWindows()


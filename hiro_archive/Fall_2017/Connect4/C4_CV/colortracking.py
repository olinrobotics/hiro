cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    # Take each frame
    _, frame = cap.read()

    # convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_white = np.array([0, 0, 0])
    upper_white = np.array([255, 50, 255])

    # Threshold the SSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_white, upper_white)

    #Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

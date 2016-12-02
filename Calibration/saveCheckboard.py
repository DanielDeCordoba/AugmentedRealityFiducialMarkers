from webcam import Webcam
import cv2
from datetime import datetime

webcam = Webcam()
webcam.start()


while True:

    # get image from webcam
    image = webcam.get_current_frame()

    # display image
    cv2.imshow('grid', image)
    cv2.waitKey(1000)

    # save image to file, if pattern found
    ret, corners = cv2.findChessboardCorners(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (9, 6), None)

    print ret

    if ret == True:
        filename = datetime.now().strftime('%Y%m%d_%Hh%Mm%Ss%f') + '.jpg'
        cv2.imwrite(filename, image)
        print filename

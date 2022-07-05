from os.path import exists
import numpy as np
import cv2
import glob
import yaml
from PIL import Image
import picamera
import time

class calibration:
    def __init__(self, r=0) -> None:
        self.res = [(1280, 720), (4056, 3040)]
        self.r=r

    def setRes(self, i):
        self.r = i

    def setResxy(self, x, y):
        try:
            index = self.res.index((x,y))
            self.r = index 
        except:
            print("THE RESOLUTION ISN'T SUPPORTED")
    
    def save_coefficients(self, mtx, dist, path):
        """ Save the camera matrix and the distortion coefficients to given path/file. """
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        cv_file.write("K", mtx)
        cv_file.write("D", dist)
        # note you *release* you don't close() a FileStorage object
        cv_file.release()
    
    def getRes(self):
        return self.res[self.r]
    
    def getimg(self, n):
        with picamera.PiCamera() as camera:
            camera.resolution = self.getRes()
            camera.framerate = 1
            # Wait for the automatic gain control to settle
            time.sleep(2)
            # Now fix the values
            #camera.shutter_speed = camera.exposure_speed
            #camera.exposure_mode = 'off'
            #g = camera.awb_gains
            #camera.awb_mode = 'off'
            #camera.awb_gains = g
            # Finally, take several photos with the fixed settings
            camera.capture_sequence(['cal/img%02d.jpg' % i for i in range(n)])
        
    def genMatrix(self, img):
        #Copy the code from the pi
        #Should take an image as a parameter and save the matrix with the name coef{r}.yml
        self.getimg(10)
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((9*6,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        images = glob.glob('cal/*.jpg')
        print(len(images))

        for fname in images:
            
            img = cv2.imread(fname)
            if not (img is None):


                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                #cv2.imshow('img',gray)
                #cv2.waitKey(500)
                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

                # If found, add object points, image points (after refining them)
                if ret == True:
                    objpoints.append(objp)

                    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                    imgpoints.append(corners2)

                    # Draw and display the corners
                    #cv2.drawChessboardCorners(img, (9,6), corners2,ret)
                    #cv2.imshow('img',img)
                    #cv2.imwrite('test.jpg',img)
                    #cv2.waitKey(500)

        #print(objpoints)
        #print('--------------------')
        #print(imgpoints)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        #print('ret={}, mtx={}, dist={}, rvecs={}, tvecs={}'.format(ret, mtx, dist, rvecs, tvecs))

        #img = cv2.imread('im0.jpg')
        #h,  w = img.shape[:2]
        #newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        # undistort
        #dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

        # crop the image
        #x,y,w,h = roi
        #dst = dst[y:y+h, x:x+w]
        #cv2.imwrite('calibresult.png',dst)

        self.save_coefficients(mtx, dist, '/home/pi/TestCamera/coef{}.yml'.format(self.r))

    def getMatrix(self):
        matrix = 'coef{}.yml'.format(self.r)
        if exists(matrix):
            return matrix
        return -1

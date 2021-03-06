from turtle import color
import cv2
import numpy as np
import json
from datetime import datetime
import yaml

class imageproc:
    """This class works with the images taken by the camera and tries to find the laser position in them using computer vision.
    """

    def __init__(self, img=None):
        """An instance of the imageproc class.

        Args:
            img (nparray, optional): demo image with the laser to construct the object around. Defaults to None.
        """

        self.img = img
        self.mask = None
        self.color=color

        self.hue_min = 240
        self.hue_max = 265
        self.sat_min = 100
        self.sat_max = 255
        self.val_min = 150
        self.val_max = 256
        self.channels = {
            'hue': None,
            'saturation': None,
            'value': None
        }

        #self.crop = (0.2777, 0.07, 0.2343, 0.2734)
        self.crop = (.07, 0.07, 0.147, 0.1)
    
    def undist(self, path) -> None:
        """Undistort the current depth image using the distortion matrix coeficients and crop the image to the size of interest.

        Args:
            path (str): location of the distortion matrix coeficients
        """

        #LOAD DISTORTION COEFICIENTS
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        mtx = cv_file.getNode("K").mat()
        dist = cv_file.getNode("D").mat()
        cv_file.release()

        #PERFORM THE UNDISTORTION
        h,  w = self.img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        dst = cv2.undistort(self.img, mtx, dist, None, newcameramtx)

        #CROP THE FINAL IMAGE TO SIZE
        x,y,w,h = roi
        #ex, eh, ew, ey = self.crop
        #up down left right crop in procentages of the initial height and width
        ey, eh, ex, ew = self.crop
        bh,  bw = self.img.shape[:2]
        ey, eh, ex, ew = int(ey*bh), int(eh*bh), int(ex*bw), int(ew*bw) 

        dst = dst[y+ey:y+h-eh, x+ex:x+w-ew]
        self.img = dst
        
    def undistC(self, path) -> None:
        """Undistort the current color image using the distortion matrix coeficients and crop the image to the size of interest.

        Args:
            path (str): location of the distortion matrix coeficients
        """

        #LOAD DISTORTION COEFICIENTS
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        mtx = cv_file.getNode("K").mat()
        dist = cv_file.getNode("D").mat()
        cv_file.release()

        #PERFORM THE UNDISTORTION
        h,  w = self.color.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        dst = cv2.undistort(self.color, mtx, dist, None, newcameramtx)

        #CROP THE FINAL IMAGE TO SIZE
        x,y,w,h = roi
        #ex, eh, ew, ey = self.crop
        #up down left right crop in procentages of the initial height and width
        ey, eh, ex, ew = self.crop
        bh,  bw = self.color.shape[:2]
        ey, eh, ex, ew = int(ey*bh), int(eh*bh), int(ex*bw), int(ew*bw) 

        dst = dst[y+ey:y+h-eh, x+ex:x+w-ew]
        self.color = dst

    def changePerspective(self) -> None:
        """Fix the perspective of the camera.
        NOT YET IMPLEMENTED! DO NOT USE!
        """

        #I have no idea how to do it
        #To be made
        pass
    
    def localMaxToImg(self, mask, dh):
        """Tries to determine the laser position in the picture using a primitive algorithm and the intensity of the color.

        Args:
            mask (nparray): mask with the laser area
            dh (int): maximum color intensity difference from average

        Returns:
            nparray: laser position encoded in a boolean array
        """

        im = cv2.bitwise_and(self.img, self.img, mask=mask)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2HLS)
        sum = im.sum(axis=(0,1))
        nr = np.count_nonzero(im, axis=(0,1))
        avg = sum/nr
        test = np.zeros(im.shape)
        for k, lines in enumerate(im):
            mx=0
            poz=0
            for p, pixel in enumerate(lines):
                if pixel[1] > mx and abs(avg[0]-pixel[0]) <= dh:
                    mx = pixel[1]
                    poz = p
            test[k][poz]=255
        return test

    def localMax(self, mask, dh):
        """Tries to determine the laser position in the picture using a primitive algorithm and the intensity of the color.

        Args:
            mask (nparray): mask with the laser area
            dh (int): maximum color intensity difference from average

        Returns:
            list: laser possition encoded in a list of indexes
        """

        points =[]
        im = cv2.bitwise_and(self.img, self.img, mask=mask)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2HLS)
        sum = im.sum(axis=(0,1))
        nr = np.count_nonzero(im, axis=(0,1))
        avg = sum/nr
        for k, lines in enumerate(im):
            mx=0
            poz=-1
            for p, pixel in enumerate(lines):
                if pixel[1] > mx and abs(avg[0]-pixel[0]) <= dh:
                    mx = pixel[1]
                    poz = p
            col = list(self.color[k, poz])
            points.append([poz] + col)
        return points

    def localMaxDIVIDEIMPERA(self, mask):
        """Tries to determine the laser position in the picture using a gready algorithm based on Divide et Impera.

        Args:
            mask (nparray): mask with the laser area

        Returns:
            list: laser possition encoded in a list of indexes
        """
        
        im = cv2.bitwise_and(self.img, self.img, mask=mask)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2HLS)
        cv2.imshow('GG', im)
        sum = im.sum(axis=(0,1))
        nr = np.count_nonzero(im, axis=(0,1))
        avg = sum/nr
        print(avg)
        test = np.zeros(im.shape)
        for k, lines in enumerate(im):
            l = np.copy(lines)
            order = np.arange(l.shape[0]).reshape(-1,1)
            l = np.hstack((l,order))
            #l= 610 *3
            #h1 s1 v1
            #h2 s2 v2
            #etc
            l = l[np.argsort(l[:, 1])] #sort the l by the V component
            n=l.shape[0]

            
            h=avg[0]
            p=0
            u=n-1
            while p<=u:
                m = int((p+u)/2)
                if l[m][0]<= h:
                    p=m+1
                else:
                    u=m-1
            if u>=0:
                st = u
            else:
                st = 0

            p=0
            u=n-1
            while p<=u:
                m = int((p+u)/2)
                if l[m][0] >= h:
                    u=m-1
                else:
                    p=m+1
            if p<n:
                dr = p
            else: 
                dr = 0


            if abs(h - l[st][0]) <  abs(h - l[dr][0]):
                f=st
            else: 
                f=dr

  
            test[k][l[n-1][3]]=255


            #l.flatten().tofile(f, sep='||')
            #f.write('-----------------------\n')
            
        return test

    def localMaxQUICK(self, mask):
        """Tries to determine the laser position in the picture using an algorithm based on QuickSort.

        Args:
            mask (nparray): mask with the laser area

        Returns:
            list: laser possition encoded in a list of indexes
        """

        points =[]
        im = cv2.bitwise_and(self.img, self.img, mask=mask)
        imhls = cv2.cvtColor(im, cv2.COLOR_BGR2HLS)
        test = np.zeros(im.shape)
        for k, lines in enumerate(imhls):
            if np.sum(mask[k]) > 0:
                l = np.copy(lines)
                order = np.arange(l.shape[0]).reshape(-1,1)
                l = np.hstack((l,order))
                #l= 610 *3
                #h1 s1 v1
                #h2 s2 v2
                #etc
                l = l[np.argsort(l[:, 1])] #sort the l by the V component
                n=l.shape[0]
                poz = l[-1][3]
                col = list(self.color[k, poz])
            else:
                poz=-1
                col=[0,0,0]
                #print('Linia {} nu contine puncte'.format(k))
            
            points.append([poz] + col)
            
        return points

    def combineFilter(self, dilate = 2):
        """Combine multiple filters(masks) into a single one to eliminate errors.

        Args:
            dilate (int, optional): magnitude of dilatation used. Defaults to 2.

        Returns:
            nparray: the combined filter(mask)
        """

        m1 = self.colorFilter()
        m2 = self.hsvFilter()
        m1 = cv2.GaussianBlur(m1,(5,5),0)
        m2 = cv2.GaussianBlur(m2,(5,5),0)
        m1 = self.dilate(m1,dilate)
        m2 = self.dilate(m2,dilate) 
        mask = cv2.bitwise_and(m1, m2)
        mask = cv2.GaussianBlur(mask,(5,5),0)
        mask = cv2.threshold(mask, 20, 255, cv2.THRESH_BINARY)[1]
        return mask

    def dilate(self,img,size):
        """Apply the dilatation transformation to an image.

        Args:
            img (nparray): surce image
            size (int): magnitude of dilatation

        Returns:
            nparray: the dilatated image
        """

        dilatation_size = size
        dilation_shape = cv2.MORPH_ELLIPSE
        element = cv2.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1), (dilatation_size, dilatation_size))
        return cv2.dilate(img, element)

    def colorFilter(self, ch=0):
        """Generate a filter(mask) based on the colors in the image.

        Args:
            ch (int, optional): color channel used. Defaults to 0 (for red lasers).
        
        Returns:
            nparray: filter generated as a boolean array
        """

        if ch == 0:
            lowerb = np.array([120, 0, 0])
            upperb = np.array([255, 120, 120])
        elif ch == 1:
            lowerb = np.array([0, 120, 0])
            upperb = np.array([120, 255, 120])
        elif ch == 2:
            lowerb = np.array([0, 0, 120])
            upperb = np.array([120, 120, 255])

        lowert = 40
        uppert = 255

		#1200,1600
        self.mask = cv2.inRange(self.img, lowerb, upperb)
        temp = cv2.bitwise_and(self.img, self.img, mask=self.mask)

        ch = temp[:,:,ch]
        ch = cv2.threshold(ch, lowert, uppert, cv2.THRESH_BINARY)[1]
        return ch
    
    def hsvFilter(self):
        """Generate a filter(mask) based on the saturation and the HSV decomposition of the image.

        Returns:
            nparray: filter generated as a boolean array
        """

        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # split the video frame into color channels
        h, s, v = cv2.split(hsv_img)
        self.channels['hue'] = h
        self.channels['saturation'] = s
        self.channels['value'] = v

        # Threshold ranges of HSV components; storing the results in place
        self.threshold_image("hue")
        self.threshold_image("saturation")
        self.threshold_image("value")

        # Perform an AND on HSV components to identify the laser!
        mask = cv2.bitwise_and(self.channels['hue'], self.channels['value'])
        mask = cv2.bitwise_and(self.channels['saturation'], mask)
        return mask

    def threshold_image(self, channel) -> None:
        """Apply a treshhold operation to the specific HSV channel.

        Args:
            channel (str): channel which the transformation should be applied
        """

        if channel == "hue":
            minimum = self.hue_min
            maximum = self.hue_max
        elif channel == "saturation":
            minimum = self.sat_min
            maximum = self.sat_max
        elif channel == "value":
            minimum = self.val_min
            maximum = self.val_max

        (t, tmp) = cv2.threshold(
            self.channels[channel],  # src
            maximum,  # threshold value
            0,  # we dont care because of the selected type
            cv2.THRESH_TOZERO_INV  # t type
        )

        (t, self.channels[channel]) = cv2.threshold(
            tmp,  # src
            minimum,  # threshold value
            255,  # maxvalue
            cv2.THRESH_BINARY  # type
        )

        if channel == 'hue':
            self.channels['hue'] = cv2.bitwise_not(self.channels['hue'])
    
    def getImg(self):
        """Get current depth image in the buffer.

        Returns:
            nparray: image
        """

        return self.img
    
    def getRes(self):
        """Get the resolution of the scan.

        Returns:
            list: resolution of the scan
        """

        return (self.img.shape[0], self.img.shape[1])
        
    def setImg(self, img) -> None:
        """Set a depth image to the internal buffer.

        Args:
            img (nparray): image to be set
        """

        self.img = img
    
    def setColor(self, img) -> None:
        """Set a color image to the internal buffer.

        Args:
            img (nparray): image to be set
        """

        self.color = img 
    
    def getMask(self, mask):
        """Get the image with the filter applied.

        Args:
            mask (nparray): filter to be used

        Returns:
            nparray: image generated
        """

        return cv2.bitwise_and(self.img, self.img, mask=mask)

class writer:
    """This class is used to arrange, sanitize and save the data generated by the scanner in a JSON file type.
    """
    
    def __init__(self, fname) -> None:
        """A writter for the scan data.

        Args:
            fname (str): filename of the data to be saved
        """

        self.fname = fname
        self.res = (0,0)
        self.points = []
        self.angle = []
        self.weight = 0
        self.toppoints = []
        self.topangle = []

    def setHeader(self, res, weight=0) -> None:
        """Set the top data of the file.

        Args:
            res (list): resolution used to scan
            weight (float, optional): weight of the object scanned. Defaults to 0.
        """

        self.res = res
        self.weight = weight
    
    def addData(self, line, angle) -> None:
        """Add data to the internal buffer.

        Args:
            line (nparray): list of laser position with color information
            angle (float): angle of the measurement
        """

        self.points.append(line)
        self.angle.append(angle)

    def addDataTop(self, line, angle) -> None:
        """Add data for the top side to the internal buffer.

        Args:
            line (nparray): list of laser position with color information
            angle (float): angle of the measurement
        """

        self.toppoints.append(line)
        self.topangle.append(angle)

    def save(self) -> None:
        """Save the data into a JSON file.
        """
        
        template = {
            "date" :  datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "res": self.res,
            "weght": self.weight,
            "samples": [],
            "top" : []
        }

        samples = template['samples']
        points_arr = np.array(self.points)
        #print(points_arr)
        #points_arr.reshape((nsamples, self.res[0]))
        for k, line in enumerate(points_arr):
            sample_template = {
                "angle": self.angle[k],
                "points": line.tolist()
            }
            samples.append(sample_template)
        
        topsamples = template['top']
        toppoints_arr = np.array(self.toppoints)
        for k, line in enumerate(toppoints_arr):
            sample_template = {
                "angle": self.topangle[k],
                "points": line.tolist()
            }
            topsamples.append(sample_template)


        data = json.dumps(template, indent=4)

        with open(self.fname, 'w') as js:
            js.write(data)

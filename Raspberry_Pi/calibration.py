from os.path import exists

class calibration:
    def __init__(self, r=0) -> None:
        self.res = [(1280, 720), (4056, 3040)]
        self.r=r

    def setRes(self, i):
        self.r = i

    def setRes(self, x, y):
        try:
            index = self.res.index((x,y))
            self.r = index 
        except:
            print("THE RESOLUTION ISN'T SUPPORTED")
    
    def getRes(sef, res):
        return sef.res[res]
        
    def genMatrix(self, img):
        #Copy the code from the pi
        #Should take an image as a parameter and save the matrix with the name coef{r}.yml
        pass

    def getMatrix(self):
        matrix = 'coef{}.yml'.format(self.r)
        if exists(matrix):
            return matrix
        return -1

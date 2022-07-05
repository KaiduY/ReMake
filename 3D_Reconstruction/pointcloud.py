import math
import numpy as np
import paramiko
from os import path
import json
from datetime import datetime
import cv2
import open3d
import sys
import trimesh
from pathlib import Path

class pointCloud:
    """This class represents the 3D reconstruction bundle for proccesing the data of the scanner.
    """

    def __init__(self):
        """An object of the pointCloud class.
        """

        self.points = []
        self.color = []
        self.flenght = 6 #mm focal lenght of the camera
        self.camera_center_distance = 300 #mm distance camera to center of the bed
        self.pixelsize = 0.00155 #mm horizontal dimension for one pixel 
        self.camera_laser_distance = 100 #mm distance between camaera and laser (the laser ray)
        self.ppm = 0
        self.date = None
        self.res = (0,0)
        self.angle=[]
        self.points = None
        self.toppoints = []
        self.topcolor = []
        self.topangle = []
        self.pointcloud = None
        self.magich = 0.8
        
    def appendLine(self, line):
        """Add a new line of data (points with color information) to the internal buffer.

        Args:
            line (nparry): array of points
        """

        self.points.append(line)

    def debug(self):
        """Show the primary parameters of the scan and draws the points in a 3D viewer.
        """

        print(self.points)
        print(self.ppm)
        print(self.res)
        open3d.visualization.draw_geometries([self.pointcloud])

    
    def download_file(self, name, host = "192.168.1.14", port = 22, password = "raspberry", username = "pi", path = '/home/pi/TestCamera/'):
        """Download the data from the Raspberry Pi via SSh.

        Args:
            name (str): name of the file
            host (str, optional): adress of the Raspberry Pi. Defaults to "192.168.1.14".
            port (int, optional): port of the Raspberry Pi. Defaults to 22.
            password (str, optional): password of the Raspberry Pi. Defaults to "raspberry".
            username (str, optional): username of the Raspberry Pi. Defaults to "pi".
            path (str, optional): local path of the file. Defaults to '/home/pi/TestCamera/'.
        """

        print("Downloading data from {}".format(host))
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(host, port, username, password)
        sftp = ssh.open_sftp()
        #remote_file = sftp.open(path + name)
        try:
            sftp.get(path+name, name)
        finally:
            sftp.close()
            ssh.close()

    def load_data(self, name):
        """Load the data in memory or downloads it from the Raspberry Pi if unavailable.

        Args:
            name (str): name of the data file
        """

        file = Path(name)
        if not file.exists():
            self.download_file(name)
        
        with open(file, 'r') as js:
            data = json.loads(js.read())
            self.date= datetime.strptime(data["date"], "%Y-%m-%d %H:%M:%S")
            self.res = tuple(data["res"])
            points = []
            colors = []
            for s in data["samples"]:
                self.angle.append(s["angle"])
                p_array = np.array(s["points"])

                xy_array = p_array[:,0].tolist()
                col_array = p_array[:,1:]
                points.append(xy_array)
                colors.append(col_array)

            #print(points)
            #self.points = points_arr[0]
            self.color = np.array(colors, dtype=object)
            #self.color.reshape(( len(data["samples"]), self.res[0]))
            self.points = np.array(points)
            #print(points[1])
            #print(self.points.shape)
            self.points.reshape(( len(data["samples"]), self.res[0]))

            toppoints = []
            topcolors = []
            for s in data["top"]:
                self.topangle.append(s["angle"])
                p_array = np.array(s["points"])

                xy_array = p_array[:,0].tolist()
                col_array = p_array[:,1:]
                toppoints.append(xy_array)
                topcolors.append(col_array)
            
            self.topcolor = np.array(colors, dtype=object)

            self.toppoints = np.array(toppoints)
            #print(points)

            self.toppoints.reshape(( len(data["top"]), self.res[0]))

            print("Data loaded successfully")
            #print(self.date)
            js.close()

    def show_laser(self, index):
        """Generate an image from the data available.

        Args:
            index (int): index of the picture taken by the scanner

        Returns:
            nparray: image generated
        """

        img = np.zeros((self.res[0], self.res[1], 3), dtype=np.uint8)
        points = self.points[index]
        for x, y in enumerate(points):
            img[x,y] = tuple(self.color[index, x])
            #print(tuple(self.color[index, x]))
        return img
        
    def compute_ppm(self):
        """Compute the ppm (pixels per meter) variable.
        """

        self.ppm = (self.camera_center_distance - self.camera_laser_distance) / self.flenght
        self.ppm = self.ppm * self.pixelsize

    def set_ppm(self, new_ppm):
        """Manually set the ppm (pixels per meter) variable.

        Args:
            new_ppm (float): new ppm value
        """

        self.ppm = new_ppm
                                
    def create_pointcloud(self, viewPC=False):
        """Generate in memory a new pointcloud from the data loaded and calculate its parameters.

        Args:
            viewPC (bool, optional): flag to show a 3D representation of the points. Defaults to False.
        """

        #self.pointcloud
        if self.ppm == 0:
            self.compute_ppm()

        center = self.res[1]/2
        hcenter = self.res[0]/2
        vertex = []
        colors = []
        multiplier = self.camera_center_distance / self.camera_laser_distance
        
        for index, view in enumerate(self.points):
            alpha = math.radians(self.angle[index])
            for height, pixel in enumerate(view):
                if pixel>=0 :
                    d = self.ppm * (pixel-center) * multiplier
                    x = d * math.cos(alpha)
                    y = d * math.sin(alpha)
                    z = height * self.ppm * self.magich
                    cond =  False or z < 200#(z>0 and z<294) or (abs(pixel-center<40)
                    if cond:
                        vertex.append((x, y, z))
                        rc = self.color[index, height, 0]
                        gc = self.color[index, height, 1]
                        bc = self.color[index, height, 2]

                        colors.append((rc, gc, bc))
        
        #Apply the same alghorithm for the top points but with the scanning plane rotated by 90 degrees 
        for index, view in enumerate(self.toppoints):
            alpha = math.radians(self.topangle[index])
            for height, pixel in enumerate(view):
                if pixel>=0 :
                    d = self.ppm * (pixel-center) * multiplier * .1 + 150
                    h = self.ppm * (height-hcenter) * self.magich * .7
                    x = h * math.cos(alpha)
                    y = h * math.sin(alpha)
                    z = d
                    cond =  abs(d-152) < 10 #???
                    if cond:
                        vertex.append((x, y, z))
                        rc = self.topcolor[index, height, 0]
                        gc = self.topcolor[index, height, 1]
                        bc = self.topcolor[index, height, 2]

                        colors.append((rc, gc, bc))
        #print(colors)
        self.pointcloud = open3d.geometry.PointCloud()
        self.pointcloud.points = open3d.utility.Vector3dVector(np.array(vertex))
        self.pointcloud.colors = open3d.utility.Vector3dVector(np.array(colors)/255)
        
        self.pointcloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.001, max_nn=30))
        _, ind = self.pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.02)
        #_, ind = self.pointcloud.remove_radius_outlier(nb_points=20, radius=20)#remove_statistical_outlier(nb_neighbors=1000, std_ratio=0.0001)
        self.pointcloud = self.pointcloud.select_by_index(ind)
        _, ind = self.pointcloud.remove_radius_outlier(nb_points=50, radius=20)#remove_statistical_outlier(nb_neighbors=1000, std_ratio=0.0001)
        self.pointcloud = self.pointcloud.select_by_index(ind)
        self.pointcloud.rotate(self.pointcloud.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4)),center=(0, 0, 0))
        if viewPC:
            open3d.visualization.draw_geometries([self.pointcloud])

    def poisson_reconstruction(self, depth):
        """Reconstruct a mesh of the pointcloud using Poisson's algorithm.

        Args:
            depth (int): parameter used to reconstruct the mesh

        Returns:
            mesh: generated mesh
        """

        mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.pointcloud, depth=depth, width=0, scale=1.1, linear_fit=True)[0]
        open3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
        bbox = self.pointcloud.get_axis_aligned_bounding_box()
        mesh = mesh.crop(bbox)
        return mesh
    
    def alpha_reconstruction(self, alpha):
        """Reconstruct a mesh of the pointcloud using Alpha reconstruction algorithm.

        Args:
            alpha (float): alpha parameter used in reconstruction

        Returns:
            mesh: generated mesh
        """

        mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(self.pointcloud, alpha)
        mesh.compute_vertex_normals()
        return mesh

    def ball_reconstruction(self, mult):
        """Reconstruct a mesh of the pointcloud using the pivoting ball algorithm.

        Args:
            mult (int): multiplier used to determine the ball size

        Returns:
            mesh: generated mesh
        """

        radii = [0.005, 0.01, 0.02, 0.04]*mult
        mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(self.pointcloud, open3d.utility.DoubleVector(radii))
        return mesh
    
    def save_mesh(self, mesh, name):
        """Save the mesh using STL format.

        Args:
            mesh (mesh): mesh to be saved
            name (str): name for the file
        """

        open3d.io.write_triangle_mesh("{}.stl".format(name), mesh)

    def fromImage(self, im):
        """Primitive algorithm used to extract data out of an boolean image containing the laser position.

        Args:
            im (nparray): image from which the data should be extracted
        """

        for k, line in enumerate(im):
            if len(self.points) == 0:
                seed = -1
            else:
                seed = self.points[k-1]

            point = self.__linePoint(line, seed)

            self.points.append(point)

    def __linePoint(self, line, seed=-1): #to be made private
        """Determine the laser position in a line of an image using Divide et Impera algorithm and the seed of the last position.

        Args:
            line (nparray): line from an image 
            seed (int, optional): seed with the last position found. Defaults to -1.

        Returns:
            int: position of the laser
        """
        
        cs=0
        cd=len(line)
        mid = (int)((cs+cd)/2)

        if seed < 0:
            seed=mid
         
        if seed >= mid:
            dr=seed
            for st in range(seed,cs,-1):
                if line[st] > 0:
                    return st

                if line[dr] > 0:
                    return dr

                dr = min(dr + 1, cd - 1) #use cd-1 because line is indexed from 0

        else:
            st = seed
            for dr in range(seed, cd, 1):
                if line[st] > 0:
                    return st

                if line[dr] > 0:
                    return dr
                    
                st = max(st - 1, cs)

        return -1

        
if __name__ == "__main__":

    view = True
    p = pointCloud()
    p.load_data('data.json')
    p.set_ppm(1)
    #p.debug()
    img = p.show_laser(1)

    p.create_pointcloud(viewPC=True)

    mesh = p.alpha_reconstruction(50)
    #mesh = p.poisson_reconstruction(3)
    #mesh = p.ball_reconstruction(1)

    p.save_mesh(mesh, "demo")
    #p.debug()
    if view:
        vis = open3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(mesh)#5
        #vis.add_geometry(p.poisson_reconstruction(12))
        #vis.add_geometry(p.ball_reconstruction(1))
        vis.run()
    
    meshh = trimesh.load_mesh('demo.stl')
    #trimesh.repair.fill_holes(meshh)
    #meshh = as_mesh(meshh)
    trimesh.repair.fix_inversion(meshh)
    trimesh.repair.fill_holes(meshh)
    #trimesh.repair.fix_normals(meshh)
    trimesh.repair.fix_winding(meshh)
    trimesh.smoothing.filter_laplacian(meshh)
    #trimesh.repair.fill_holes(meshh)

    if view:
        meshh.show()

    meshh.show()

    meshh.export("demo.stl")

    if view:
        cv2.imshow("any", img)
        k = cv2.waitKey(0)


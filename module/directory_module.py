import os
import natsort

class DirectoryModule:
    def __init__(self, point_dir):
        self.point_dir = point_dir

    def point_path(self):
        point_dir = os.path.join(self.point_dir)                      
        pointfile_list = os.listdir(point_dir)                                              
        pointfile_list = natsort.natsorted(pointfile_list)                                                                   
        point_fullpaths = [os.path.join(point_dir, name) for name in pointfile_list]    
        num_points = len(pointfile_list)     
        return point_fullpaths, num_points
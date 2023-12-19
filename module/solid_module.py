import numpy as np
import open3d as o3d

class SOLiDModule:
    def __init__(self, fov, num_angle, num_range, num_elevation, max_length):
        self.fov = fov
        self.num_angle = num_angle
        self.num_range = num_range
        self.num_elevation = num_elevation
        self.max_length = max_length

    def xy2theta(self, x, y):
        if (x >= 0 and y >= 0): 
            theta = 180/np.pi * np.arctan(y/x)
        if (x < 0 and y >= 0): 
            theta = 180 - ((180/np.pi) * np.arctan(y/(-x)))
        if (x < 0 and y < 0): 
            theta = 180 + ((180/np.pi) * np.arctan(y/x))
        if ( x >= 0 and y < 0):
            theta = 360 - ((180/np.pi) * np.arctan((-y)/x))
        return theta

    def pt2rah(self, point, fov, gap_range, gap_angle, gap_elevation, num_range, num_angle, num_elevation):
        x = point[0]
        y = point[1]
        z = point[2]
        
        if(x == 0.0):
            x = 0.001
        if(y == 0.0):
            y = 0.001 

        theta = self.xy2theta(x, y) 
        faraway = np.sqrt(x*x + y*y) 
        phi = np.rad2deg(np.arctan2(z, np.sqrt(x**2 + y**2)))

        idx_range = np.divmod(faraway, gap_range)[0]      
        idx_angle = np.divmod(theta, gap_angle)[0]    
        idx_elevation = np.divmod(phi, gap_elevation)[0]+(num_elevation/2-1)    

        if(idx_range >= num_range):
            idx_range = num_range-1 

        if(idx_elevation >= num_elevation):
            idx_elevation = num_elevation-1 

        return int(idx_range), int(idx_angle), int(idx_elevation)   

    def pt2solid(self, ptcloud):
        num_points = ptcloud.shape[0]              
        
        gap_range = self.max_length/self.num_range             
        gap_angle = 360/self.num_angle              
        gap_elevation = (self.fov*2)/self.num_elevation                

        rh_counter = np.zeros([self.num_range, self.num_elevation])               #sc 카운터 2차원 배열
        sh_counter = np.zeros([self.num_angle, self.num_elevation])               #sc 카운터 2차원 배열
        for pt_idx in range(num_points):        # 포인트 클라우드의 수만큼 반복
            point = ptcloud[pt_idx, :]          # 해당 포인트의 x, y, z값을 가져옴
            idx_range, idx_angle, idx_elevation = self.pt2rah(point, self.fov, gap_range, gap_angle, gap_elevation, self.num_range, self.num_angle, self.num_elevation) #극좌표계로 변환(링, 섹터)
            rh_counter[idx_range, idx_elevation] = rh_counter[idx_range, idx_elevation] + 1     # 해당 빈의 카운트 +1
            sh_counter[idx_angle, idx_elevation] = sh_counter[idx_angle, idx_elevation] + 1     # 해당 빈의 카운트 +1
        
        range_matrix = rh_counter
        angle_matrix = sh_counter
        number_vector = np.sum(range_matrix, axis=0)
        min_val = number_vector.min()
        max_val = number_vector.max()
        number_vector = (number_vector - min_val) / (max_val - min_val)

        range_solid = range_matrix.dot(number_vector)
        angle_solid = angle_matrix.dot(number_vector)

        return range_solid, angle_solid
    
    def get_descriptor(self, scan):
        range_solid, angle_solid = self.pt2solid(scan)
        solid = np.concatenate((range_solid, angle_solid))
        return solid
    
    def loop_detection(self, query, candidate):
        cosine_similarity = np.dot(query, candidate) / (np.linalg.norm(query) * np.linalg.norm(candidate))
        return cosine_similarity

    def pose_estimation(self, query, candidate):
        initial_cosdist = []
        for shift_index in range(len(candidate)):
            initial_cosine_similarity = np.sum(np.abs(query - np.roll(candidate, shift_index)))
            initial_cosdist.append(initial_cosine_similarity)
        angle_difference = 360-(np.argmin(initial_cosdist)+1)*(360/self.num_angle)
        return angle_difference


import copy
import argparse
import numpy as np
np.set_printoptions(precision=4)

import matplotlib.pyplot as plt

from tqdm import tqdm
from module.util_module import *
from module.point_module import *
from module.solid_module import *
from module.directory_module import *
from module.posegraph_module import *
from matplotlib.animation import FFMpegWriter

class TerminalColors:
    BLACK = '\033[1;30m'
    RED = '\033[1;31m'
    GREEN = '\033[1;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[1;34m'
    MAGENTA = '\033[1;35m'
    CYAN = '\033[1;36m'
    WHITE = '\033[1;37m'
    RESET = '\033[0m'

# LiDAR DTYPE ============================================================================================
## KTTI
lidar_dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]
## HeLiPR-Aeva for Town01 / Town02
# lidar_dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('reflectivity', np.float32), ('velocity', np.float32), ('time_offset_ns', np.int32), ('line_index', np.uint8)]
## HeLiPR-Aeva for KAIST04 / KAIST05
# lidar_dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('reflectivity', np.float32), ('velocity', np.float32), ('time_offset_ns', np.int32), ('line_index', np.uint8), ('intensity', np.float32)]
## HeLiPR-Avia
# lidar_dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('reflectivity', np.uint8), ('tag', np.uint8), ('line', np.uint8), ('offset_time', np.uint32)]
## PARK
# lidar_dtype = [('x', np.float64), ('y', np.float64), ('z', np.float64), ('intensity', np.float64)]
# =========================================================================================================

# PARSER ==================================================================================================
parser = argparse.ArgumentParser(description= "SOLiD")
parser.add_argument('--fov',            type = float, default = '26.8',             help = 'FOV')
parser.add_argument('--num_range',      type = int,   default = '40',               help = 'channel')
parser.add_argument('--num_angle',      type = int,   default = '60',               help = 'channel')
parser.add_argument('--num_elevation',  type = int,   default = '64',               help = 'channel')
parser.add_argument('--min_distance',   type = int,   default = '3',                help = 'channel')
parser.add_argument('--max_distance',   type = int,   default = '80',               help = 'channel')
parser.add_argument('--voxel_size',     type = float, default = '0.5',              help = 'channel')
parser.add_argument('--point_dir',      type = str,   default = 'Datasets/bin/',    help = 'channel')
parser.add_argument('--recent_node',    type = int,   default = '100',              help = 'channel')
parser.add_argument('--loop_threshold', type = float, default = '0.004',            help = 'channel')
parser.add_argument('--save_gap',       type = int,   default = '300')
parser.add_argument('--sequence_idx',   type = str,   default = '05')

args = parser.parse_args()
# =========================================================================================================

# Directory Module ========================================================================================
UM = UtilModule()
# =========================================================================================================

# Directory Module ========================================================================================
DM = DirectoryModule(args.point_dir)
point_fullpaths, num_points = DM.point_path()
# =========================================================================================================

# Point Module ============================================================================================
PM = PointModule(args.min_distance, 
                 args.max_distance, 
                 args.voxel_size,
                 lidar_dtype)
# =========================================================================================================

# SOLiD Module ============================================================================================
SM = SOLiDModule(args.fov, 
                 args.num_angle, 
                 args.num_range, 
                 args.num_elevation, 
                 args.max_distance)
# =========================================================================================================

# PGO Module ==============================================================================================
PGM = PoseGraphModule()
PGM.addPriorFactor()
# =========================================================================================================

# Video Params ============================================================================================
save_dir = "result/" + args.sequence_idx
if not os.path.exists(save_dir): os.makedirs(save_dir)
ResultSaver = PoseGraphResultSaver(init_pose=PGM.curr_se3, 
                             save_gap=args.save_gap,
                             num_frames=num_points,
                             seq_idx=args.sequence_idx,
                             save_dir=save_dir)

fig_idx = 1
fig = plt.figure(fig_idx)
writer = FFMpegWriter(fps=30)
video_name = "result/" + args.sequence_idx + ".mp4"
num_frames_to_skip_to_show = 10
num_frames_to_save = np.floor(num_points/num_frames_to_skip_to_show)
# =========================================================================================================

solid_database = np.empty((1, args.num_range + args.num_angle), dtype=int)
point_database = [None] * 100000

with writer.saving(fig, video_name, num_frames_to_save): # this video saving part is optional
    for curr_idx, scan_path in tqdm(enumerate(point_fullpaths), total=num_points, mininterval=5.0):
        curr_scan_pts  = PM.readScan(scan_path) 
        curr_scan_pts  = PM.remove_closest_points(curr_scan_pts)
        curr_scan_pts  = PM.remove_far_points(curr_scan_pts)
        curr_scan_pts  = PM.down_sampling(curr_scan_pts)
        curr_solid     = np.array([SM.get_descriptor(curr_scan_pts)])
        solid_database = np.append(solid_database, curr_solid, axis=0) 
            
        point_database[curr_idx] = curr_scan_pts

        PGM.curr_node_idx = curr_idx # make start with 0
        if(curr_idx == 0):
            PGM.prev_node_idx = PGM.curr_node_idx
            prev_scan_pts = copy.deepcopy(curr_scan_pts)
            icp_initial = np.eye(4)
            continue

        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(curr_scan_pts)

        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(prev_scan_pts)
        target.estimate_normals()

        reg_p2p = o3d.pipelines.registration.registration_icp(source = source, 
                                                              target = target, 
                                                              max_correspondence_distance = 0.5, 
                                                              init = icp_initial, 
                                                              estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(), 
                                                              criteria = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.001, max_iteration=20))
        odom_transform = reg_p2p.transformation 

        PGM.curr_se3 = np.matmul(PGM.curr_se3, odom_transform)
        icp_initial = odom_transform
        PGM.addOdometryFactor(odom_transform)

        PGM.prev_node_idx = PGM.curr_node_idx
        prev_scan_pts = copy.deepcopy(curr_scan_pts)

        if PGM.curr_node_idx <= args.recent_node:
            pass 
        else:
            cosdist = []
            
            for candidate_idx in range(PGM.curr_node_idx - args.recent_node):
                query_R_solid     = solid_database[PGM.curr_node_idx, :args.num_range]
                candidate_R_solid = solid_database[candidate_idx, :args.num_range]
                cosine_similarity = SM.loop_detection(query_R_solid, candidate_R_solid)
                cosdist.append(1-cosine_similarity)

            loop_idx      = np.argmin(cosdist)
            loop_distance = cosdist[loop_idx]
            
            if loop_distance < args.loop_threshold:
                print(TerminalColors.GREEN + "Loop Detection: " + str(PGM.curr_node_idx) + " and " + str(loop_idx) + " with " + str(loop_distance) + TerminalColors.RESET) 
                query_A_solid     = solid_database[PGM.curr_node_idx, args.num_range:]
                candidate_A_solid = solid_database[loop_idx, args.num_range:]
                angle_difference  = SM.pose_estimation(query_A_solid, candidate_A_solid)
                print(TerminalColors.BLUE + "Angle Difference: " + str(angle_difference) + TerminalColors.RESET) 

                init_pose         = UM.yawdeg2se3(angle_difference)
                loop_scan_pts = point_database[loop_idx]
                loop = o3d.geometry.PointCloud()
                loop.points = o3d.utility.Vector3dVector(loop_scan_pts)
                loop.estimate_normals()

                reg_p2p = o3d.pipelines.registration.registration_icp(source = source, 
                                                                      target = loop, 
                                                                      max_correspondence_distance = 0.5, 
                                                                      init = init_pose, 
                                                                      estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(), 
                                                                      criteria = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.001, max_iteration=20))
                loop_transform = reg_p2p.transformation

                PGM.addLoopFactor(loop_transform, loop_idx)
                PGM.optimizePoseGraph()
                ResultSaver.saveOptimizedPoseGraphResult(PGM.curr_node_idx, PGM.graph_optimized)

        ResultSaver.saveUnoptimizedPoseGraphResult(PGM.curr_se3, PGM.curr_node_idx) 
        
        if(curr_idx % num_frames_to_skip_to_show == 0): 
            ResultSaver.vizCurrentTrajectory(fig_idx, PGM.graph_initials)
            writer.grab_frame()
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


file1_path = 'curb_w_grass.npy'
file2_path = 'grid_water.npy'

class TerrainIdentification:
    def __init__(self, file_path):
        self.STEP_HEIGHT_THRESHOLD = 0.005  
        self.WINDOW_SIZE = 0.3  # Considering robot legs has a rectangular contact patch
        self.MIN_POINTS_IN_WINDOW = 5000 # This value can be changes according ot the window_size variable
        self.NORMALS_ANGLE_THRESHOLD = np.radians(15)  # Input is degrees
        self.file_path = file_path
        self.pcd = None  # Point Cloud

    def load_point_cloud(self):
        points = np.load(self.file_path)
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    def calculate_normals(self):
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    #filter steppable areas based on Normal angle Threshold
    def filter_steppable_areas(self):
        steppable_points = []
        for point, normal in zip(np.asarray(self.pcd.points), np.asarray(self.pcd.normals)):
            if abs(normal[2]) > np.cos(self.NORMALS_ANGLE_THRESHOLD): 
                steppable_points.append(point)
        return np.array(steppable_points)

    #filter steppable areas based on Height Mapping Threshold
    def filter_steppable_areas_by_height(self, points):
        steppable_points = []
        for i, point in enumerate(points):
            is_steppable = True
            for j in range(max(0, i - 1), min(len(points), i + 2)):
                if i != j and np.abs(points[i][2] - points[j][2]) > self.STEP_HEIGHT_THRESHOLD:
                    is_steppable = False
                    break
            if is_steppable:
                steppable_points.append(points[i])
        return np.array(steppable_points)
    
    #filter steppable areas based on desnity threshold in a rectangular window
    def filter_steppable_areas_by_window(self, points):
        # Using a KDTree for neighbor search
        tree = cKDTree(points[:, :2])  
        steppable_points = []

        for point in points:
            neighbors_idx = tree.query_ball_point(point[:2], r=self.WINDOW_SIZE)
            if len(neighbors_idx) >= self.MIN_POINTS_IN_WINDOW:
                steppable_points.append(point)
        
        return np.array(steppable_points)

    def run_identification(self):
        self.load_point_cloud()
        self.calculate_normals()

        steppable_areas = self.filter_steppable_areas() # Filter by Normal Angle

        steppable_areas = self.filter_steppable_areas_by_height(steppable_areas) # Filter by height difference

        safe_to_step_areas = self.filter_steppable_areas_by_window(steppable_areas) # Filter by sliding window

        # points back to a point cloud for visualizatoin
        steppable_point_cloud = o3d.geometry.PointCloud()
        steppable_point_cloud.points = o3d.utility.Vector3dVector(safe_to_step_areas)
        o3d.visualization.draw_geometries([steppable_point_cloud])

terrain_identifier1 = TerrainIdentification(file1_path)
terrain_identifier1.run_identification()

import os
import open3d as o3d

xyz = []

for root, dirs, files in os.walk("/mydir"):
    for file in files:
        with open(os.path.join(os.getcwd(), file), 'r') as f:
            for line in f:
                split_line = line.split(" ")
                xyz.append([split_line[0], split_line[1], split_line[2]])

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            o3d.io.write_point_cloud("{}.ply".format(os.path.join(root+"/ply/", file)), pcd)
from openalea.plantgl.all import *
import numpy as np

def load_points(filename):
    s = Scene(filename)
    points = s[0].geometry.pointList
    # points.translate(s[0].geometry.translation)
    return points


# Reconstruction of the mtg

try:
    import mtgmanip as mm
    from xumethod import xu_method
    import serial
except ImportError as ie:
    import openalea.plantscan3d.mtgmanip as mm
    from openalea.plantscan3d.xumethod import xu_method
    import openalea.plantscan3d.serial as serial


def find_root(points):
    center = points.getCenter()
    pminid, pmaxid = points.getZMinAndMaxIndex()
    zmin = points[pminid].z
    zmax = points[pmaxid].z
    initp = center
    initp.z = zmin
    return points.findClosest(initp)[0], zmin, zmax


def skeleton(points, binratio=10, k=20):
    mini, maxi = points.getZMinAndMaxIndex()
    root = Vector3(points[mini])

    mtg = mm.initialize_mtg(root)
    zdist = points[maxi].z - points[mini].z
    binlength = zdist / binratio

    vtx = list(mtg.vertices(mtg.max_scale()))
    startfrom = vtx[0]
    mtg = xu_method(mtg, startfrom, points, binlength, k)

    return mtg

points_file = load_points('M05_0325_a.txt.ply')
skeleton_mtg = skeleton(points_file)

print(type(skeleton_mtg))
child_count = {}
for i in range(2,skeleton_mtg.nb_vertices(scale=2)):
    parent = skeleton_mtg.__getitem__(i)['parent']
    if parent not in child_count:
        child_count[parent] = 1
    else:
        child_count[parent] += 1

branch_point_count = 0
branches = []
for i in child_count:
    if child_count[i] > 1:
        branches.append(i)
        branch_point_count += 1

# print(branches)
# print(branch_point_count)
branch_coords = []

for i in branches:
    branch_coords.append(skeleton_mtg.__getitem__(i)['position'])

branch_coords_converted = []
for i in branch_coords:
    branch_coords_converted.append([i.x, i.y, i.z])

coords=[]
for i in range(2,skeleton_mtg.nb_vertices(scale=2)):
    coord = skeleton_mtg.__getitem__(i)['position']
    coords.append(coord)

coords_converted = []
for i in coords:
    coords_converted.append([i.x, i.y, i.z])


# REMOVE BRANCH POINTS FROM ORIGINAL POINT CLOUD SO I CAN DISPLAY THEM ALONGSIDE ORIGINAL IN DIFFERENT COLOUR
for i in branch_coords_converted:
    coords_converted.remove(i)


# Find end points
all_vertices = list(range(2,skeleton_mtg.nb_vertices(scale=2)))
print(len(all_vertices))
for i in range(2,skeleton_mtg.nb_vertices(scale=2)):
    parent = skeleton_mtg.__getitem__(i)['parent']
    if parent is not None:
        try:
            all_vertices.remove(parent)
        except:
            pass

# DISPLAY
import open3d as o3d
pcd = o3d.geometry.PointCloud()
cnn_15 = np.asarray(coords_converted)
pcd.points = o3d.utility.Vector3dVector(cnn_15)
pcd.paint_uniform_color([0, 1, 0])

pcd2 = o3d.geometry.PointCloud()
cnn_15 = np.asarray(branch_coords_converted)
pcd2.points = o3d.utility.Vector3dVector(cnn_15)
pcd2.paint_uniform_color([1, 0, 0])

o3d.visualization.draw_geometries([pcd, pcd2])

Display test results

print("Number of branch points:", len(branch_coords_converted))
print("Number of end points:", len(all_vertices))
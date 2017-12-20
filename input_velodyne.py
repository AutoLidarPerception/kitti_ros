#!/usr/bin/env python
import sys
import os
import rospy
import numpy as np
import cv2
# python-pcl
import pcl
# for image show
import matplotlib.pyplot as plt 
import glob
import math
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
# detected box
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
# self-implemented XML parser
from parse_xml import parseXML

def load_pc_from_pcd(pcd_path):
    # Load PointCloud data from pcd file
    p = pcl.load(pcd_path)
    return np.array(list(p), dtype=np.float32)

def load_pc_from_bin(bin_path):
    # Load PointCloud data from bin file [X, Y, Z, I]
    obj = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    return obj

def read_label_from_txt(label_path):
    # Read label from txt file
    text = np.fromfile(label_path)
    bounding_box = []
    with open(label_path, "r") as f:
        labels = f.read().split("\n")
        for label in labels:
            if not label:
                continue
            label = label.split(" ")
            if (label[0] == "DontCare"):
                continue

            if label[0] == ("Car" or "Van"): #  or "Truck"
                bounding_box.append(label[8:15])

    if bounding_box:
        data = np.array(bounding_box, dtype=np.float32)
        return data[:, 3:6], data[:, :3], data[:, 6]
    else:
        return None, None, None

def read_label_from_xml(label_path):
    """
      Read label from xml file.

    # Returns:
        label_dic (dictionary): labels for one sequence.
            size (list): Bounding Box Size. [h, w, l]
            place (list): Bounding Box Position. [tx, ty, tz]
            rotate (list): Bounding Box Rotation. [rx, ry, rz]
        tracklet_counter: number of label(trajectory) for one sequence
    """
    labels = parseXML(label_path)
    label_dic = {}
    tracklet_counter = 0
    for label in labels:
        tracklet_counter += 1
        first_frame = label.firstFrame
        nframes = label.nFrames
        size = label.size
        obj_type = label.objectType
        for index, place, rotate in zip(range(first_frame, first_frame+nframes), label.trans, label.rots):
            if index in label_dic.keys():
                label_dic[index]["place"] = np.vstack((label_dic[index]["place"], place))
                label_dic[index]["size"] = np.vstack((label_dic[index]["size"], np.array(size)))
                label_dic[index]["rotate"] = np.vstack((label_dic[index]["rotate"], rotate))
            else:
                label_dic[index] = {}
                label_dic[index]["place"] = place
                label_dic[index]["rotate"] = rotate
                label_dic[index]["size"] = np.array(size)
    return label_dic, tracklet_counter

def read_calib_file(calib_path):
    # Read a calibration file
    data = {}
    with open(calib_path, 'r') as f:
        for line in f.readlines():
            if not line or line == "\n":
                continue
            key, value = line.split(':', 1)
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass
    return data

def proj_to_velo(calib_data):
    # Projection matrix to 3D axis for 3D Label
    rect = calib_data["R0_rect"].reshape(3, 3)
    velo_to_cam = calib_data["Tr_velo_to_cam"].reshape(3, 4)
    inv_rect = np.linalg.inv(rect)
    inv_velo_to_cam = np.linalg.pinv(velo_to_cam[:, :3])
    return np.dot(inv_velo_to_cam, inv_rect)


def filter_camera_angle(places):
    # Filter camera angles for KiTTI Datasets
    bool_in = np.logical_and((places[:, 1] < places[:, 0] - 0.27), (-places[:, 1] < places[:, 0] - 0.27))
    # bool_in = np.logical_and((places[:, 1] < places[:, 0]), (-places[:, 1] < places[:, 0]))
    return places[bool_in]

def create_publish_obj(obj, places, rotates, size):
    """Create object of correct data for publisher"""
    for place, rotate, sz in zip(places, rotates, size):
        x, y, z = place
        obj.append((x, y, z))
        h, w, l = sz
        if l > 10:
            continue
        for hei in range(0, int(h*100)):
            for wid in range(0, int(w*100)):
                for le in range(0, int(l*100)):
                    a = (x - l / 2.) + le / 100.
                    b = (y - w / 2.) + wid / 100.
                    c = (z) + hei / 100.
                    obj.append((a, b, c))
    return obj

def get_boxcorners(places, rotates, size):
    # Create 8 corners of bounding box from ground center
    corners = []
    for place, rotate, sz in zip(places, rotates, size):
        x, y, z = place
        h, w, l = sz
        if l > 10:
            continue

        corner = np.array([
            [x - l / 2., y - w / 2., z],
            [x + l / 2., y - w / 2., z],
            [x - l / 2., y + w / 2., z],
            [x - l / 2., y - w / 2., z + h],
            [x - l / 2., y + w / 2., z + h],
            [x + l / 2., y + w / 2., z],
            [x + l / 2., y - w / 2., z + h],
            [x + l / 2., y + w / 2., z + h],
        ])

        corner -= np.array([x, y, z])

        rotate_matrix = np.array([
            [np.cos(rotate), -np.sin(rotate), 0],
            [np.sin(rotate), np.cos(rotate), 0],
            [0, 0, 1]
        ])

        a = np.dot(corner, rotate_matrix.transpose())
        a += np.array([x, y, z])
        corners.append(a)
    return np.array(corners)

def publish_point_clouds(publisher, points):
    # Publish point clouds and bounding boxes
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"
    # points.shape = (?, 4)
    # points[:, :3] ==> (?, 0...2)
    msg_points = pc2.create_cloud_xyz32(header, points[:, :3])

    publisher.publish(msg_points)

def publish_bounding_boxes(publisher, boxes):
    # Publish bounding boxes
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"
    msg_boxes = pc2.create_cloud_xyz32(header, boxes)

    publisher.publish(msg_boxes)

def raw_to_voxel(pc, resolution=0.50, x=(0, 90), y=(-50, 50), z=(-4.5, 5.5)):
    """Convert PointCloud2 to Voxel"""
    logic_x = np.logical_and(pc[:, 0] >= x[0], pc[:, 0] < x[1])
    logic_y = np.logical_and(pc[:, 1] >= y[0], pc[:, 1] < y[1])
    logic_z = np.logical_and(pc[:, 2] >= z[0], pc[:, 2] < z[1])
    pc = pc[:, :3][np.logical_and(logic_x, np.logical_and(logic_y, logic_z))]
    pc =((pc - np.array([x[0], y[0], z[0]])) / resolution).astype(np.int32)
    voxel = np.zeros((int((x[1] - x[0]) / resolution), int((y[1] - y[0]) / resolution), int(round((z[1]-z[0]) / resolution))))
    voxel[pc[:, 0], pc[:, 1], pc[:, 2]] = 1
    return voxel

def center_to_sphere(places, size, resolution=0.50, min_value=np.array([0., -50., -4.5]), scale=4, x=(0, 90), y=(-50, 50), z=(-4.5, 5.5)):
    """Convert object label to Training label for objectness loss"""
    x_logical = np.logical_and((places[:, 0] < x[1]), (places[:, 0] >= x[0]))
    y_logical = np.logical_and((places[:, 1] < y[1]), (places[:, 1] >= y[0]))
    z_logical = np.logical_and((places[:, 2] < z[1]), (places[:, 2] >= z[0]))
    xyz_logical = np.logical_and(x_logical, np.logical_and(y_logical, z_logical))
    center = places.copy()
    center[:, 2] = center[:, 2] + size[:, 0] / 2.
    sphere_center = ((center[xyz_logical] - min_value) / (resolution * scale)).astype(np.int32)
    return sphere_center

def sphere_to_center(p_sphere, resolution=0.5, scale=4, min_value=np.array([0., -50., -4.5])):
    """from sphere center to label center"""
    center = p_sphere * (resolution*scale) + min_value
    return center

def voxel_to_corner(corner_vox, resolution, center):#TODO
    """Create 3D corner from voxel and the diff to corner"""
    corners = center + corner_vox
    return corners

def read_labels(label_path, label_type, calib_path=None, is_velo_cam=False, proj_velo=None):
    # Read labels from xml or txt file.
    if label_type == "txt": #TODO
        """
          Original Label value is shifted about 0.27m from object center.
          So need to revise the position of objects.
        """
        places, size, rotates = read_label_from_txt(label_path)
        if places is None:
            return None, None, None
        rotates = np.pi / 2 - rotates
        dummy = np.zeros_like(places)
        dummy = places.copy()
        if calib_path:
            places = np.dot(dummy, proj_velo.transpose())[:, :3]
        else:
            places = dummy
        if is_velo_cam:
            places[:, 0] += 0.27

    elif label_type == "xml":
        # bounding_boxes[frame index] 
        bounding_boxes, frame_counter = read_label_from_xml(label_path)
        #TODO dynamic index according to velodyne points file index
        # need to check boundary
        places = bounding_boxes[107]["place"]
        rotates = bounding_boxes[107]["rotate"][:, 2]
        size = bounding_boxes[107]["size"]

    return places, rotates, size

def create_label(places, size, corners, resolution=0.50, x=(0, 90), y=(-50, 50), z=(-4.5, 5.5), scale=4, min_value=np.array([0., -50., -4.5])):
    """Create training Labels which satisfy the range of experiment"""
    x_logical = np.logical_and((places[:, 0] < x[1]), (places[:, 0] >= x[0]))
    y_logical = np.logical_and((places[:, 1] < y[1]), (places[:, 1] >= y[0]))
    z_logical = np.logical_and((places[:, 2] + size[:, 0]/2. < z[1]), (places[:, 2] + size[:, 0]/2. >= z[0]))
    xyz_logical = np.logical_and(x_logical, np.logical_and(y_logical, z_logical))

    center = places.copy()
    center[:, 2] = center[:, 2] + size[:, 0] / 2. # Move bottom to center
    sphere_center = ((center[xyz_logical] - min_value) / (resolution * scale)).astype(np.int32)

    train_corners = corners[xyz_logical].copy()
    anchor_center = sphere_to_center(sphere_center, resolution=resolution, scale=scale, min_value=min_value) #sphere to center
    for index, (corner, center) in enumerate(zip(corners[xyz_logical], anchor_center)):
        train_corners[index] = corner - center
    return sphere_center, train_corners

def corner_to_train(corners, sphere_center, resolution=0.50, x=(0, 90), y=(-50, 50), z=(-4.5, 5.5), scale=4, min_value=np.array([0., -50., -4.5])):
    """Convert corner to Training label for regression loss"""
    x_logical = np.logical_and((corners[:, :, 0] < x[1]), (corners[:, :, 0] >= x[0]))
    y_logical = np.logical_and((corners[:, :, 1] < y[1]), (corners[:, :, 1] >= y[0]))
    z_logical = np.logical_and((corners[:, :, 2] < z[1]), (corners[:, :, 2] >= z[0]))
    xyz_logical = np.logical_and(x_logical, np.logical_and(y_logical, z_logical)).all(axis=1)
    train_corners = corners[xyz_logical].copy()
    sphere_center = sphere_to_center(sphere_center, resolution=resolution, scale=scale, min_value=min_value) #sphere to center
    for index, (corner, center) in enumerate(zip(corners[xyz_logical], sphere_center)):
        train_corners[index] = corner - center
    return train_corners

def corner_to_voxel(voxel_shape, corners, sphere_center, scale=4):
    """Create final regression label from corner"""
    corner_voxel = np.zeros((voxel_shape[0] / scale, voxel_shape[1] / scale, voxel_shape[2] / scale, 24))
    corner_voxel[sphere_center[:, 0], sphere_center[:, 1], sphere_center[:, 2]] = corners
    return corner_voxel

def create_objectness_label(sphere_center, resolution=0.5, x=90, y=100, z=10, scale=4):
    """Create Objectness label"""
    obj_maps = np.zeros((int(x / (resolution * scale)), int(y / (resolution * scale)), int(round(z / (resolution * scale)))))
    obj_maps[sphere_center[:, 0], sphere_center[:, 1], sphere_center[:, 2]] = 1
    return obj_maps

def process(velodyne_path, label_path=None, calib_path=None, dataformat="pcd", label_type="txt", is_velo_cam=False):
    p = []
    pc = None
    bounding_boxes = None
    places = None
    rotates = None
    size = None
    proj_velo = None

    if dataformat == "bin":
        pc = load_pc_from_bin(velodyne_path)
    elif dataformat == "pcd":
        pc = load_pc_from_pcd(velodyne_path)

    if calib_path:
        calib = read_calib_file(calib_path)
        proj_velo = proj_to_velo(calib)[:, :3]

    if label_path:
        '''
         read_label_from_xml()
           +size (list): Bounding Box Size. [h, w, l]
           +place (list): Bounding Box bottom center's Position. [tx, ty, tz]
           +rotate (list): Bounding Box bottom center's Rotation. [rx, ry, rz]
        '''
        places, rotates, size = read_labels(label_path, label_type, calib_path=calib_path, is_velo_cam=is_velo_cam, proj_velo=proj_velo)
    # Create 8 corners of bounding box from ground center
    corners = get_boxcorners(places, rotates, size)
    print("# of Point Clouds", len(pc))

    # Camera angle filters
    pc = filter_camera_angle(pc)
    # obj = []
    # obj = create_publish_obj(obj, places, rotates, size)

    p.append((0, 0, 0))
    p.append((0, 0, -1))
    print pc.shape
    print 1
    # publish_pc2(pc, obj)
    a = center_to_sphere(places, size, resolution=0.25)
    print places
    print a
    print sphere_to_center(a, resolution=0.25)
    bbox = sphere_to_center(a, resolution=0.25)
    print corners.shape
    # publish_pc2(pc, bbox.reshape(-1, 3))

    # publish point clouds & publish boxes 8 corners
    # One shape dimension can be -1. In this case, the value is inferred from the length of the array and remaining dimensions
    # reshape to 3 columns
    publish_pc2(pc, corners.reshape(-1, 3))

if __name__ == "__main__":
    # pcd_path = "../data/training/velodyne/000012.pcd"
    # label_path = "../data/training/label_2/000012.txt"
    # calib_path = "../data/training/calib/000012.txt"
    # process(pcd_path, label_path, calib_path=calib_path, dataformat="pcd")

    pcd_path = None
    bin_path = "./data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/"
    xml_path = "./data/2011_09_26/2011_09_26_drive_0001_sync/tracklet_labels.xml"
    calib_path = None
    # img_path = "./data/2011_09_26/2011_09_26_drive_0001_sync/image_0[0-3]/data/"
    img_path = "./data/2011_09_26/2011_09_26_drive_0001_sync/image_02/data/"

    datas = []
    if os.path.isdir(bin_path):
        for lists in os.listdir(bin_path):
            if os.path.isdir(lists):
                continue
            else:
                datas.append(lists)
    datas.sort()

    # bounding_boxes[frame index] 
    bounding_boxes, tracklet_counter = read_label_from_xml(xml_path)
    #TODO dynamic index according to velodyne points file index
    # need to check boundary

    rospy.init_node("kitti_publisher")
    # Publisher of PointCloud data
    pub_points = rospy.Publisher("/kitti/points_raw", PointCloud2, queue_size=1000000)
    # Publisher of bounding box corner vertex
    pub_boxes = rospy.Publisher("/kitti/points_corners", PointCloud2, queue_size=1000000)

    idx = 0
    # Rate(frequency)
    r = rospy.Rate(0.2)

    for data in datas:
        pc = load_pc_from_bin(bin_path+data)

        img_name = os.path.splitext(data)[0]+".png"
        img_file = cv2.imread(img_path+img_name)

        print("# of Point Clouds", len(pc))

        if calib_path:
            calib = read_calib_file(calib_path)
            proj_velo = proj_to_velo(calib)[:, :3]

        # Camera angle filters
        # pc = filter_camera_angle(pc)

        corners = None
        if idx in bounding_boxes.keys():
            places = bounding_boxes[idx]["place"]
            rotates = bounding_boxes[idx]["rotate"][:, 2]
            size = bounding_boxes[idx]["size"]
            # Create 8 corners of bounding box from ground center
            corners = get_boxcorners(places, rotates, size)
        else:
            print "no object in current frame: " + data

        publish_point_clouds(pub_points, pc)
        if corners is not None:
            publish_bounding_boxes(pub_boxes, corners.reshape(-1, 3))

        plt.figure(figsize=(16,6), frameon = False).patch.set_alpha(0)
        plt.title(img_name)
        plt.imshow(img_file)

        # frequency control by r.sleep()
        plt.pause(0.001)
        r.sleep()

        if rospy.is_shutdown():
            print "ros node had shutdown..."
            break

        idx += 1
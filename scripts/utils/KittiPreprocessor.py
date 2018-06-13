#!/usr/bin/env python

import numpy as np

# self-implemented XML parser
from XmlParser import XmlParser

class KittiPreprocessor(object):
    """
      Load PointCloud data from bin file [X, Y, Z, I]
    """
    @staticmethod
    def load_pc_from_bin(bin_path):
        obj = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
        return obj


    """
      Read label of care objects from xml file.
    
      Returns:
        label_dic (dictionary): labels for one sequence.
            size (list): Bounding Box Size. [h, w, l]
            place (list): Bounding Box Position. [tx, ty, tz]
            rotate (list): Bounding Box Rotation. [rx, ry, rz]
        tracklet_counter: number of label(trajectory) for one sequence
    """
    @staticmethod
    def read_label_from_xml(label_path, care_types):
        labels = XmlParser.parseXML(label_path)
        label_dic = {}
        tracklet_counter = 0
        for label in labels:
            obj_type = label.objectType
            care = False
            for care_type in care_types:
                if obj_type == care_type:
                    care = True
                    break
            if care:
                tracklet_counter += 1
                first_frame = label.firstFrame
                nframes = label.nFrames
                size = label.size
                for index, place, rotate in zip(range(first_frame, first_frame+nframes), label.trans, label.rots):
                    if index in label_dic.keys():
                        # array merged using vertical stack
                        label_dic[index]["place"] = np.vstack((label_dic[index]["place"], place))
                        label_dic[index]["size"] = np.vstack((label_dic[index]["size"], np.array(size)))
                        label_dic[index]["rotate"] = np.vstack((label_dic[index]["rotate"], rotate))
                    else:
                        # inited as array
                        label_dic[index] = {}
                        label_dic[index]["place"] = place
                        label_dic[index]["rotate"] = rotate
                        label_dic[index]["size"] = np.array(size)

        return label_dic, tracklet_counter


    """
      Filter camera angles for KiTTI Datasets
    """
    @staticmethod
    def filter_by_camera_angle(pc):
        bool_in = np.logical_and((pc[:, 1] < pc[:, 0] - 0.27), (-pc[:, 1] < pc[:, 0] - 0.27))
        """
        /*
         * @brief KiTTI Velodyne Coordinate
         *          |x(forward)
         *      C   |   D
         *          |
         *  y---------------
         *          |
         *      B   |   A
         */
        """
        # bool_in = np.where(pc[:, 0] > 0)
        return pc[bool_in]

    """
      Get 8 box corners from KiTTI 3D Oriented Bounding Box
    """
    @staticmethod
    def get_boxcorners(places, rotates_z, size):
        # Create 8 corners of bounding box from ground center
        if rotates_z.size <= 0:
            return None
        elif rotates_z.size == 1:
            x, y, z = places
            h, w, l = size
            rotate_z = rotates_z
            if l > 10:
                return None

            corner = np.array([
                [x - l / 2., y - w / 2., z],        #
                [x + l / 2., y - w / 2., z],        #
                [x + l / 2., y + w / 2., z],
                [x - l / 2., y + w / 2., z],
                [x - l / 2., y - w / 2., z + h],
                [x + l / 2., y - w / 2., z + h],
                [x + l / 2., y + w / 2., z + h],
                [x - l / 2., y + w / 2., z + h],
            ])

            corner -= np.array([x, y, z])

            rotate_matrix = np.array([
                [np.cos(rotate_z), -np.sin(rotate_z), 0],
                [np.sin(rotate_z), np.cos(rotate_z), 0],
                [0, 0, 1]
            ])

            a = np.dot(corner, rotate_matrix.transpose())
            a += np.array([x, y, z])

            return np.array(a)
        # rotates_z may be only one dimension
        else:
            corners = []
            for place, rotate_z, sz in zip(places, rotates_z, size):
                x, y, z = place
                h, w, l = sz
                if l > 10:
                    continue

                corner = np.array([
                    [x - l / 2., y - w / 2., z],        #
                    [x + l / 2., y - w / 2., z],        #
                    [x + l / 2., y + w / 2., z],
                    [x - l / 2., y + w / 2., z],
                    [x - l / 2., y - w / 2., z + h],
                    [x + l / 2., y - w / 2., z + h],
                    [x + l / 2., y + w / 2., z + h],
                    [x - l / 2., y + w / 2., z + h],
                ])

                corner -= np.array([x, y, z])

                rotate_matrix = np.array([
                    [np.cos(rotate_z), -np.sin(rotate_z), 0],
                    [np.sin(rotate_z), np.cos(rotate_z), 0],
                    [0, 0, 1]
                ])

                a = np.dot(corner, rotate_matrix.transpose())
                a += np.array([x, y, z])
                corners.append(a)
            # all corners
            return np.array(corners)
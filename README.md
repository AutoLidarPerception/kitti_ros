# kitti_ros
　　[KiTTI dataset](http://www.cvlibs.net/datasets/kitti/) process based-on ROS.


## TODO list
- [x] Publish `*.bin` Point Cloud in topic **/kitti/points_raw** _(sensor_msgs/PointCloud2)_.
- [x] Publish ground truth in `tracklet_labels.xml`.
    - [x] Publish `care_objects`' 3D OBB (Oriented bounding box) in topic **/kitti/bb_raw** _(geometry_msgs/PoseArray)_.
    - [x] Publish as well as **/kitti/bb_marker** _(visualization_msgs/MarkerArray)_ for visualization.
- [x] Publish `*.png` Image in topic **/kitti/img_raw** _(sensor_msgs/Image)_.
- [x] Publish `*.txt` Pose in tf between `imu_frame` and `world_frame`.
- [ ] Publish `*.txt` Calibration in tf between Coordinates.


## How to use
　We name your ros workspace as `CATKIN_WS` and `git clone` [kitti_ros](https://github.com/Durant35/kitti_ros) as a ros package.
```sh
# clone source code
$ cd $(CATKIN_WS)/src
$ git clone https://github.com/Durant35/kitti_ros

# build your ros workspace
$ cd $(CATKIN_WS)
$ catkin build -DCMAKE_BUILD_TYPE=Release

# launch kitti_ros's kitti_player
$ source devel/setup.bash
$ roslaunch kitti_ros kitti_player.launch
```


## [Parameters](./launch/kitti_player.launch)
+ `keyboard_file`: Keyboard listener is based on Linux input subsystem.
+ `fps`: default `10`Hz, the same as LiDAR frequency.
+ `kitti_data_path`: KiTTI raw data directory, like `.../2011_09_26_drive_0005_sync`
```yaml
.
├── 2011_09_26_drive_0005_sync
│   ├── image_00
│   │   ├── data
│   │   │   ├── 0000000xxx.png
│   │   │   ├── ...
│   │   └── timestamps.txt
│   ├── image_01
│   │   ├── data
│   │   │   ├── 0000000xxx.png
│   │   │   └── ...
│   │   └── timestamps.txt
│   ├── image_02
│   │   ├── data
│   │   │   ├── 0000000xxx.png
│   │   │   └── ...
│   │   └── timestamps.txt
│   ├── image_03
│   │   ├── data
│   │   │   ├── 0000000xxx.png
│   │   │   └── ...
│   │   └── timestamps.txt
│   ├── oxts
│   │   ├── data
│   │   │   ├── 0000000xxx.txt
│   │   │   └── ...
│   │   ├── dataformat.txt
│   │   └── timestamps.txt
│   ├── tracklet_labels.xml
│   └── velodyne_points
│       ├── data
│       │   ├── 0000000xxx.bin
│       │   └── xxx
│       ├── timestamps_end.txt
│       ├── timestamps_start.txt
│       └── timestamps.txt
├── 201?_??_??_drive_0???_sync
│   ├── ...
│   └── ...
├── calib_cam_to_cam.txt
├── calib_imu_to_velo.txt
└── calib_velo_to_cam.txt
```
+ `filter_by_camera_angle`: Only care about Camera's angle of view, default `true`.
+ `care_objects`: default `['Car','Van','Truck','Pedestrian','Sitter','Cyclist','Tram','Misc']`, `[]` means no forground objects.


## References
```bibtex
@article{geiger2013vision,
  title={Vision meets robotics: The KITTI dataset},
  author={Geiger, Andreas and Lenz, Philip and Stiller, Christoph and Urtasun, Raquel},
  journal={The International Journal of Robotics Research},
  volume={32},
  number={11},
  pages={1231--1237},
  year={2013},
  publisher={Sage Publications Sage UK: London, England}
}
```


## Thanks
+ [MarkMuth](https://github.com/MarkMuth)/[**QtKittiVisualizer**](https://github.com/MarkMuth/QtKittiVisualizer)
+ [yukitsuji](https://github.com/yukitsuji)/[**3D_CNN_tensorflow** ](https://github.com/yukitsuji/3D_CNN_tensorflow)
+ [strawlab](https://github.com/strawlab/python-pcl)/[**python-pcl** ](https://github.com/Durant35/python-pcl)
+ [**utiasSTARS/pykitti**](https://github.com/utiasSTARS/pykitti)



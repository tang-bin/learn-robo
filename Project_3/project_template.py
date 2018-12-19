#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

sample_index = 3
model_path = '/home/robond/catkin_ws/model_' + str(sample_index) + '.sav'
output_path = '/home/robond/catkin_ws/output_' + str(sample_index) + '.yaml'
scene_num = Int32
scene_num.data = sample_index

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:
    
    # TODO: Convert ROS msg to PCL data
    
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    
    outlier_filter = cloud.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(20)
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(0.1)
    cloud_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    
    passthrough_z = cloud_filtered.make_passthrough_filter()
    passthrough_z.set_filter_field_name("z")
    passthrough_z.set_filter_limits(0.6, 1.1)
    cloud_filtered = passthrough_z.filter()
    
    passthrough_y = cloud_filtered.make_passthrough_filter()
    passthrough_y.set_filter_field_name("y")
    passthrough_y.set_filter_limits(-0.45, 0.45)
    cloud_filtered = passthrough_y.filter()

    # TODO: RANSAC Plane Segmentation
    
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # TODO: Extract inliers and outliers
    
    inliers, coefficients = seg.segment()
    cloud_filtered = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    
    white_cloud = XYZRGB_to_XYZ(cloud_filtered) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(5000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])
                                         
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    
    print(cloud_filtered)
    pcl_msg = pcl_to_ros(cloud_filtered)    
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    
    pcl_objects_pub.publish(ros_cluster_cloud)
    #pcl_table_pub.publish(pcl_msg)


    # Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    
    detected_objects_labels = []
    detected_objects = []
    
    object_pos = []

    for index, pts_list in enumerate(cluster_indices):
    
        # Grab the points for the cluster
        pcl_cluster = cloud_filtered.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))
        object_pos.append(list(white_cloud[pts_list[0]]))

        # Add the detected object to the list of detected objects.
        
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
        
        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
         pr2_mover(detected_objects, object_pos)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list, object_pos):

    # TODO: Initialize variables
    
    output_list = []

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    
    for index, obj in enumerate(object_list):
        
        object_name = String()
        object_name.data = object_list_param[index]['name']

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        
        pp = object_pos[index]     
        pick_point = Point(float(pp[0]), float(pp[1]), float(pp[2]))        
        pick_pose = Pose(position=pick_point)
        
        # TODO: Assign the arm to be used for pick_place
        object_group = object_list_param[index]['group']  
        arm_name = String()
        if object_group == "green":
            arm_name.data = "right"
        else:
            arm_name.data = "left"

        # TODO: Create 'place_pose' for the object
        
        dropbox = rospy.get_param('/dropbox')
        if object_group == "green":
            dp = dropbox[1]['position']
        else:
            dp = dropbox[0]['position']
        drop_point = Point(float(dp[0]), float(dp[1]), float(dp[2]))
        place_pose = Pose(position=drop_point)


        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        
        output_list.append(make_yaml_dict(scene_num, object_name, arm_name, pick_pose, place_pose))
        
        # Wait for 'pick_place_routine' service to come up
        #rospy.wait_for_service('pick_place_routine')

        #try:
        #    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            # TODO: Insert your message variables to be sent as a service request
        #    resp = pick_place_routine(scene_num, object_name, arm_name, pick_pose, place_pose)

        #    print ("Response: ",resp.success)

        #except rospy.ServiceException, e:
        #    print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    
    send_to_yaml(output_path, output_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    
    rospy.init_node("clustering", anonymous=True)

    # TODO: Create Subscribers
    
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    #pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    
    model = pickle.load(open(model_path, 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    
    while not rospy.is_shutdown():
        rospy.spin()

#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud, PointCloud2, ChannelFloat32
from geometry_msgs.msg import Point32

import numpy as np
from sklearn.cluster import DBSCAN, MeanShift
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares


pointcloud_publisher = None
pointcloud_publisher2 = None


def c_func(w,m):
    return np.sign(np.cos(w)) * np.abs(np.cos(w))**m

def s_func(w,m):
    return np.sign(np.sin(w)) * np.abs(np.sin(w))**m

# superellipsoid -> x,y,z : https://en.wikipedia.org/wiki/Superellipsoid
def superellipsoid_to_xyz(A=1,B=1,C=1,E1=0.5,E2=0.3,custom_args=None):
    if custom_args is None:
        v = np.linspace(-np.pi/2, np.pi/2, 20)
        u = np.linspace(-np.pi, np.pi, 30)
    else:
        v = np.linspace(custom_args["v_min"], custom_args["v_max"], custom_args["v_count"])
        u = np.linspace(custom_args["u_min"], custom_args["u_max"], custom_args["u_count"])
    uu, vv = np.meshgrid(u,v)
    
    #r = 2    # from wikipedia
    #t = 2.5  # from wikipedia
    r = 2/E2  # from the paper
    t = 2/E1  # from the paper
    
    x = A * c_func(vv, 2/t) * c_func(uu, 2/r)
    y = B * c_func(vv, 2/t) * s_func(uu, 2/r)
    z = C * s_func(vv, 2/t)
    
    return x,y,z

def rotate_xyz(x,y,z,roll,pitch,yaw):
    datashape = x.shape
    r = R.from_rotvec(np.array([roll, pitch, yaw]))
    xyz = r.apply(np.array([x.flatten(),y.flatten(),z.flatten()]).T)
    x,y,z = xyz.T
    return x.reshape(datashape), y.reshape(datashape), z.reshape(datashape)

def translate_xyz(x,y,z,Tx,Ty,Tz):
    x += Tx
    y += Ty
    z += Tz
    return x,y,z

def loss(x0,x,y,z):
    a,b,c,e1,e2,tx,ty,tz,yaw,pitch,roll = x0

    e1, e2 = np.abs(e1), np.abs(e2)
    
    # https://otik.uk.zcu.cz/bitstream/11025/1637/1/D71.pdf
    x,y,z = rotate_xyz(x,y,z,roll,pitch,yaw)
    x,y,z = translate_xyz(x,y,z,tx,ty,tz)
    
    #x,y,z = x+0j, y+0j, z+0j
    x,y,z = np.abs(x), np.abs(y), np.abs(z)
    
    f = ( (x/a)**(2/e2) + (y/b)**(2/e2) )**(e2/e1) + (z/c)**(2/e1)
    #f = (np.sqrt(a*b*c) * (f**e1 - 1.))**2
    f = np.abs(a*b*c) * (f**e1 - 1.)**2
    
    return f


def callback(data):

    gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    gen_points = np.array(list(gen)).T


    # cluster data points
    db = DBSCAN(eps=0.02, min_samples=50).fit(gen_points.T)
    print(db.labels_)
    
    clusters = []
    for cluster_id in range(-1, max(db.labels_)+1):
        mask = db.labels_ == cluster_id
        clusters.append(gen_points[:,mask])

        # todo
        if cluster_id == 0:
            break


    # publish debug point cloud
    debug_pointcloud = PointCloud()
    ch1 = ChannelFloat32()
    ch1.name = "cluster_id"
    debug_pointcloud.channels.append(ch1)
    debug_pointcloud.header = data.header
    for cluster_id, cluster in enumerate(clusters):
        for x,y,z in cluster.T:
            debug_pointcloud.points.append(Point32(x, y, z))
            debug_pointcloud.channels[0].values.append(cluster_id)
    pointcloud_publisher.publish(debug_pointcloud)


    # publish debug point cloud (2)
    debug_pointcloud = PointCloud()
    debug_pointcloud.header = data.header
    for cluster_id, cluster in enumerate(clusters):
        if cluster_id < 0:
            continue

        x_, y_, z_ = clusters[cluster_id]
        x_mean, y_mean, z_mean = x_.mean(), y_.mean(), z_.mean()
        x_, y_, z_ = x_ - x_mean, y_ - y_mean, z_ - z_mean
        x_std, y_std, z_std = x_.std(), y_.std(), z_.std()
        w_std = np.mean((x_std, y_std, z_std))
        x_, y_, z_ = x_ / w_std, y_ / w_std, z_ / w_std

        x0 = (0.1,0.1,0.1,0.7,0.7,0,0,0,0,0,0)
        result = least_squares(loss, x0, args=(x_.flatten(), y_.flatten(), z_.flatten()), method="lm")
        #result = least_squares(loss, x0, args=(x_.flatten(), y_.flatten(), z_.flatten()))
        A,B,C,E1,E2,tx,ty,tz,yaw,pitch,roll = result["x"]

        #A,B,C,E1,E2,tx,ty,tz,yaw,pitch,roll = x0

        print("A:{},B:{},C:{},E1:{},E2:{},tx:{},ty:{},tz:{},yaw:{},pitch:{},roll:{}".format(A,B,C,E1,E2,tx,ty,tz,yaw,pitch,roll))

        if A < 0.1 or B < 0.1 or C < 0.1:
            continue

        # CALCULATE SUPERELLIPSOID
        xr, yr, zr = superellipsoid_to_xyz(A,B,C,E1,E2,custom_args=None)
        xr, yr, zr = rotate_xyz(xr, yr, zr,-roll,-pitch,-yaw)
        xr, yr, zr = translate_xyz(xr, yr, zr,-tx,-ty,-tz)

        xr, yr, zr = xr * w_std, yr * w_std, zr * w_std
        xr, yr, zr = xr + x_mean, yr + y_mean, zr + z_mean

        for x,y,z in zip(xr.flatten(), yr.flatten(), zr.flatten()):
            if np.abs(x)>1000 or np.abs(y)>1000 or np.abs(z)>1000:
                continue
            debug_pointcloud.points.append(Point32(x, y, z))

    pointcloud_publisher2.publish(debug_pointcloud)


    
def listener():
    global pointcloud_publisher, pointcloud_publisher2
    rospy.init_node('listener', anonymous=True)
    pointcloud_publisher = rospy.Publisher("/my_pointcloud_topic", PointCloud)
    pointcloud_publisher2 = rospy.Publisher("/my_pointcloud_topic2", PointCloud)
    rospy.Subscriber("/capsicum_superellipsoid_detector/pc_roi_out", PointCloud2, callback, queue_size=2)
    rospy.spin()

if __name__ == '__main__':
    listener()

#!/usr/bin/env python

import rospy
import rosparam
from mlr_msgs.msg import Point2dArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import cv2
import numpy as np
from cv_bridge import CvBridge

color_palette = [(231,  76,  60),
                 ( 46, 204, 113),
                 (155,  89, 182),
                 (241, 196,  15),
                 ( 52, 152, 219),
                 ( 52,  73,  94),
                 (230, 126,  34),
                 ( 26, 188, 156),
                 (224, 246,  53),
                 (149, 165, 166),
                 (245, 160, 167),
                 (218, 202, 143),
                 (141, 177, 115),
                 (199,  74, 108),
                 (101,  70,  65),
                 ( 54,  99, 120),
                 (146,  39,  41)]

class Object:
    """Object defines a set of point trajectories that move together. Available parameters are:
    pos: [[x0,y0],[x1,y1]] # key positions of the trajectory (specify at least 2 here)
    dim: [dx,dy] # dimensions of the object, specifies the initial measurement distribution
    n: n # number of measurement points, default: 8
    noise: sigma # tracker noise, applied in each frame, default: .001
    """
    def __init__(self, pos, dim=(.05,.05), n=8, noise=.001):
        self.dist = np.array(dim)
        self.x = np.array(pos[0])
        self.v = np.diff(pos,axis=0)*(len(pos)-1)
        self.forward()
        self.c = (220,220,220)
        self.points = self.x + np.random.rand(n,len(self.x))*2.*dim-dim
        self.err = lambda: np.random.randn(n,len(self.x))*noise

    def forward(self):
        self.mode = "forward"
        self.t = 0
        self.dx = lambda t,dt : dt*self.v[int(t*len(self.v))]
    def backward(self):
        self.mode = "backward"
        self.t = 0
        self.dx = lambda t,dt : -dt*self.v[int((1.-t)*len(self.v))]
    def toggle(self):
        if self.mode == "forward": self.backward()
        elif self.mode == "backward": self.forward()
        return self
    def setColor(self, c):
        self.c = c
        return self

    def update(self, t):
        v = self.dx(t,t-self.t)
        self.x += v
        self.points += self.err()+v
        self.t = t
        return self

    def draw(self, img):
        if self.dist.shape == (2,):
            p1 = np.array((self.x-self.dist)*(img.shape[1],img.shape[0]),np.int)
            p2 = np.array((self.x+self.dist)*(img.shape[1],img.shape[0]),np.int)
            cv2.rectangle(img, tuple(p1-(2,2)), tuple(p2+(2,2)), (0,0,0),3)
            cv2.rectangle(img, tuple(p1), tuple(p2), self.c,3)
        return self


class Simulation:
    """Simulation represents the world, configuration parameters are:
    duration: 16 # duration for the specified trajectories in seconds
    rate: 15 # hz of publishing images and trajectories
    resolution: [640,480] # image resolution the above dimensions are scaled to
    suppress_points: False # set True if you only want the image published
    """
    def __init__(self, duration, rate=15, resolution=(640,480), suppress_points=False):
        global color_palette

        self.pub_image = rospy.Publisher("camera/rgb/image_color",Image,queue_size=1)
        self.pub_points = rospy.Publisher("tracking/lk2d/points",Point2dArray,queue_size=1)
        self.br = CvBridge()
        self.start = rospy.Time.now()
        self.duration = rospy.Duration.from_sec(duration)
        self.rate = rospy.Rate(rate)
        self.resolution = resolution
        self.suppress = suppress_points
        self.msg = Point2dArray()
        self.msg.header.frame_id = 'camera_rgb_optical_frame'
        self.msg.scale_x = 1./self.resolution[0]
        self.msg.scale_y = 1./self.resolution[1]

    def set_objects(self, objs):
        self.objects = objs
        map(lambda o,c : o.setColor(c), self.objects, color_palette[:len(objs)])
        self.msg.ids = range(np.sum(map(lambda o: len(o.points), objs)))
        return self

    def run(self):
        print("Running...")
        while not rospy.is_shutdown():
            img = np.ones([self.resolution[1],self.resolution[0],3],np.uint8)*255
            dt = (rospy.Time.now() - self.start).to_sec()/self.duration.to_sec()
            if dt >= 1.:
                dt, self.start = 0, rospy.Time.now()
                map(lambda o : o.toggle(), self.objects)
                continue

            map(lambda o: o.update(dt).draw(img), self.objects)
            self.msg.header.stamp = rospy.Time.now()
            self.msg.x,self.msg.y = \
              map(list,np.vstack([ self.resolution*o.points for o in self.objects ]).T)
            img_msg = self.br.cv2_to_imgmsg(img,'rgb8')
            img_msg.header = self.msg.header
            self.pub_image.publish(img_msg)
            self.rate.sleep()
            if not self.suppress:
                self.pub_points.publish(self.msg)


if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2 :
        print("Please specify exactly one simulation scenario config yaml file. Thank you.")
        exit(0)

    rospy.init_node('simulation2d_node')
    param,ns = rosparam.load_file(sys.argv[1])[0]
    try:
        objects = [ Object(**p) for p in param['objects'] ]
    except NameError as e:
        print("Couldn't instantiate objects.")
        print("Something seems wrong with the objects section of your yaml file:")
        print("\t%s"%e)
        exit(0)
    except TypeError as e:
        print("Couldn't instantiate objects.")
        print("Something seems wrong with the objects section of your yaml file:")
        print("\t%s"%e)
        print(Object.__doc__)
        exit(0)
    except:
        print(sys.exc_info())
        raise
    print("Object creation successful. (%s objects loaded)" % len(objects))

    try:
        sim = Simulation(**param['world'])
    except NameError as e:
        print("Couldn't instantiate simulation.")
        print("Something seems wrong with the world section of your yaml file.")
        print("\t%s"%e)
        exit(0)
    except TypeError as e:
        print("Couldn't instantiate simulation.")
        print("Something seems wrong with the world section of your yaml file.")
        print("\t%s"%e)
        print(Simulation.__doc__)
        exit(0)
    except:
        print(sys.exc_info())
        raise

    sim.set_objects(objects)
    print("Simulator creation successful.")
    print("Default output points [mlr_msgs::Point2dArray], topic is: tracking/lk2d/points")
    print("Default output images [sensor_msgs::Image], topic is: camera/rgb/image_color")
    try:
        sim.run()
    except rospy.ROSInterruptException:
        pass
    

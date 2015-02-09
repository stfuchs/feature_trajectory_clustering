#!/usr/bin/env python

import rospy
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
    def __init__(self, pos, dist=(.05,.05), n=8, noise=.001):
        self.dist = np.array(dist)
        self.x = np.array(pos[0])
        self.v = np.diff(pos,axis=0)*(len(pos)-1)
        self.forward()
        self.c = (220,220,220)
        self.points = self.x + np.random.rand(n,len(self.x))*2.*dist-dist
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
    def __init__(self, duration, objects, dims=(640,480)):
        global color_palette

        self.pub_image = rospy.Publisher("camera/rgb/image_color",Image,queue_size=1)
        self.pub_points = rospy.Publisher("tracking/lk2d/points",Point2dArray,queue_size=1)
        self.br = CvBridge()
        self.start = rospy.Time.now()
        self.duration = duration
        self.objects = objects
        map(lambda o,c : o.setColor(c), self.objects, color_palette[:len(objects)])
        self.dims = dims
        self.msg = Point2dArray()
        self.msg.header.frame_id = 'camera_rgb_optical_frame'
        self.msg.scale_x = 1./self.dims[0]
        self.msg.scale_y = 1./self.dims[1]
        self.msg.ids = range(np.sum(map(lambda o: len(o.points), objects)))

    def run(self, rate):
        while not rospy.is_shutdown():
            img = np.ones([self.dims[1],self.dims[0],3],np.uint8)*255
            dt = (rospy.Time.now() - self.start).to_sec()/self.duration.to_sec()
            if dt >= 1.:
                dt, self.start = 0, rospy.Time.now()
                map(lambda o : o.toggle(), self.objects)
                continue

            map(lambda o: o.update(dt).draw(img), self.objects)
            self.msg.header.stamp = rospy.Time.now()
            self.msg.x,self.msg.y = map(list,np.vstack([ self.dims*o.points for o in self.objects ]).T)
            img_msg = self.br.cv2_to_imgmsg(img,'rgb8')
            img_msg.header = self.msg.header
            self.pub_image.publish(img_msg)
            rate.sleep()
            self.pub_points.publish(self.msg)


if __name__ == '__main__':
    rospy.init_node('simulation2d_node')

    objects1 = [Object(pos=[(.5,.5),(.5,.5)], dist=(.45,.45), n=20, noise=.0005),
                Object(pos=[(.3,.3),(.5,.5),(.5,.5),(.8,.3)], dist=(.1,.05), n=10, noise=.001),
                Object(pos=[(.8,.1),(.8,.8)], dist=(.05,.05), n=5, noise=.001)]

    objects2 = [Object(pos=[(.5,.5),(.5,.5)], dist=(.45,.45), n=50, noise=.0005),
                Object(pos=[(.1,.2),(.3,.2),(.5,.2),(.7,.2),(.9,.2),(.9,.2),(.9,.2),(.9,.2)],n=20),
                Object(pos=[(.1,.4),(.1,.4),(.3,.4),(.5,.4),(.7,.4),(.9,.4),(.9,.4),(.9,.4)],n=20),
                Object(pos=[(.1,.6),(.1,.6),(.1,.6),(.3,.6),(.5,.6),(.7,.6),(.9,.6),(.9,.6)],n=20),
                Object(pos=[(.1,.8),(.1,.8),(.1,.8),(.1,.8),(.3,.8),(.5,.8),(.7,.8),(.9,.8)],n=20)]

    sim = Simulation(rospy.Duration.from_sec(16), objects2)

    try:
        sim.run(rospy.Rate(15))
    except rospy.ROSInterruptException:
        pass
    

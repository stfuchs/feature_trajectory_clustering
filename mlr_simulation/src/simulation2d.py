#!/usr/bin/env python

import rospy
import rosparam
from mlr_msgs.msg import Point2dArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Bool
import cv2
import numpy as np
from numpy import pi
from cv_bridge import CvBridge

class ColorPalette(object):
    c = [(231,  76,  60),
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

class Newid(object):
    count = -1
    @staticmethod
    def get():
        Newid.count+=1
        return Newid.count

class MP(object):
    profiles = {
        "sin"  : lambda s: .5*(1.-np.cos(pi*s)),
        "ramp" : lambda s: s
    }

class Entity(object):
    """Entity defines a set of point trajectories that move together. Available parameters are:

    pos: [[x0,y0],[x1,y1]] list of key positions, linear interpolation in between
    angle: [alpha0, alpha1] list of key angles, linear interpolation in between
        both key position and angles define the object frame

    mp_pos,mp_angle: string or list of strings, ["ramp","sin"], specify motion profile
        used for interpolation between above key points. If list of strings then:
        len(mp_[pos|angle])==len(ankor_[pos|angle])-1

    ext_low,ext_high: set the size of the entity by defining the lower and upper extention point,
        with respect to object frame. This defines where trajectories will be spawned. 

    n: number of measurement points at the beginning
    noise: tracker noise, applied in each frame
    birth: probability of spawning new trajectory (default=0)
    death: probability of deleting existing trajectory (default=0)
    """
    
    def __init__(self, pos, angle=[0,0], mp_pos="ramp", mp_angle="ramp",
                 ext_low=[0,0], ext_high=[.1,.1], n=1, noise=0, birth=0, death=0):
        
        self.bb = np.vstack([ ext_low, np.array([ext_low[0],ext_high[1]]),
                              ext_high, np.array([ext_high[0],ext_low[1]]) ]).T

        self.size = self.bb[:,2] - self.bb[:,0]
        self.points = np.random.rand(2,n)*self.size.reshape(2,1)+self.bb[:,0].reshape(2,1)
        self.ids = [ Newid.get() for i in range(n) ]
        self.Tx = len(pos)-1
        self.Ta = len(angle)-1
        self.X = np.mat(pos).T
        eval_angle = lambda s: eval(s) if isinstance(s,str) else s
        self.A = map(eval_angle,angle)
        
        if isinstance(mp_pos,list):
            self.mpx = [ MP.profiles[mp] for mp in mp_pos ]
        else:
            self.mpx = [ MP.profiles[mp_pos] for i in range(self.Tx) ]
            
        if isinstance(mp_angle,list):
            self.mpa = [ MP.profiles[mp] for mp in mp_angle ]
        else:
            self.mpa = [ MP.profiles[mp_angle] for i in range(self.Ta) ]
            
        self.sigma = noise
        self.birth = birth
        self.death = death
        self.birth_count = 1
        self.death_count = 1
        self.update_count = 1

    def set_color(self, c):
        self.c = c
        return self
    def set_rate(self, r):
        self.r = r
        return self

    def transform_to_image(self, X, R, t, s):
        res = np.array(R*X+t)
        res[1,:] = 1.-res[1,:] # reflect y-axis for image domain
        return (res*s).astype(int)
    
    def update(self, t):
        tx0 = int(t*self.Tx)
        ta0 = int(t*self.Ta)
        dx = self.mpx[tx0](t*self.Tx-tx0)*(self.X[:,tx0+1] - self.X[:,tx0])
        da = self.mpa[ta0](t*self.Ta-ta0)*(self.A[ta0+1] - self.A[ta0])
        self.x = self.X[:,tx0]+dx
        self.a = self.A[ta0]+da
        self.R = np.mat([ [np.cos(self.a), -np.sin(self.a)],[np.sin(self.a),np.cos(self.a)] ])
        lp = list(self.points.T)
        if len(lp)>1 and np.random.rand() <= self.death*self.r:
            idx = int(np.random.rand()*len(lp))
            del lp[idx]
            del self.ids[idx]
            #self.death_count += 1
        if np.random.rand() <= self.birth*self.r:
            self.ids.append(Newid().get())
            lp.append(np.random.rand(2)*self.size+self.bb[:,0])
            #self.birth_count += 1
        #self.update_count += 1
        #if (self.death_count%100)==0 or (self.birth_count%100)==0:
        #    print("birth rate: %s\ndeath rate: %s"%
        #          (float(self.birth_count)/self.update_count,
        #           float(self.death_count)/self.update_count))
        self.points = np.array(lp).T + np.random.randn(2,len(lp))*self.sigma
        return self
        
    def draw(self, img):
        S = np.array( [[img.shape[1]],[img.shape[0]]] )
        bb = self.transform_to_image(self.bb,self.R,self.x,S)
        for i in range(bb.shape[1]):
            cv2.line(img,tuple(bb[:,i]),tuple(bb[:,(i+1)%bb.shape[1]]),self.c,3,cv2.CV_AA)
        self.tracks = self.transform_to_image(self.points,self.R,self.x,S)
        
        

class World(object):
    """World represents the world, configuration parameters are:

    duration: 16 # duration for the specified trajectories in seconds
    rate: 15 # hz of publishing images and trajectories
    resolution: [640,480] # image resolution the above dimensions are scaled to
    """

    def __init__(self, duration, rate=15, resolution=(640,480)):
        self.pub_image = rospy.Publisher("/camera/rgb/image_color",Image,queue_size=1)
        self.pub_points = rospy.Publisher("lk2d/points",Point2dArray,queue_size=1)
        self.br = CvBridge()
        self.duration = float(duration)
        self.end = rospy.Time.now().to_sec()+self.duration
        self.forward = lambda: 1.-(self.end - rospy.Time.now().to_sec())/self.duration
        self.backward = lambda: (self.end - rospy.Time.now().to_sec())/self.duration
        self.time = self.forward
        self.rate = rospy.Rate(rate)
        self.inv_rate = 1./float(rate)
        self.resolution = resolution
        self.msg = Point2dArray()
        self.msg.header.frame_id = 'camera_rgb_optical_frame'
        self.msg.scale_x = 1./self.resolution[0]
        self.msg.scale_y = 1./self.resolution[1]

    def set_entities(self, ents):
        self.entities = ents
        map(lambda o,c : o.set_color(c), self.entities, ColorPalette.c[:len(ents)])
        map(lambda o: o.set_rate(self.inv_rate), self.entities)
        return self

    def spin_once(self):
        img = np.ones([self.resolution[1],self.resolution[0],3],np.uint8)*255
        t = self.time()
        if t >= 1.:
            self.end = rospy.Time.now().to_sec()+self.duration
            self.time = self.backward
            self.rate.sleep()
            return
        if t <= 0.:
            self.end = rospy.Time.now().to_sec()+self.duration
            self.time = self.forward
            self.rate.sleep()
            return

        map(lambda o: o.update(t).draw(img), self.entities)
        self.msg.header.stamp = rospy.Time.now()
        self.msg.x,self.msg.y = map(list,np.vstack([e.tracks.T for e in self.entities]).T)
        self.msg.ids = list(np.hstack([e.ids for e in self.entities]))
        img_msg = self.br.cv2_to_imgmsg(img,'rgb8')
        img_msg.header = self.msg.header
        self.pub_image.publish(img_msg)
        self.rate.sleep()
        self.pub_points.publish(self.msg)

class Simulator(object):
    def __init__(self):
        self.sub = rospy.Subscriber("reset_all", Bool, 
                                    self.stop, queue_size=1)
        self.has_stopped = True
        self.reconfigure()

    def run(self):
        while not self.has_stopped and not rospy.is_shutdown():
            self.world.spin_once()

    def stop(self, msg):
        self.has_stopped = msg.data
        print("Stopped! Trying reconfigure...")
        self.reconfigure()

    def reconfigure(self):
        try:
            yaml = rospy.get_param('/tracking/scenario')
        except KeyError:
            print("could not find parameter: tracking/scenario on server")
            print("Waiting for reset message (/tracking/reset_all) ...")
            self.has_stopped = True
            return

        try:
            param,ns = rosparam.load_file(yaml)[0]
        except rosparam.RosParamException:
            print("Failed to load %s" %yaml)
            print("Waiting for reset message (/tracking/reset_all) ...")
            self.has_stopped = True
            return
        print("Loaded %s" %yaml)
        try:
            self.entities = [ Entity(**p) for p in param['entities'] ]
        except NameError as e:
            print("Couldn't instantiate entities.")
            print("Something seems wrong with the entities section of your yaml file:")
            print("\t%s"%e)
            print("Waiting for reset message (/tracking/reset_all) ...")
            self.has_stopped = True
            return
        except TypeError as e:
            print("Couldn't instantiate entities.")
            print("Something seems wrong with the entities section of your yaml file:")
            print("\t%s"%e)
            print(Entity.__doc__)
            print("Waiting for reset message (/tracking/reset_all) ...")
            self.has_stopped = True
            return
        except:
            print(sys.exc_info())
            raise
            print("Entity creation successful. (%s entities loaded)" % len(self.entities))

        try:
            self.world = World(**param['world'])
        except NameError as e:
            print("Couldn't instantiate simulation.")
            print("Something seems wrong with the world section of your yaml file.")
            print("\t%s"%e)
            print("Waiting for reset message (/tracking/reset_all) ...")
            self.has_stopped = True
            return
        except TypeError as e:
            print("Couldn't instantiate simulation.")
            print("Something seems wrong with the world section of your yaml file.")
            print("\t%s"%e)
            print(World.__doc__)
            print("Waiting for reset message (/tracking/reset_all) ...")
            self.has_stopped = True
            return
        except:
            print(sys.exc_info())
            raise

        self.has_stopped = False
        print("Simulator creation successful.")
        print("Default output points [mlr_msgs::Point2dArray], topic is: tracking/lk2d/points")
        print("Default output images [sensor_msgs::Image], topic is: camera/rgb/image_color")
        self.world.set_entities(self.entities)
        print("Running...")

if __name__ == '__main__':
    import sys
    rospy.init_node('simulation2d_node')

    if len(sys.argv) != 2:
        if not rospy.has_param('tracking/scenario'):
            print("Please specify exactly one simulation scenario config yaml file.")
            print("You can provide one via command line argument or by setting a parameter")
            print("on the rosparam server. (rosparam set /tracking/scenario /full/path/to.yaml)")
    else:
        rospy.set_param('/tracking/scenario', sys.argv[1])
    sim = Simulator()

    try:
        while not rospy.is_shutdown():
            sim.run()
            rospy.Rate(2).sleep() # waiting for reconfigure
    except rospy.ROSInterruptException:
        pass
    

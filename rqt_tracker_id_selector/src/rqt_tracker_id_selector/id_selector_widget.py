import os
import glob
import rospy
import rospkg

from mlr_msgs.msg import Point2dArray, Point3dArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtGui import QWidget, QTableWidgetItem, QIcon

import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import defaultdict
import yaml

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


class IdSelectorWidget(QWidget):

    def __init__(self):
        super(IdSelectorWidget, self).__init__()
        self.setObjectName('IdSelectorWidget')

        # Get path to UI file which should be in the "resource" folder of this package
        pkg_path = rospkg.RosPack().get_path('rqt_tracker_id_selector')
        ui_file = os.path.join(pkg_path, 'resource', 'IdSelector.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self.labels = ["ID","label"]
        self.id_table.setColumnCount(len(self.labels))
        self.id_table.setHorizontalHeaderLabels(self.labels)
        self.lineEdit.setText("~/labels.yaml")

        self.sub_traj = rospy.Subscriber("tracking/lk2d/points", Point2dArray,
                                         self.cb_points, queue_size=10)
        self.sub_traj = rospy.Subscriber("tracking/lk3d/points", Point3dArray,
                                         self.cb_points3d, queue_size=10)
        self.sub_img = rospy.Subscriber("/camera/rgb/image_color", Image,
                                        self.cb_img, queue_size=1)
        self.pub_img = rospy.Publisher("tracking/selected_tracks",Image,queue_size=1)
        self.pub_marker = rospy.Publisher("trajectory_marker",MarkerArray,queue_size=1)
        
        self.bridge = CvBridge()
        self.img = None
        
        self.ids = {}
        self.tracks = defaultdict(list)
        self.tracks3d = defaultdict(list)
        self.select = []

    def get(self, i,j):
        return int(self.id_table.item(i,j).text())

    def clear_markers(self):
        ma = MarkerArray()
        ma.markers = [ Marker() for tid in self.tracks3d.keys() ]
        for tid,i in zip(self.tracks3d.keys(), range(len(self.tracks3d))):
            ma.markers[i].header.frame_id = "camera_rgb_optical_frame"
            ma.markers[i].ns = "line"
            ma.markers[i].id = tid
            ma.markers[i].action = 2
        self.pub_marker.publish(ma)
        for i in range(len(self.tracks3d)):
            ma.markers[i].ns = "point"
        self.pub_marker.publish(ma)


    def add_marker(self, array, track, color, tid):
        m1 = Marker()
        m1.header.frame_id = "camera_rgb_optical_frame"
        m1.ns = "line"
        m1.id = tid
        m1.type = 4 #lines
        m1.action = 0
        m1.scale.x = .002
        m1.color.a = 1.
        m1.color.r = color[0]/255.
        m1.color.g = color[1]/255.
        m1.color.b = color[2]/255.
        m1.points = track
        array.append(m1)
        m2 = Marker()
        m2.header.frame_id = "camera_rgb_optical_frame"
        m2.ns = "point"
        m2.id = tid
        m2.type = 8 #points
        m2.action = 0
        m2.scale.x = .008
        m2.scale.y = .008
        m2.color.a = 1.
        m2.color.r = color[0]/255.
        m2.color.g = color[1]/255.
        m2.color.b = color[2]/255.
        m2.points = [ track[-1] ]
        array.append(m2)

    def update_image(self):
        if self.img is None:
            return
        img = self.img.copy()
        self.clear_markers()
        ma = MarkerArray()
        for s in self.id_table.selectedItems():
            if s.column() is not 0: continue
            t = self.tracks[int(s.text())]
            if self.checkLabels.isChecked():
                c1 = ColorPalette.c[self.get(s.row(),1)]
                c2 = c1
            else:
                c1 = ColorPalette.c[0]
                c2 = (220,220,220)
            for i in range(1,len(t)):
                if self.checkLines.isChecked():
                    cv2.line(img,t[i-1],t[i],c1,1,cv2.CV_AA)
                if self.radioAll.isChecked():
                    cv2.circle(img,t[i],5,c2,1,cv2.CV_AA)
            if self.radioLast.isChecked():
                cv2.circle(img,t[-1],5,c2,1,cv2.CV_AA)
            self.add_marker(ma.markers, self.tracks3d[int(s.text())], c1, int(s.text()))
        msg = self.bridge.cv2_to_imgmsg(img,'rgb8')
        self.pub_marker.publish(ma)
        self.pub_img.publish(msg)

    def cb_img(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg,'rgb8')

    def cb_points3d(self, p):
        for i in range(len(p.ids)):
            mp = Point()
            mp.x = p.x[i]
            mp.y = p.y[i]
            mp.z = p.z[i]
            self.tracks3d[p.ids[i]].append( mp )

    def cb_points(self, p):
        for i in range(len(p.ids)):
            self.tracks[p.ids[i]].append( (int(p.x[i]),int(p.y[i])) )
            if p.ids[i] not in self.ids:
                row = self.id_table.rowCount()
                self.id_table.insertRow(row)
                i0 = QTableWidgetItem()
                i1 = QTableWidgetItem()
                i0.setData(Qt.DisplayRole,p.ids[i])
                i1.setData(Qt.DisplayRole,0)
                i0.setFlags(i0.flags() ^ Qt.ItemIsEditable)
                self.id_table.setItem(row,0,i0)
                self.id_table.setItem(row,1,i1)
                self.ids[p.ids[i]] = [i0,i1]

    @Slot()
    def on_incrButton_clicked(self):
        for s in self.id_table.selectedItems():
            if s.column() is not 0: continue
            li = self.id_table.item(s.row(),1)
            li.setData(Qt.DisplayRole, int(li.text())+1)
        self.update_image()
        
    @Slot()
    def on_decrButton_clicked(self):
        for s in self.id_table.selectedItems():
            if s.column() is not 0: continue
            li = self.id_table.item(s.row(),1)
            li.setData(Qt.DisplayRole, int(li.text())-1)
        self.update_image()

    @Slot()
    def on_clearButton_clicked(self):
        self.ids = {}
        self.select = []
        self.id_table.setRowCount(0)
        self.tracks = defaultdict(list)


    @Slot()
    def on_saveButton_clicked(self):
        d = defaultdict(list)
        for i in range(self.id_table.rowCount()):
            d[self.get(i,1)].append(self.get(i,0))
        try:
            f = open(os.path.expanduser(self.lineEdit.text()),'w')
            yaml.dump(dict(d),f)
            f.close()
        except:
            print("failed to dump to file %s"%self.lineEdit.text())

    @Slot()
    def on_loadButton_clicked(self):
        try:
            f = open(os.path.expanduser(self.lineEdit.text()),'r')
            d = yaml.load(f)
            f.close()
            for oid,tids in d.items():
                for tid in tids:
                    if tid in self.ids:
                        self.ids[tid][1].setData(Qt.DisplayRole,oid)
        except:
            print("failed to load file %s"%self.lineEdit.text())

    @Slot()
    def on_id_table_itemSelectionChanged(self):
        self.update_image()

    @Slot(bool)
    def on_radioAll_toggled(self, state):
        self.update_image()

    @Slot(bool)
    def on_radioLast_toggled(self, state):
        self.update_image()
        
    @Slot(int)
    def on_checkLines_stateChanged(self, state):
        self.update_image()

    @Slot(int)
    def on_checkLabels_stateChanged(self, state):
        self.update_image()

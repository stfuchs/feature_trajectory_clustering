import os
import glob
import rospy
import rospkg

from mlr_msgs.msg import Point2dArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtGui import QWidget, QTableWidgetItem, QIcon

import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import defaultdict

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
        self.lineEdit.setText(os.path.expanduser("~"))

        self.sub_traj = rospy.Subscriber("tracking/lk2d/points", Point2dArray,
                                         self.cb_points, queue_size=1)
        self.sub_img = rospy.Subscriber("/camera/rgb/image_color", Image,
                                        self.cb_img, queue_size=1)
        self.pub_img = rospy.Publisher("tracking/selected_tracks",Image,queue_size=1)
        
        self.bridge = CvBridge()
        self.img = None
        
        self.ids = {}
        self.tracks = defaultdict(list)
        self.select = []

    def cb_img(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg,'rgb8')

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
    def on_showButton_clicked(self):
        for s in self.select:
            print("Id: %s, track length: %s"%(s,len(self.tracks[s])))
        print(" ")
        if self.img is None:
            return
        img = self.img.copy()
        for s in self.select:
            t = self.tracks[s]
            for i in range(1,len(t)):
                cv2.line(img,t[i-1],t[i],(231, 76, 60))
        msg = self.bridge.cv2_to_imgmsg(img,'rgb8')
        self.pub_img.publish(msg)

    @Slot()
    def on_clearButton_clicked(self):
        self.ids = {}
        self.select = []
        self.id_table.setRowCount(0)
        self.tracks = defaultdict(list)


    @Slot()
    def on_saveButton_clicked(self):
        pass

    @Slot()
    def on_id_table_itemSelectionChanged(self):
        self.select = []
        for i in self.id_table.selectedItems():
            if i.column() is 0:
                self.select.append(int(i.text()))
                #self.select.append(int(self.id_table.item(i.row(),1).text()))
            
    @Slot(int)
    def on_comboBox_activated(self, index):
        pass

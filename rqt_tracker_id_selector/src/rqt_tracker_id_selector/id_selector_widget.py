import os
import glob
import rospy
import rospkg

from mlr_msgs.msg import KernelState, Point2dArray, ObjectIds
from std_msgs.msg import Float64, Bool

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtGui import QWidget, QTableWidgetItem, QIcon

import numpy as np

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
        self.startButton.setIcon(QIcon.fromTheme('media-playback-start'))
        self.stopButton.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.refreshButton.setIcon(QIcon.fromTheme('view-refresh'))
        wildcard = os.path.join(rospkg.RosPack().get_path('mlr_simulation'),'yaml','*.yaml')
        self.files = glob.glob(wildcard)
        self.fnames = map(os.path.basename, self.files)
        map(self.comboBox.addItem, self.fnames)


        self.sub_traj = rospy.Subscriber("tracking/lk2d/points", Point2dArray,
                                         self.cb_points, queue_size=1)
        self.sub_kernel = rospy.Subscriber("tracking/kernel", KernelState,
                                           self.cb_kernel, queue_size=1, buff_size=2**24)
        self.sub_obj = rospy.Subscriber("tracking/objects", ObjectIds,
                                        self.cb_obj, queue_size=1)

        self.pub_d = rospy.Publisher("tracking/monitor/d", Float64, queue_size=1)
        self.pub_k = rospy.Publisher("tracking/monitor/k", Float64, queue_size=1)
        self.pub_reset = rospy.Publisher("tracking/reset_all", Bool, queue_size=1)
        self.ids = {}
        self.select = []

    def cb_points(self, points):
        if len(self.select) != 2:
            return
        try:
            idx1 = points.ids.index(self.select[0])
            idx2 = points.ids.index(self.select[1])
        except ValueError:
            print("Selected ids %s not in Point2dArray message" % self.select)
            return
        msg = Float64()
        msg.data = (points.x[idx1]*points.scale_x - points.x[idx2]*points.scale_x)**2
        msg.data+= (points.y[idx1]*points.scale_y - points.y[idx2]*points.scale_y)**2
        msg.data = np.sqrt(msg.data)
        self.pub_d.publish(msg)

    def cb_kernel(self, kernel_state):
        n = len(kernel_state.ids)
        new_ids = map( int, np.array(kernel_state.ids) >> 32 )
        M = np.zeros([n,n],np.float32)
        M[np.triu_indices(n,1)] = kernel_state.data
        K = M+M.T+np.diag(np.ones(n,dtype=np.float32))

        for i in new_ids:
            if i not in self.ids:
                row = self.id_table.rowCount()
                self.id_table.insertRow(row)
                i0 = QTableWidgetItem()
                i1 = QTableWidgetItem()
                i0.setData(Qt.DisplayRole,i)
                i1.setData(Qt.DisplayRole,0)
                self.id_table.setItem(row,0,i0)
                self.id_table.setItem(row,1,i1)
                self.ids[i] = [i0,i1]
        if len(self.select) == 2:
            try:
                msg = Float64()
                msg.data = K[new_ids.index(self.select[0]), new_ids.index(self.select[1])]
                self.pub_k.publish(msg)
            except ValueError:
                print("Selected ids %s not in Kernel message" % self.select)
                return

    def cb_obj(self, objects):
        return
        for i,l in zip(objects.ids,objects.labels):
            if i in self.ids:
                self.ids[i>>32][1].setData(Qt.DisplayRole,l)

    @Slot()
    def on_startButton_clicked(self):
        self.ids = {}
        self.select = []
        self.id_table.clear()
        self.id_table.setRowCount(0)
        rospy.set_param('/tracking/scenario', self.files[self.comboBox.currentIndex()])
        msg = Bool()
        msg.data = True
        self.pub_reset.publish(msg)

    @Slot()
    def on_stopButton_clicked(self):
        try:
            rospy.delete_param('/tracking/scenario')
        except KeyError:
            pass
        msg = Bool()
        msg.data = True
        self.pub_reset.publish(msg)

    @Slot()
    def on_refreshButton_clicked(self):
        while self.comboBox.count() != 0:
            self.comboBox.removeItem(0)

        wildcard = os.path.join(rospkg.RosPack().get_path('mlr_simulation'),'yaml','*.yaml')
        self.files = glob.glob(wildcard)
        self.fnames = map(os.path.basename, self.files)
        map(self.comboBox.addItem, self.fnames)        

    @Slot()
    def on_id_table_itemSelectionChanged(self):
        items = self.id_table.selectedItems()
        self.select = [ int(i.text()) for i in items if i.column() is 0 ]
            
    @Slot(int)
    def on_comboBox_activated(self, index):
        pass

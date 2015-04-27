import os
import glob
import rospy
import rospkg

from std_msgs.msg import Bool
from mlr_msgs.msg import SimControl
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtGui import QWidget, QIcon

import numpy as np

class SimcontrolWidget(QWidget):

    def __init__(self):
        super(SimcontrolWidget, self).__init__()
        self.setObjectName('SimcontrolWidget')

        # Get path to UI file which should be in the "resource" folder of this package
        pkg_path = rospkg.RosPack().get_path('rqt_tracker_simcontrol')
        ui_file = os.path.join(pkg_path, 'resource', 'Simcontrol.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self.startButton.setIcon(QIcon.fromTheme('media-playback-start'))
        self.stopButton.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.pauseButton.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.nextButton.setIcon(QIcon.fromTheme('media-skip-forward'))
        self.refreshButton.setIcon(QIcon.fromTheme('view-refresh'))
        wildcard = os.path.join(rospkg.RosPack().get_path('mlr_simulation'),'yaml','*.yaml')
        self.files = glob.glob(wildcard)
        self.files.sort()
        self.fnames = map(os.path.basename, self.files)
        map(self.comboBox.addItem, self.fnames)

        self.pub_reset = rospy.Publisher("tracking/reset_all", Bool, queue_size=1)
        self.pub_control = rospy.Publisher("tracking/sim_control", SimControl, queue_size=1)


    @Slot()
    def on_startButton_clicked(self):
        rospy.set_param('/tracking/scenario', self.files[self.comboBox.currentIndex()])
        msg = SimControl()
        msg.mode ="play"
        self.pub_control.publish(msg)

    @Slot()
    def on_pauseButton_clicked(self):
        msg = SimControl()
        msg.mode = "pause"
        self.pub_control.publish(msg)

    @Slot()
    def on_nextButton_clicked(self):
        msg = SimControl()
        msg.mode = "next"
        self.pub_control.publish(msg)
        
    @Slot()
    def on_stopButton_clicked(self):
        try:
            rospy.delete_param('/tracking/scenario')
        except KeyError:
            pass
        msg1 = SimControl()
        msg1.mode = "stop"
        self.pub_control.publish(msg1)
        
        msg2 = Bool()
        msg2.data = True
        self.pub_reset.publish(msg2)

    @Slot()
    def on_refreshButton_clicked(self):
        while self.comboBox.count() != 0:
            self.comboBox.removeItem(0)

        wildcard = os.path.join(rospkg.RosPack().get_path('mlr_simulation'),'yaml','*.yaml')
        self.files = glob.glob(wildcard)
        self.files.sort()
        self.fnames = map(os.path.basename, self.files)
        map(self.comboBox.addItem, self.fnames)
            
    @Slot(int)
    def on_comboBox_activated(self, index):
        pass

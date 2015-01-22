import os
import rospy
import rospkg

from mlr_msgs.msg import KernelState

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtGui import QWidget, QTableWidgetItem

class IdSelectorWidget(QWidget):

    def __init__(self):
        super(IdSelectorWidget, self).__init__()
        self.setObjectName('IdSelectorWidget')

        # Get path to UI file which should be in the "resource" folder of this package
        pkg_path = rospkg.RosPack().get_path('rqt_tracker_id_selector')
        ui_file = os.path.join(pkg_path, 'resource', 'IdSelector.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self.labels = ["ID","count","info"]
        self.id_table.setColumnCount(len(self.labels))
        self.id_table.setHorizontalHeaderLabels(self.labels)
        
        self.subscribe()

    def subscribe(self):
        topic = self.topic_text.text()
        self.ids = {}
        self.id_table.clear()
        self.id_table.setHorizontalHeaderLabels(self.labels)
        try:
            self.sub = rospy.Subscriber(topic, KernelState, self.lk_callback, queue_size=1)
        except:
            item = QTableWidgetItem()
            item.setText('Subscription to %s failed'%topic)
            row = self.id_table.rowCount()
            self.id_table.insertRow(row)
            self.id_table.setItem(row,2,item)
            print('Subscription to %s failed'%topic)


    def lk_callback(self, kernel_state):
        for i in kernel_state.ids:
            if i in self.ids:
                count = self.ids[i][1].data(Qt.DisplayRole) + 1
                self.ids[i][1].setData(Qt.DisplayRole,count)
            else:
                row = self.id_table.rowCount()
                self.id_table.insertRow(row)
                i0 = QTableWidgetItem()
                i1 = QTableWidgetItem()
                i0.setData(Qt.DisplayRole,i)
                i1.setData(Qt.DisplayRole,1)
                self.id_table.setItem(row,0,i0)
                self.id_table.setItem(row,1,i1)
                self.ids[i] = [i0,i1]

    @Slot()
    def on_load_button_clicked(self):
        self.subscribe()
        return

    @Slot()
    def on_id_table_itemSelectionChanged(self):
        items = self.id_table.selectedItems()
        ids = [ int(i.text()) for i in items if i.column() is 0 ]
        print(ids)
            
        

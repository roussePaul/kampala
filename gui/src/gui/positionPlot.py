import os
import rospy
import QtGui
import QtCore
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal
import pyqtgraph as pg
import analysis
import utils
import subprocess
from mavros_msgs.msg import OverrideRCIn
from mocap.msg import QuadPositionDerived


class positionPlotPlugin(Plugin):

    
    Update = pyqtSignal(list,list,pg.PlotDataItem)

    UpdateV = pyqtSignal()
    
    def __init__(self, context):
        super(positionPlotPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('positionPlotPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        
        
        
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'positionPlot.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('positionPlotUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        #Setting coordinate independent variables

        

        self.plotwidget = pg.PlotWidget()
    

        self.velplotwidget = pg.PlotWidget()
    

        self.channelplotwidget = pg.PlotWidget()

        self.time_offset = rospy.get_time()
        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3','iris4','iris5'])
        self.id = rospy.get_param('/body_array',[1,2,3,4,5])[self._widget.IrisInputBox.currentIndex()]


        self.plotwidget.getPlotItem().addLegend()
        self.velplotwidget.getPlotItem().addLegend()
        self.channelplotwidget.getPlotItem().addLegend()

        layout = QtGui.QGridLayout()
        vellayout = QtGui.QGridLayout()
        channellayout = QtGui.QGridLayout()
        layout.addWidget(self.plotwidget)
        vellayout.addWidget(self.velplotwidget)
        channellayout.addWidget(self.channelplotwidget)

        self.plotitem = self.plotwidget.getPlotItem()
        self.velplotitem = self.velplotwidget.getPlotItem()
        self.channelplotitem = self.channelplotwidget.getPlotItem()

        self.plotitem.setLabel('left','position','m')
        self.plotitem.setLabel('bottom','time','s')
        self.velplotitem.setLabel('left','speed','m/s')
        self.velplotitem.setLabel('bottom','time','s')



        self._widget.frame.setLayout(layout)
        self._widget.frame_2.setLayout(vellayout)
        self._widget.frame_3.setLayout(channellayout)
        self.timevector = [0]*100
        self.dtimevector = [0]*100
        self.channeltimevector = [0]*100
        self.sub = ''
        self.dsub = ''
        self.channelsub = ''
        self.updatetimer = rospy.Timer(rospy.Duration(0.1), self.updatecallback)

        
        #Setting variables for each coordinate and channel
        
        self.Xplotvector = [0]*100
        self.Xcurve = self.plotitem.plot(self.timevector,self.Xplotvector, name='x')
        self.Xcurve.setPen(pg.mkPen('r'))
        
        self.Yplotvector = [0]*100
        self.Ycurve = self.plotitem.plot(self.timevector,self.Yplotvector, name='y')
        self.Ycurve.setPen(pg.mkPen('g'))

        self.Zplotvector = [0]*100
        self.Zcurve = self.plotitem.plot(self.timevector,self.Zplotvector, name='z')
        self.Zcurve.setPen(pg.mkPen('y'))

        self.XDesplotvector = [0]*100
        self.XDescurve = self.plotitem.plot(self.dtimevector,self.XDesplotvector, name='x<sub>d</sub>')
        self.XDescurve.setPen(pg.mkPen(color=(100,0,0)))

        self.YDesplotvector = [0]*100
        self.YDescurve = self.plotitem.plot(self.dtimevector,self.YDesplotvector, name='y<sub>d</sub>')
        self.YDescurve.setPen(pg.mkPen(color=(0,100,0)))

        self.ZDesplotvector = [0]*100
        self.ZDescurve = self.plotitem.plot(self.dtimevector,self.ZDesplotvector, name='z<sub>d</sub>')
        self.ZDescurve.setPen(pg.mkPen(color=(255,255,100)))

        self.Xvelplotvector = [0]*100
        self.Xvelcurve = self.velplotitem.plot(self.timevector,self.Xvelplotvector, name='v<sub>x</sub>')
        self.Xvelcurve.setPen(pg.mkPen('r'))
        
        self.Yvelplotvector = [0]*100
        self.Yvelcurve = self.velplotitem.plot(self.timevector,self.Yvelplotvector, name='v<sub>y</sub>')
        self.Yvelcurve.setPen(pg.mkPen('g'))

        self.Zvelplotvector = [0]*100
        self.Zvelcurve = self.velplotitem.plot(self.timevector,self.Zvelplotvector, name='v<sub>z</sub>')
        self.Zvelcurve.setPen(pg.mkPen('y'))

        self.XDesvelplotvector = [0]*100
        self.XDesvelcurve = self.velplotitem.plot(self.dtimevector,self.XDesvelplotvector, name='v<sub>xd</sub>')
        self.XDesvelcurve.setPen(pg.mkPen(color=(100,0,0)))

        self.YDesvelplotvector = [0]*100
        self.YDesvelcurve = self.velplotitem.plot(self.dtimevector,self.YDesvelplotvector, name='v<sub>yd</sub>')
        self.YDesvelcurve.setPen(pg.mkPen(color=(0,100,0)))

        self.ZDesvelplotvector = [0]*100
        self.ZDesvelcurve = self.velplotitem.plot(self.dtimevector,self.ZDesvelplotvector, name='v<sub>zd</sub>')
        self.ZDesvelcurve.setPen(pg.mkPen(color=(255,255,100)))

        self.channel1plotvector = [0]*100
        self.channel1curve = self.channelplotitem.plot(self.channeltimevector,self.channel1plotvector, name='channel1')
        self.channel1curve.setPen(pg.mkPen(color=(255,0,0)))

        self.channel2plotvector = [0]*100
        self.channel2curve = self.channelplotitem.plot(self.channeltimevector,self.channel2plotvector, name='channel2')
        self.channel2curve.setPen(pg.mkPen(color=(0,255,0)))

        self.channel3plotvector = [0]*100
        self.channel3curve = self.channelplotitem.plot(self.channeltimevector,self.channel3plotvector, name='channel3')
        self.channel3curve.setPen(pg.mkPen(color=(0,0,255),width=3))

        self.channel4plotvector = [0]*100
        self.channel4curve = self.channelplotitem.plot(self.channeltimevector,self.channel4plotvector, name='channel4')
        self.channel4curve.setPen(pg.mkPen(color=(100,100,100),style=QtCore.Qt.DashLine))

        #Connecting slots to signals
        self._widget.startButton.clicked.connect(self.setQuad)
        self.Update.connect(self.UpdatePlot)
        self.UpdateV.connect(self.UpdateView)

        # Defining quicker references to the functions used to see if the checkboxes for each curve are checked,
        # to shorten lookup time.
        
        self.xcheck = self._widget.Xcheck.isChecked
        self.ycheck = self._widget.Ycheck.isChecked
        self.zcheck = self._widget.Zcheck.isChecked

        self.xdcheck = self._widget.XDescheck.isChecked
        self.ydcheck = self._widget.YDescheck.isChecked
        self.zdcheck = self._widget.ZDescheck.isChecked

        self.vxcheck = self._widget.V_XCheck.isChecked
        self.vycheck = self._widget.V_YCheck.isChecked
        self.vzcheck = self._widget.V_ZCheck.isChecked

        self.vxdcheck = self._widget.V_XDescheck.isChecked
        self.vydcheck = self._widget.V_YDescheck.isChecked
        self.vzdcheck = self._widget.V_ZDescheck.isChecked

        self.ch1check = self._widget.channel1check.isChecked
        self.ch2check = self._widget.channel2check.isChecked
        self.ch3check = self._widget.channel3check.isChecked
        self.ch4check = self._widget.channel4check.isChecked
       


    
    def mocapcallback(self,data):

        self.timevector[:-1] = self.timevector[1:]
        self.timevector[-1] = self.getTime()

        self.Xplotvector[:-1] = self.Xplotvector[1:]
        self.Xplotvector[-1] = data.x
        self.Yplotvector[:-1] = self.Yplotvector[1:]
        self.Yplotvector[-1] = data.y
        self.Zplotvector[:-1] = self.Zplotvector[1:]
        self.Zplotvector[-1] = data.z

        self.Xvelplotvector[:-1] = self.Xvelplotvector[1:]
        self.Xvelplotvector[-1] = data.x_vel
        self.Yvelplotvector[:-1] = self.Yvelplotvector[1:]
        self.Yvelplotvector[-1] = data.y_vel
        self.Zvelplotvector[:-1] = self.Zvelplotvector[1:]
        self.Zvelplotvector[-1] = data.z_vel
        

    def rcoverridecallback(self,data):

        self.channeltimevector[:-1] = self.channeltimevector[1:]
        self.channeltimevector[-1] = self.getTime()

        self.channel1plotvector[:-1] = self.channel1plotvector[1:]
        self.channel1plotvector[-1] = data.channels[0]
        self.channel2plotvector[:-1] = self.channel2plotvector[1:]
        self.channel2plotvector[-1] = data.channels[1]
        self.channel3plotvector[:-1] = self.channel3plotvector[1:]
        self.channel3plotvector[-1] = data.channels[2]
        self.channel4plotvector[:-1] = self.channel4plotvector[1:]
        self.channel4plotvector[-1] = data.channels[3]

        

    def targetcallback(self,data):

        self.dtimevector[:-1] = self.dtimevector[1:]
        self.dtimevector[-1] = self.getTime()

        self.XDesplotvector[:-1] = self.XDesplotvector[1:]
        self.XDesplotvector[-1] = data.x
        self.YDesplotvector[:-1] = self.YDesplotvector[1:]
        self.YDesplotvector[-1] = data.y
        self.ZDesplotvector[:-1] = self.ZDesplotvector[1:]
        self.ZDesplotvector[-1] = data.z

        self.XDesvelplotvector[:-1] = self.XDesvelplotvector[1:]
        self.XDesvelplotvector[-1] = data.x_vel
        self.YDesvelplotvector[:-1] = self.YDesvelplotvector[1:]
        self.YDesvelplotvector[-1] = data.y_vel
        self.ZDesvelplotvector[:-1] = self.ZDesvelplotvector[1:]
        self.ZDesvelplotvector[-1] = data.z_vel

    def updatecallback(self,timereventdummyargument):

        self.UpdateV.emit()

        if self.xcheck():
            self.Update.emit(self.Xplotvector,self.timevector,self.Xcurve)           
        if self.ycheck():
            self.Update.emit(self.Yplotvector,self.timevector,self.Ycurve)
        if self.zcheck():
            self.Update.emit(self.Zplotvector,self.timevector,self.Zcurve)
    
        if self.vxcheck():
            self.Update.emit(self.Xvelplotvector,self.timevector,self.Xvelcurve)
        if self.vycheck():
            self.Update.emit(self.Yvelplotvector,self.timevector,self.Yvelcurve)
        if self.vzcheck():
            self.Update.emit(self.Zvelplotvector,self.timevector,self.Zvelcurve)

        if self.ch1check():
            self.Update.emit(self.channel1plotvector,self.channeltimevector,self.channel1curve)
        if self.ch2check():
            self.Update.emit(self.channel2plotvector,self.channeltimevector,self.channel2curve)
        if self.ch3check():
            self.Update.emit(self.channel3plotvector,self.channeltimevector,self.channel3curve)
        if self.ch4check():
            self.Update.emit(self.channel4plotvector,self.channeltimevector,self.channel4curve)

        if self.xdcheck():
            self.Update.emit(self.XDesplotvector,self.dtimevector,self.XDescurve)
        if self.ydcheck():
            self.Update.emit(self.YDesplotvector,self.dtimevector,self.YDescurve)
        if self.zdcheck():
            self.Update.emit(self.ZDesplotvector,self.dtimevector,self.ZDescurve)
        
        if self.vxdcheck():
            self.Update.emit(self.XDesvelplotvector,self.dtimevector,self.XDesvelcurve)
        if self.vydcheck():
            self.Update.emit(self.YDesvelplotvector,self.dtimevector,self.YDesvelcurve)
        if self.vzdcheck():
            self.Update.emit(self.ZDesvelplotvector,self.dtimevector,self.ZDesvelcurve)

    def UpdatePlot(self,vector,timevector,curve):
        curve.setData(timevector,vector)

    def UpdateView(self):
        self.plotitem.setXRange(self.timevector[0],self.timevector[-1])
        self.velplotitem.setXRange(self.timevector[0],self.timevector[-1])
        self.channelplotitem.setXRange(self.channeltimevector[0],self.channeltimevector[-1])

    def setQuad(self):
        self.id = rospy.get_param('/body_array',[1,2,3,4,5])[self._widget.IrisInputBox.currentIndex()]
        if self.sub != '':
            self.sub.unregister()
        self.sub = rospy.Subscriber('/body_data/id_' + str(self.id) , QuadPositionDerived, self.mocapcallback)
        if self.channelsub != '':
            self.channelsub.unregister()
        self.channelsub = rospy.Subscriber('/' + self._widget.IrisInputBox.currentText() + '/mavros/rc/override', OverrideRCIn, self.rcoverridecallback)
        if self.dsub != '':
            self.dsub.unregister()
        self.dsub = rospy.Subscriber('/' + self._widget.IrisInputBox.currentText() + '/trajectory_gen/target', QuadPositionDerived, self.targetcallback)

    def getTime(self):
        time = rospy.get_time() - self.time_offset
        return time

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    

import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal
import pyqtgraph as pg

import analysis
import utils
import subprocess
from mavros.msg import OverrideRCIn
from mocap.msg import QuadPositionDerived


class positionPlotPlugin(Plugin):


    Xon = pyqtSignal(float)
    Yon = pyqtSignal(float)
    Zon = pyqtSignal(float)
    Xoff = pyqtSignal()
    Yoff = pyqtSignal() 
    Zoff = pyqtSignal()
    Xvelon = pyqtSignal(float)
    Yvelon = pyqtSignal(float)
    Zvelon = pyqtSignal(float)
    Xveloff = pyqtSignal()
    Yveloff = pyqtSignal()
    Zveloff = pyqtSignal()



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

        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3','iris4','iris5'])
        self.id = rospy.get_param('/body_array',[1,2,3,4,5])[self._widget.IrisInputBox.currentIndex()]

        plotwidget = pg.PlotWidget()
        plotwidget.getPlotItem().addLegend()

        velplotwidget = pg.PlotWidget()
        velplotwidget.getPlotItem().addLegend()

        channelplotwidget = pg.PlotWidget()
        channelplotwidget.getPlotItem().addLegend()
        

        layout = QtGui.QGridLayout()
        vellayout = QtGui.QGridLayout()
        channellayout = QtGui.QGridLayout()
        layout.addWidget(plotwidget)
        vellayout.addWidget(velplotwidget)
        channellayout.addWidget(channelplotwidget)

        plotwidget.getPlotItem().setLabel('left','position','m')
        plotwidget.getPlotItem().setLabel('bottom','time','s')
        velplotwidget.getPlotItem().setLabel('left','speed','m/s')
        velplotwidget.getPlotItem().setLabel('bottom','time','s')



        self._widget.frame.setLayout(layout)
        self._widget.frame_2.setLayout(vellayout)
        self._widget.frame_3.setLayout(channellayout)
        self.timevector = [0]*100
        self.sub = ''
        self.channelsub = ''

        
        #Setting variables for each coordinate and channel
        
        self.Xplotvector = [0]*100
        self.Xcurve = plotwidget.getPlotItem().plot(self.timevector,self.Xplotvector, name='x')
        self.Xcurve.setPen(pg.mkPen('r'))
        
        self.Yplotvector = [0]*100
        self.Ycurve = plotwidget.getPlotItem().plot(self.timevector,self.Yplotvector, name='y')
        self.Ycurve.setPen(pg.mkPen('g'))

        self.Zplotvector = [0]*100
        self.Zcurve = plotwidget.getPlotItem().plot(self.timevector,self.Zplotvector, name='z')
        self.Zcurve.setPen(pg.mkPen('b'))

        self.Xvelplotvector = [0]*100
        self.Xvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Xvelplotvector, name='v<sub>x</sub>')
        self.Xvelcurve.setPen(pg.mkPen('r'))
        
        self.Yvelplotvector = [0]*100
        self.Yvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Yvelplotvector, name='v<sub>y</sub>')
        self.Yvelcurve.setPen(pg.mkPen('g'))

        self.Zvelplotvector = [0]*100
        self.Zvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Zvelplotvector, name='v<sub>z</sub>')
        self.Zvelcurve.setPen(pg.mkPen('b'))

        self.channel1plotvector = [0]*100
        self.channel1curve = channelplotwidget.getPlotItem().plot(self.channel1plotvector)

        #Connecting slots to signals
        self._widget.startButton.clicked.connect(self.setQuad)
        self.Xon.connect(self.XonUpdate)
        self.Yon.connect(self.YonUpdate)
        self.Zon.connect(self.ZonUpdate)
        self.Xoff.connect(self.XoffUpdate)
        self.Yoff.connect(self.YoffUpdate)
        self.Zoff.connect(self.ZoffUpdate)
        self.Xvelon.connect()
        
       


    def callback(self,data):

        self.timevector[:-1] = self.timevector[1:]
        self.timevector[-1] = self.timevector[-2] + data.time_diff
        

        if self._widget.Xcheck.isChecked():
            self.Xon.emit(data.x)
        else:
            self.Xoff.emit()
                      
        if self._widget.Ycheck.isChecked():
            self.Yon.emit(data.y)
        else:
            self.Yoff.emit()

        if self._widget.Zcheck.isChecked():
            self.Zon.emit(data.z)
        else:
            self.Zoff.emit()

            

        if self._widget.V_XCheck.isChecked():
            self.Xvelplotvector[:-1] = self.Xvelplotvector[1:]
            self.Xvelplotvector[-1] = data.x_vel
            self.Xvelcurve.setData(self.timevector,self.Xvelplotvector)
        else:
            self.Xvelplotvector[:-1] = self.Xvelplotvector[1:]
            self.Xvelplotvector[-1] = 0.0
            self.Xvelcurve.setData(self.timevector,self.Xvelplotvector)


        if self._widget.V_YCheck.isChecked():
            self.Yvelplotvector[:-1] = self.Yvelplotvector[1:]
            self.Yvelplotvector[-1] = data.y_vel
            self.Yvelcurve.setData(self.timevector,self.Yvelplotvector)
        else:
            self.Yvelplotvector[:-1] = self.Yvelplotvector[1:]
            self.Yvelplotvector[-1] = 0.0
            self.Yvelcurve.setData(self.timevector,self.Yvelplotvector)

        if self._widget.V_ZCheck.isChecked():
            self.Zvelplotvector[:-1] = self.Zvelplotvector[1:]
            self.Zvelplotvector[-1] = data.z_vel
            self.Zvelcurve.setData(self.timevector,self.Zvelplotvector)
        else:
            self.Zvelplotvector[:-1] = self.Zvelplotvector[1:]
            self.Zvelplotvector[-1] = 0.0
            self.Zvelcurve.setData(self.timevector,self.Zvelplotvector)

    def XonUpdate(self,data):
        self.Xplotvector[:-1] = self.Xplotvector[1:]
        self.Xplotvector[-1] = data
        self.Xcurve.setData(self.timevector,self.Xplotvector)

    def YonUpdate(self,data):
        self.Yplotvector[:-1] = self.Yplotvector[1:]
        self.Yplotvector[-1] = data
        self.Ycurve.setData(self.timevector,self.Yplotvector)

    def ZonUpdate(self,data):
        self.Zplotvector[:-1] = self.Zplotvector[1:]
        self.Zplotvector[-1] = data
        self.Zcurve.setData(self.timevector,self.Zplotvector)

    def XoffUpdate(self):
        self.Xplotvector[:-1] = self.Xplotvector[1:]
        self.Xplotvector[-1] = 0.0
        self.Xcurve.setData(self.timevector,self.Xplotvector)

    def YoffUpdate(self):
        self.Yplotvector[:-1] = self.Yplotvector[1:]
        self.Yplotvector[-1] = 0.0
        self.Ycurve.setData(self.timevector,self.Yplotvector)

    def ZoffUpdate(self):
        self.Zplotvector[:-1] = self.Zplotvector[1:]
        self.Zplotvector[-1] = 0.0
        self.Zcurve.setData(self.timevector,self.Zplotvector)





    def callback2(self,data):
        
        self.channel1plotvector[:-1] = self.channel1plotvector[1:]
        if self._widget.channel1check.isChecked():
            self.channel1plotvector[-1] = data.channels[0]
        else:
            self.channel1plotvector[-1] = 0.0
        self.channel1curve.setData(self.channel1plotvector)


    def setQuad(self):
        self.id = rospy.get_param('/body_array',[1,2,3,4,5])[self._widget.IrisInputBox.currentIndex()]
        if self.sub != '':
            self.sub.unregister()
        self.sub = rospy.Subscriber('/body_data/id_' + str(self.id) , QuadPositionDerived, self.callback)
        if self.channelsub != '':
            self.channelsub.unregister()
        self.channelsub = rospy.Subscriber('/' + self._widget.IrisInputBox.currentText() + '/mavros/rc/override', OverrideRCIn, self.callback2)
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

    

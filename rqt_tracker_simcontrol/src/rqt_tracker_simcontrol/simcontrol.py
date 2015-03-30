
from qt_gui.plugin import Plugin

from .simcontrol_widget import SimcontrolWidget


class Simcontrol(Plugin):

    def __init__(self, context):
        super(Simcontrol, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Simcontrol')

        # Process standalone plugin command-line arguments
        #from argparse import ArgumentParser
        #parser = ArgumentParser()
        # Add argument(s) to the parser.
        #parser.add_argument("-q", "--quiet", action="store_true",
        #              dest="quiet",
        #              help="Put plugin in silent mode")
        #args, unknowns = parser.parse_known_args(context.argv())
        #if not args.quiet:
        #    print 'arguments: ', args
        #    print 'unknowns: ', unknowns

        self._widget = SimcontrolWidget()

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle()+(' (%d)'%context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

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

#!/usr/bin/env python

import sys

from gui.rqt_iris import IrisWidget
from rqt_gui.main import Main

plugin = 'rqt_iris'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
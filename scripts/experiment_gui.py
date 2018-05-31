#!/usr/bin/env python

import sys

from experiment_gui.experiment_gui_module import ExperimentGUI
from rqt_gui.main import Main

plugin = 'experiment_gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))

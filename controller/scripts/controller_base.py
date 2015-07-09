#!/usr/bin/env python

# Abstract base class for a controller, serving as an interface. This
# interface should be used by Blender and for any communication with the 
# controller.

from abc import ABCMeta, abstractmethod

class Controller():

    __metaclass__ = ABCMeta  

    # Provides ability to reset a controller, for example the integral part in
    # the PID
    def reset(self):
        pass

    # Returns the control output of controller, for example in the form
    # of an acceleration vector
    @abstractmethod
    def get_output(self, current_point, target_point):
        pass

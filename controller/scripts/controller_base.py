#!/usr/bin/env python

# Abstract base class for a controller, serving as an interface. This
# interface should be used by Blender and for any communication with the 
# controller.

from abc import ABCMeta, abstractmethod

class Controller():
    """This is an abstract base class for a controller."""
    
    __metaclass__ = ABCMeta  

    
    def reset(self):
        """This function provides the ability to reset a controller, for example 
        the integral part in the PID."""
        pass

    ##@param current_point: the point at which the quad is now
    ##@param target_point: the target of the quad
    @abstractmethod
    def get_output(self, current_point, target_point):
        """This function returns the control output of a controller."""
        pass

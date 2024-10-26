#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

class MathUtils:
    def normalize_angle_radians(self, angle):
        if angle < -3.14:
            angle += 3.14
        elif angle >= 3.14:
            angle -= 3.14
        return angle
    
    def radians_to_degrees(self, radians):
        return (radians * 180) / 3.14
    
    def degrees_to_radians(self, degrees):
        return (degrees * 3.14) / 180
    
    
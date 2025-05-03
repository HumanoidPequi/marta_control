 #!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

class MathUtils:
    def normalize_roll_angle_radians(self, angle):
        angle_degree = self.degrees_to_radians(angle)
        angle_degree_normalized = self.normalize_roll_angle_degree(angle_degree)
        return self.degrees_to_radians(angle_degree_normalized)
    
    def normalize_roll_angle_degree(self,angle_degrees):
        """
        Normaliza um ângulo para o intervalo [-90, 90] graus.

        Parâmetros:
        - angle_degrees (float): Ângulo em graus.

        Retorna:
        - float: Ângulo normalizado entre -90 e 90 graus.
        """
        # Normaliza para o intervalo [0, 360)
        angle_degrees = angle_degrees % 360
        
        # Converte para o intervalo [-180, 180)
        if angle_degrees > 180:
            angle_degrees -= 360
        
        # Finalmente, limita para [-90, 90]
        if angle_degrees > 90:
            angle_degrees = 180 - angle_degrees
        elif angle_degrees < -90:
            angle_degrees = -180 - angle_degrees

        return angle_degrees
    
    def normalize_pitch_angle_radians(self, angle):
        if angle < -3.14159:
            angle += 3.14159*2
        elif angle >= 3.14159:
            angle -= 3.14159*2
        return angle

    def radians_to_degrees(self, radians):
        return (radians * 180) / 3.14159
    
    def degrees_to_radians(self, degrees):
        return (degrees * 3.14159) / 180
    
    
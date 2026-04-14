import math
import time

from servo import Servo
from Vector2D import Vector2D_class

"""Hier worden de lengtes van de onder- en bovenarm gedefinieerd"""
Anorm = 80  # lengte bovenarm
Bnorm = 60  # lengte onderarm

class ServoArm:
    
    def __init__(self, shoulder_pin, elbow_pin, lift_pin):
        self.shoulder_servo = Servo(shoulder_pin)
        self.elbow_servo = Servo(elbow_pin)
        self.lift_servo = Servo(lift_pin)

    def __C2AB(self, C_vector):
        # Construeert de A en B vectoren op basis van vector C
        X = Vector2D_class(1, 0) # Hulpvector
        Cnorm = C_vector.norm()  # lengte van vector C
        beta = math.acos((Bnorm ** 2 - Anorm ** 2 - Cnorm ** 2) / (-2 * Anorm * Cnorm))  # cosinus regel
        delta = C_vector.vecs2angle(X)
        phi = beta + delta
        # Construct A_vector in Cartesian coordinates from polar coordinates
        A_vector = Vector2D_class(Anorm * math.cos(phi), Anorm * math.sin(phi))
        B_vector = C_vector.sub(A_vector)
        return A_vector, B_vector

    def __AB2phigamma(self, A_vector, B_vector):
        # berekent de hoeken phi en gamma
        X = Vector2D_class(1, 0)
        phi = A_vector.vecs2angle(X)
        gamma = A_vector.vecs2angle(B_vector)
        return phi, gamma

    def set_xy(self, x, y):
        C_vector = Vector2D_class(x, y)
        A_vector, B_vector = self.__C2AB(C_vector)
        phi, gamma = self.__AB2phigamma(A_vector, B_vector)
        self.shoulder_servo.set_angle_radians(phi)
        self.elbow_servo.set_angle_radians(gamma)
        time.sleep(0.1)

    def pen_up(self):
        self.lift_servo.set_angle_degrees(90)

    def pen_down(self):
        self.lift_servo.set_angle_degrees(90 + 60)

    def home(self):
        self.pen_up()
        self.set_xy(60, 80)

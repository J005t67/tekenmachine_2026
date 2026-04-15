

from ulab import numpy as np
import Vector2D
import math
import servo
import time

class brachi_class:

    def __init__(self):

        def arange_with_endpoint(start, stop, step):
            return np.arange(start, stop + step, step)

        """Hier worden de lengtes van de onder- en bovenarm gedefinieerd"""
        Anorm = 80  # lengte bovenarm
        Bnorm = 60  # lengte onderarm

        xp = arange_with_endpoint(0, 180, 10)
        fp_schouder = np.array([0, 10, 20, 26, 37, 48, 61, 72, 82, 90, 100, 108, 120, 129, 140, 151, 162, 172, 180])
        fp_elleboog = np.array([0, 9, 17, 31, 41, 49, 63, 71, 79, 90, 96, 105, 117, 127, 139, 150, 162, 170, 180])
        fp_pen = xp
        schouder_offset = 0  # graden
        elleboog_offset = 10  # graden
        pen_offset = 0  # graden
        freq = 50
        self.Anorm = Anorm
        self.Bnorm = Bnorm
        self.reach = Anorm + Bnorm
        self.servo_schouder = servo.servo_class("schouder", 18, freq, xp, fp_schouder, schouder_offset)
        self.servo_elleboog = servo.servo_class("elleboog", 19, freq, xp, fp_elleboog, elleboog_offset)
        self.servo_pen = servo.servo_class("pen", 20, freq, xp, fp_pen, pen_offset)

    def C2AB(self, C_vector):
        # Construeert de A en B vectoren op basis van vector C
        X = Vector2D.Vector2D_class(1, 0)  # Hulpvector
        Cnorm = C_vector.norm()  # lengte van vector C
        beta = math.acos((self.Bnorm ** 2 - self.Anorm ** 2 - Cnorm ** 2) / (-2 * self.Anorm * Cnorm))  # cosinus regel
        delta = C_vector.vecs2angle(X)
        phi = beta + delta
        # Construct A_vector in Cartesian coordinates from polar coordinates
        A_vector = Vector2D.Vector2D_class(self.Anorm * math.cos(phi), self.Anorm * math.sin(phi))
        B_vector = C_vector.sub(A_vector)
        return A_vector, B_vector

    def AB2phigamma(self, A_vector, B_vector):
        # berekent de hoeken phi en gamma
        X = Vector2D.Vector2D_class(1, 0)
        phi = A_vector.vecs2angle(X)
        gamma = A_vector.vecs2angle(B_vector)
        return phi, gamma

    def set_xy(self, x, y):
        C_vector = Vector2D.Vector2D_class(x, y)
        if C_vector.norm() <= self.reach:
            A_vector, B_vector = self.C2AB(C_vector)
            phi, gamma = self.AB2phigamma(A_vector, B_vector)
            self.servo_schouder.set_servo_angle(self.servo_schouder.angle_to_servo_angle(math.degrees(phi)))
            self.servo_elleboog.set_servo_angle(self.servo_elleboog.angle_to_servo_angle(math.degrees(gamma)))
            time.sleep(0.1)
        else:
            print("Point out of reach.")

    def pen_up(self):
        self.servo_pen.set_servo_angle(90)

    def pen_down(self):
        self.servo_pen.set_servo_angle(90 + 60)

    def home(self):
        self.pen_up()
        time.sleep(1)
        self.set_xy(60, 80)






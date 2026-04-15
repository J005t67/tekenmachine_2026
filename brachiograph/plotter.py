"""Contains a base class for a drawing robot (MicroPython port).

Changes from the CPython version (https://github.com/evildmp/BrachioGraph/blob/main/plotter.py):
- pigpio replaced with machine.PWM / machine.Pin
- numpy replaced with a pure-Python polyfit / poly1d implementation
- tqdm progress bars replaced with plain print() progress
- readchar replaced with sys.stdin.read(1)
- pprint replaced with json.dumps + print
- time.monotonic() replaced with ticks_ms() / ticks_diff()
- Turtle graphics support removed (unavailable on MicroPython)
"""

from time import sleep, ticks_ms, ticks_diff
import json
import math
import sys

try:
    from machine import PWM, Pin
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


# --------------- Pure-Python polynomial utilities (replaces numpy) ---------------

def _gauss_solve(A, b):
    """Solve the linear system Ax = b using Gaussian elimination with partial pivoting."""
    n = len(b)
    # Build augmented matrix [A | b]
    M = [A[i][:] + [b[i]] for i in range(n)]

    for col in range(n):
        # Partial pivot: swap in the row with the largest absolute value in this column
        max_row = max(range(col, n), key=lambda r: abs(M[r][col]))
        M[col], M[max_row] = M[max_row], M[col]

        pivot = M[col][col]
        if abs(pivot) < 1e-12:
            raise ValueError("Singular matrix in polyfit")

        for row in range(col + 1, n):
            factor = M[row][col] / pivot
            for j in range(col, n + 1):
                M[row][j] -= factor * M[col][j]

    # Back substitution
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = M[i][n]
        for j in range(i + 1, n):
            x[i] -= M[i][j] * x[j]
        x[i] /= M[i][i]
    return x


def polyfit(x_data, y_data, deg):
    """Fit a polynomial of given degree to (x_data, y_data) via least squares.

    Returns coefficients in descending order (highest degree first),
    matching the numpy.polyfit convention.
    """
    n = len(x_data)
    d = deg + 1
    # Build the Vandermonde matrix A (columns: x^0, x^1, ..., x^deg)
    A = [[float(x_data[i]) ** j for j in range(d)] for i in range(n)]
    # Normal equations: (A^T A) c = A^T y
    ATA = [[sum(A[k][i] * A[k][j] for k in range(n)) for j in range(d)] for i in range(d)]
    ATy = [sum(A[k][i] * float(y_data[k]) for k in range(n)) for i in range(d)]
    coeffs_asc = _gauss_solve(ATA, ATy)
    return list(reversed(coeffs_asc))  # highest degree first


def poly1d(coeffs):
    """Return a callable that evaluates the polynomial at x using Horner's method.

    coeffs must be in descending order (highest degree first),
    matching the numpy.poly1d convention.
    """
    def evaluate(x):
        result = 0.0
        for c in coeffs:
            result = result * x + c
        return result
    return evaluate


def _mean(values):
    return sum(values) / len(values)


# ---------------------------------------------------------------------------------


class Plotter:
    def __init__(
        self,
        virtual=False,           # a virtual plotter runs in software only
        #  ----------------- geometry of the plotter -----------------
        bounds=(-10, 5, 10, 15), # the maximum rectangular drawing area
        #  ----------------- naive calculation values -----------------
        servo_1_parked_pw=1500,  # pulse-widths when parked
        servo_2_parked_pw=1500,
        servo_1_degree_ms=-10,   # microseconds pulse-width per degree
        servo_2_degree_ms=10,
        servo_1_parked_angle=0,  # the arm angle in the parked position
        servo_2_parked_angle=0,
        #  ----------------- hysteresis -----------------
        hysteresis_correction_1=0,  # hardware error compensation
        hysteresis_correction_2=0,
        #  ----------------- servo angles and pulse-widths in lists -----------------
        servo_1_angle_pws=(),    # pulse-widths for various angles
        servo_2_angle_pws=(),
        #  ----------------- servo angles and pulse-widths in lists (bi-directional) -----
        servo_1_angle_pws_bidi=(),  # bi-directional pulse-widths for various angles
        servo_2_angle_pws_bidi=(),
        #  ----------------- the pen -----------------
        pw_up=None,              # pulse-widths for pen up/down
        pw_down=None,
        #  ----------------- physical control -----------------
        angular_step=None,       # default step of the servos in degrees
        wait=None,               # default wait time between operations
        resolution=None,         # default resolution of the plotter in cm
    ):
        self.last_moved = ticks_ms()
        self.virtual = virtual
        self.angle_1 = servo_1_parked_angle
        self.angle_2 = servo_2_parked_angle

        self.bounds = bounds

        # ------ Servo 1 ------
        self.servo_1_parked_pw = servo_1_parked_pw
        self.servo_1_degree_ms = servo_1_degree_ms
        self.servo_1_parked_angle = servo_1_parked_angle
        self.hysteresis_correction_1 = hysteresis_correction_1

        if servo_1_angle_pws_bidi:
            servo_1_angle_pws = []
            differences = []
            for angle, pws in servo_1_angle_pws_bidi.items():
                pw = (pws["acw"] + pws["cw"]) / 2
                servo_1_angle_pws.append([angle, pw])
                differences.append((pws["acw"] - pws["cw"]) / 2)
            self.hysteresis_correction_1 = _mean(differences)

        if servo_1_angle_pws:
            angles = [row[0] for row in servo_1_angle_pws]
            pws_vals = [row[1] for row in servo_1_angle_pws]
            self.angles_to_pw_1 = poly1d(polyfit(angles, pws_vals, 3))
        else:
            self.angles_to_pw_1 = self.naive_angles_to_pulse_widths_1

        # ------ Servo 2 ------
        self.servo_2_parked_pw = servo_2_parked_pw
        self.servo_2_degree_ms = servo_2_degree_ms
        self.servo_2_parked_angle = servo_2_parked_angle
        self.hysteresis_correction_2 = hysteresis_correction_2

        if servo_2_angle_pws_bidi:
            servo_2_angle_pws = []
            differences = []
            for angle, pws in servo_2_angle_pws_bidi.items():
                pw = (pws["acw"] + pws["cw"]) / 2
                servo_2_angle_pws.append([angle, pw])
                differences.append((pws["acw"] - pws["cw"]) / 2)
            self.hysteresis_correction_2 = _mean(differences)

        if servo_2_angle_pws:
            angles = [row[0] for row in servo_2_angle_pws]
            pws_vals = [row[1] for row in servo_2_angle_pws]
            self.angles_to_pw_2 = poly1d(polyfit(angles, pws_vals, 3))
        else:
            self.angles_to_pw_2 = self.naive_angles_to_pulse_widths_2

        # set some initial values required for moving methods
        self.previous_pw_1 = self.previous_pw_2 = 0
        self.active_hysteresis_correction_1 = self.active_hysteresis_correction_2 = 0
        self.reset_report()

        if self.virtual or not HARDWARE_AVAILABLE:
            self.wait = wait if wait is not None else 0
            self.virtualise()
        else:
            self._servo_1 = PWM(Pin(18))
            self._servo_1.freq(50)
            self._servo_2 = PWM(Pin(19))
            self._servo_2.freq(50)
            # Track current pulse widths (machine.PWM has no read-back)
            self._current_pw_1 = 0
            self._current_pw_2 = 0
            self.virtual = False
            self.wait = wait if wait is not None else 0.01

        pw_up = pw_up or 1400
        pw_down = pw_down or 1600

        self.pen = Pen(bg=self, pw_up=pw_up, pw_down=pw_down, virtual=self.virtual)

        self.angular_step = angular_step or 0.1
        self.resolution = resolution or 0.1

        self.set_angles(self.servo_1_parked_angle, self.servo_2_parked_angle)
        sleep(1)

        self.status()

    def virtualise(self):
        print("Initialising virtual BrachioGraph")
        self.virtual_pw_1 = self.angles_to_pw_1(-90)
        self.virtual_pw_2 = self.angles_to_pw_2(90)
        self.virtual = True

    #  ----------------- plotting methods -----------------

    def plot_file(self, filename="", bounds=None, angular_step=None, wait=None, resolution=None):
        """Plots an image encoded as JSON lines in ``filename``."""
        bounds = bounds or self.bounds
        with open(filename, "r") as line_file:
            lines = json.load(line_file)
        self.plot_lines(lines, bounds, angular_step, wait, resolution, flip=True)

    def plot_lines(
        self,
        lines=None,
        bounds=None,
        angular_step=None,
        wait=None,
        resolution=None,
        flip=False,
        rotate=False,
    ):
        """Passes each segment of each line in lines to ``draw_line()``"""
        if lines is None:
            lines = []
        bounds = bounds or self.bounds
        lines = self.rotate_and_scale_lines(lines=lines, bounds=bounds, flip=True)
        total = len(lines)
        for i, line in enumerate(lines):
            print("Lines: {}/{}".format(i + 1, total))
            x, y = line[0]
            if (round(self.x, 1), round(self.y, 1)) != (round(x, 1), round(y, 1)):
                self.xy(x, y, angular_step, wait, resolution)
            for point in line[1:]:
                x, y = point
                self.xy(x, y, angular_step, wait, resolution, draw=True)
        self.park()

    #  ----------------- pattern-drawing methods -----------------

    def box(self, bounds=None, angular_step=None, wait=None, resolution=None, repeat=1, reverse=False):
        """Draw a box marked out by the ``bounds``."""
        bounds = bounds or self.bounds
        if not bounds:
            return "Box drawing is only possible when the bounds attribute is set."

        self.xy(bounds[0], bounds[1], angular_step, wait, resolution)

        for r in range(repeat):
            if not reverse:
                self.xy(bounds[2], bounds[1], angular_step, wait, resolution, draw=True)
                self.xy(bounds[2], bounds[3], angular_step, wait, resolution, draw=True)
                self.xy(bounds[0], bounds[3], angular_step, wait, resolution, draw=True)
                self.xy(bounds[0], bounds[1], angular_step, wait, resolution, draw=True)
            else:
                self.xy(bounds[0], bounds[3], angular_step, wait, resolution, draw=True)
                self.xy(bounds[2], bounds[3], angular_step, wait, resolution, draw=True)
                self.xy(bounds[2], bounds[1], angular_step, wait, resolution, draw=True)
                self.xy(bounds[0], bounds[1], angular_step, wait, resolution, draw=True)

        self.park()

    def test_pattern(
        self,
        lines=4,
        bounds=None,
        angular_step=None,
        wait=None,
        resolution=None,
        repeat=1,
        reverse=False,
        both=False,
    ):
        self.vertical_lines(lines, bounds, angular_step, wait, resolution, repeat, reverse, both)
        self.horizontal_lines(lines, bounds, angular_step, wait, resolution, repeat, reverse, both)

    def vertical_lines(
        self,
        lines=4,
        bounds=None,
        angular_step=None,
        wait=None,
        resolution=None,
        repeat=1,
        reverse=False,
        both=False,
    ):
        bounds = bounds or self.bounds
        if not bounds:
            return "Plotting a test pattern is only possible when the bounds attribute is set."

        if not reverse:
            top_y = self.top
            bottom_y = self.bottom
        else:
            bottom_y = self.top
            top_y = self.bottom

        for n in range(repeat):
            step = (self.right - self.left) / lines
            x = self.left
            while x <= self.right:
                self.draw_line((x, top_y), (x, bottom_y), angular_step, wait, resolution, both)
                x = x + step

        self.park()

    def horizontal_lines(
        self,
        lines=4,
        bounds=None,
        angular_step=None,
        wait=None,
        resolution=None,
        repeat=1,
        reverse=False,
        both=False,
    ):
        bounds = bounds or self.bounds
        if not bounds:
            return "Plotting a test pattern is only possible when the bounds attribute is set."

        if not reverse:
            min_x = self.left
            max_x = self.right
        else:
            max_x = self.left
            min_x = self.right

        for n in range(repeat):
            step = (self.bottom - self.top) / lines
            y = self.top
            while y >= self.bottom:
                self.draw_line((min_x, y), (max_x, y), angular_step, wait, resolution, both)
                y = y + step

        self.park()

    #  ----------------- x/y drawing methods -----------------

    def draw_line(self, start=(0, 0), end=(0, 0), angular_step=None, wait=None, resolution=None, both=False):
        """Draws a line between two points"""
        start_x, start_y = start
        end_x, end_y = end
        self.xy(start_x, start_y, angular_step, wait, resolution)
        self.xy(end_x, end_y, angular_step, wait, resolution, draw=True)
        if both:
            self.xy(start_x, start_y, angular_step, wait, resolution, draw=True)

    def xy(self, x=None, y=None, angular_step=None, wait=None, resolution=None, draw=False):
        """Moves the pen to the xy position; optionally draws while doing it."""
        wait = wait if wait is not None else self.wait
        resolution = resolution or self.resolution

        x = x if x is not None else self.x
        y = y if y is not None else self.y
        (angle_1, angle_2) = self.xy_to_angles(x, y)

        if draw:
            (x_length, y_length) = (x - self.x, y - self.y)
            length = math.sqrt(x_length ** 2 + y_length ** 2)
            no_of_steps = round(length / resolution) or 1
            (length_of_step_x, length_of_step_y) = (x_length / no_of_steps, y_length / no_of_steps)

            for step in range(no_of_steps):
                self.x = self.x + length_of_step_x
                self.y = self.y + length_of_step_y
                angle_1, angle_2 = self.xy_to_angles(self.x, self.y)
                self.move_angles(angle_1, angle_2, angular_step, wait, draw)
        else:
            self.move_angles(angle_1, angle_2, angular_step, wait, draw)

    #  ----------------- servo angle drawing methods -----------------

    def move_angles(self, angle_1=None, angle_2=None, angular_step=None, wait=None, draw=False):
        """Moves the servo motors to the specified angles step-by-step."""
        wait = wait if wait is not None else self.wait
        angular_step = angular_step or self.angular_step

        if draw:
            self.pen.down()
        else:
            self.pen.up()

        diff_1 = diff_2 = 0
        if angle_1 is not None:
            diff_1 = angle_1 - self.angle_1
        if angle_2 is not None:
            diff_2 = angle_2 - self.angle_2

        no_of_steps = int(max(abs(diff_1 / angular_step), abs(diff_2 / angular_step))) or 1
        (length_of_step_1, length_of_step_2) = (diff_1 / no_of_steps, diff_2 / no_of_steps)

        for step in range(no_of_steps):
            self.angle_1 = self.angle_1 + length_of_step_1
            self.angle_2 = self.angle_2 + length_of_step_2

            time_since_last_moved = ticks_diff(ticks_ms(), self.last_moved) / 1000  # seconds
            if time_since_last_moved < wait:
                sleep(wait - time_since_last_moved)

            self.set_angles(self.angle_1, self.angle_2)
            self.last_moved = ticks_ms()

    # ----------------- pen-moving methods -----------------

    def set_angles(self, angle_1=None, angle_2=None):
        """Moves the servo motors to the specified angles immediately."""
        pw_1 = pw_2 = None

        if angle_1 is not None:
            pw_1 = self.angles_to_pw_1(angle_1)

            if pw_1 > self.previous_pw_1:
                self.active_hysteresis_correction_1 = self.hysteresis_correction_1
            elif pw_1 < self.previous_pw_1:
                self.active_hysteresis_correction_1 = -self.hysteresis_correction_1

            self.previous_pw_1 = pw_1
            pw_1 = pw_1 + self.active_hysteresis_correction_1

            self.angle_1 = angle_1
            self.angles_used_1.add(int(angle_1))
            self.pulse_widths_used_1.add(int(pw_1))

        if angle_2 is not None:
            pw_2 = self.angles_to_pw_2(angle_2)

            if pw_2 > self.previous_pw_2:
                self.active_hysteresis_correction_2 = self.hysteresis_correction_2
            elif pw_2 < self.previous_pw_2:
                self.active_hysteresis_correction_2 = -self.hysteresis_correction_2

            self.previous_pw_2 = pw_2
            pw_2 = pw_2 + self.active_hysteresis_correction_2

            self.angle_2 = angle_2
            self.angles_used_2.add(int(angle_2))
            self.pulse_widths_used_2.add(int(pw_2))

        self.x, self.y = self.angles_to_xy(self.angle_1, self.angle_2)
        self.set_pulse_widths(pw_1, pw_2)

    def park(self):
        """Park the plotter."""
        if self.virtual:
            print("Parking")
        self.pen.up()
        self.move_angles(self.servo_1_parked_angle, self.servo_2_parked_angle)

    #  ----------------- angles-to-pulse-widths methods -----------------

    def naive_angles_to_pulse_widths_1(self, angle):
        """A rule-of-thumb calculation of pulse-width for the desired servo angle"""
        return (angle - self.servo_1_parked_angle) * self.servo_1_degree_ms + self.servo_1_parked_pw

    def naive_angles_to_pulse_widths_2(self, angle):
        """A rule-of-thumb calculation of pulse-width for the desired servo angle"""
        return (angle - self.servo_2_parked_angle) * self.servo_2_degree_ms + self.servo_2_parked_pw

    # ----------------- line-processing methods -----------------

    def rotate_and_scale_lines(self, lines=None, rotate=False, flip=False, bounds=None):
        """Rotates and scales the lines so that they best fit the available drawing ``bounds``."""
        if lines is None:
            lines = []
        (
            rotate,
            x_mid_point,
            y_mid_point,
            box_x_mid_point,
            box_y_mid_point,
            divider,
        ) = self.analyse_lines(lines, rotate, bounds)

        for line in lines:
            for point in line:
                if rotate:
                    point[0], point[1] = point[1], point[0]

                x = point[0]
                x = x - x_mid_point
                x = x / divider

                if flip ^ rotate:
                    x = -x

                x = x + box_x_mid_point

                y = point[1]
                y = y - y_mid_point
                y = y / divider
                y = y + box_y_mid_point

                point[0], point[1] = x, y

        return lines

    def analyse_lines(self, lines=None, rotate=False, bounds=None):
        """Analyses the co-ordinates in ``lines`` and returns scaling/rotation parameters."""
        if lines is None:
            lines = []
        bounds = bounds or self.bounds

        x_values_in_lines = set()
        y_values_in_lines = set()

        for line in lines:
            x_values_in_line, y_values_in_line = zip(*line)
            x_values_in_lines.update(x_values_in_line)
            y_values_in_lines.update(y_values_in_line)

        min_x, max_x = min(x_values_in_lines), max(x_values_in_lines)
        min_y, max_y = min(y_values_in_lines), max(y_values_in_lines)

        x_range, y_range = max_x - min_x, max_y - min_y
        box_x_range, box_y_range = bounds[2] - bounds[0], bounds[3] - bounds[1]

        x_mid_point, y_mid_point = (max_x + min_x) / 2, (max_y + min_y) / 2
        box_x_mid_point, box_y_mid_point = (bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2

        if (x_range >= y_range and box_x_range >= box_y_range) or (
            x_range <= y_range and box_x_range <= box_y_range
        ):
            divider = max((x_range / box_x_range), (y_range / box_y_range))
            rotate = False
        else:
            divider = max((x_range / box_y_range), (y_range / box_x_range))
            rotate = True
            x_mid_point, y_mid_point = y_mid_point, x_mid_point

        return (rotate, x_mid_point, y_mid_point, box_x_mid_point, box_y_mid_point, divider)

    # ----------------- physical control methods -----------------

    def set_pulse_widths(self, pw_1=None, pw_2=None):
        """Applies the supplied pulse-width values to the servos.

        Pulse widths are in microseconds. Converts to duty_u16 for machine.PWM:
            duty_u16 = pw_us / 20000 * 65535   (50 Hz → 20 ms period)
        """
        if self.virtual:
            if pw_1:
                if 500 < pw_1 < 2500:
                    self.virtual_pw_1 = int(pw_1)
                else:
                    raise ValueError
            if pw_2:
                if 500 < pw_2 < 2500:
                    self.virtual_pw_2 = int(pw_2)
                else:
                    raise ValueError
        else:
            if pw_1:
                self._servo_1.duty_u16(int(pw_1 * 65535 // 20000))
                self._current_pw_1 = int(pw_1)
            if pw_2:
                self._servo_2.duty_u16(int(pw_2 * 65535 // 20000))
                self._current_pw_2 = int(pw_2)

    def get_pulse_widths(self):
        """Returns the current pulse-width values in microseconds."""
        if self.virtual:
            return (self.virtual_pw_1, self.virtual_pw_2)
        else:
            return (self._current_pw_1, self._current_pw_2)

    def quiet(self, servos=None):
        """Stop sending pulses to the servos so they are no longer energised."""
        if servos is None:
            servos = [18, 19, 20]

        if self.virtual:
            print("Going quiet")
        else:
            for servo_pin in servos:
                if servo_pin == 18:
                    self._servo_1.duty_u16(0)
                elif servo_pin == 19:
                    self._servo_2.duty_u16(0)
                elif servo_pin == 20:
                    self.pen._pwm.duty_u16(0)

    # ----------------- manual driving methods -----------------

    def capture_pws(self):
        """Helps capture angle/pulse-width data for the servos.

        Reads single characters from sys.stdin (the serial REPL).
        """
        print(
            """
Drive each servo over a wide range of movement (do not exceed a pulse-width
range ~600 to ~2400). To capture the pulse-width value for a particular angle,
press "c", then enter the angle. For each angle, do this in both directions,
clockwise and anti-clockwise. Press "0" to exit.
        """
        )

        pw_1, pw_2 = self.get_pulse_widths()
        pen_pw = self.pen.get_pw()

        last_action = None
        pws1_dict = {}
        pws2_dict = {}
        pen_pw_dict = {}

        print("0 to exit, c to capture a value, v to show captured values")
        print("Shoulder a: -10  A: -1   s: +10  S: +1")
        print("Elbow    k: -10  K: -1   l: +10  L: +1")
        print("Pen      z: -10          x: +10")

        controls = {
            "a": [-10, 0, 0, "acw"],
            "A": [-1, 0, 0, "acw"],
            "s": [+10, 0, 0, "cw"],
            "S": [+1, 0, 0, "cw"],
            "k": [0, -10, 0, "acw"],
            "K": [0, -1, 0, "acw"],
            "l": [0, +10, 0, "cw"],
            "L": [0, +1, 0, "cw"],
            "z": [0, 0, -10],
            "x": [0, 0, +10],
        }

        while True:
            key = sys.stdin.read(1)
            values = controls.get(key)

            if values:
                if values[0] or values[1] or values[2]:
                    pw_1 += values[0]
                    pw_2 += values[1]
                    pen_pw += values[2]
                    print("shoulder: {}, elbow: {}, pen: {}".format(pw_1, pw_2, pen_pw))
                    self.set_pulse_widths(pw_1, pw_2)
                    self.pen.pw(pen_pw)
                    last_action = values

            elif key == "0" or key == "v":
                print("servo_1_angle_pws_bidi =")
                print(json.dumps(pws1_dict))
                print("servo_2_angle_pws_bidi =")
                print(json.dumps(pws2_dict))
                print("Pen pulse-widths =")
                print(json.dumps(pen_pw_dict))
                if key == "0":
                    return

            elif key == "c":
                if not last_action:
                    print("Drive the servos to a new position first")
                elif last_action[0]:
                    angle = int(input("Enter the angle of the inner arm: "))
                    pws1_dict.setdefault(angle, {})[last_action[3]] = pw_1
                    print(pws1_dict)
                elif last_action[1]:
                    angle = int(input("Enter the angle of the outer arm: "))
                    pws2_dict.setdefault(angle, {})[last_action[3]] = pw_2
                    print(pws2_dict)
                elif last_action[2]:
                    state = input("Enter the state of the pen ([u]p, [d]own): ")
                    pen_pw_dict[state] = pen_pw
                    print(pen_pw)

    def drive_xy(self):
        """Control the x/y position using the keyboard via the serial REPL."""
        while True:
            key = sys.stdin.read(1)
            if key == "0":
                return
            elif key == "a":
                self.x = self.x - 1
            elif key == "s":
                self.x = self.x + 1
            elif key == "A":
                self.x = self.x - 0.1
            elif key == "S":
                self.x = self.x + 0.1
            elif key == "k":
                self.y = self.y - 1
            elif key == "l":
                self.y = self.y + 1
            elif key == "K":
                self.y = self.y - 0.1
            elif key == "L":
                self.y = self.y + 0.1

            print(self.x, self.y)
            self.xy(self.x, self.y)

    # ----------------- reporting methods -----------------

    def status(self):
        """Provides a report of the plotter status."""
        print("------------------------------------------")
        print("                      | Servo 1 | Servo 2 ")
        print("----------------------|---------|---------")

        pw_1, pw_2 = self.get_pulse_widths()
        print("         pulse-width | {:>7.0f} | {:>7.0f}".format(pw_1, pw_2))

        angle_1, angle_2 = self.angle_1, self.angle_2
        print("               angle | {:>7.0f} | {:>7.0f}".format(angle_1, angle_2))

        h1, h2 = self.hysteresis_correction_1, self.hysteresis_correction_2
        print("hysteresis corr.     | {:>7.1f} | {:>7.1f}".format(h1, h2))
        print("------------------------------------------")
        print("        x/y location | {:>7.1f} | {:>7.1f}".format(self.x, self.y))
        print()
        print("------------------------------------------")
        print("pen:", self.pen.position)
        print("------------------------------------------")
        print("left: {}, right: {}, top: {}, bottom: {}".format(
            self.left, self.right, self.top, self.bottom))
        print("------------------------------------------")
        print("wait: {} seconds".format(self.wait))
        print("------------------------------------------")
        print("resolution: {} cm".format(self.resolution))
        print("------------------------------------------")
        print("angular step: {}°".format(self.angular_step))
        print("------------------------------------------")

    @property
    def left(self):
        return self.bounds[0]

    @property
    def bottom(self):
        return self.bounds[1]

    @property
    def right(self):
        return self.bounds[2]

    @property
    def top(self):
        return self.bounds[3]

    def reset_report(self):
        self.angle_1 = self.angle_2 = None
        self.angles_used_1 = set()
        self.angles_used_2 = set()
        self.pulse_widths_used_1 = set()
        self.pulse_widths_used_2 = set()

    # ----------------- trigonometric methods -----------------

    def xy_to_angles(self, x=0, y=0):
        """Dummy method; override in sub-classes."""
        return (0, 0)

    def angles_to_xy(self, angle_1, angle_2):
        """Dummy method; override in sub-classes."""
        return (0, 0)


class Pen:
    def __init__(self, bg, pw_up=1700, pw_down=1300, pin=20, transition_time=0.25, virtual=False):
        self.bg = bg
        self.pin = pin
        self.pw_up = pw_up
        self.pw_down = pw_down
        self.transition_time = transition_time
        self.position = "down"
        self.virtual = virtual

        if self.virtual:
            print("Initialising virtual Pen")
            self.virtual_pw = pw_up  # will be corrected by self.up() below
        else:
            self._pwm = PWM(Pin(pin))
            self._pwm.freq(50)
            self._current_pw = 0

        self.up()

    def down(self):
        if self.position == "up":
            if self.virtual:
                self.virtual_pw = self.pw_down
            else:
                self.ease_pen(self.pw_up, self.pw_down)
            self.position = "down"

    def up(self):
        if self.position == "down":
            if self.virtual:
                self.virtual_pw = self.pw_up
            else:
                self.ease_pen(self.pw_down, self.pw_up)
            self.position = "up"

    def ease_pen(self, start, end):
        """Moves the pen gently instead of all at once."""
        diff = end - start
        angle = float(start)
        length_of_step = diff / abs(diff)
        for i in range(abs(diff)):
            angle += length_of_step
            self._pwm.duty_u16(int(angle * 65535 // 20000))
            self._current_pw = int(angle)
            sleep(0.001)

    def pw(self, pulse_width):
        """Set pen motor pulse-width directly."""
        if self.virtual:
            self.virtual_pw = pulse_width
        else:
            self._pwm.duty_u16(int(pulse_width * 65535 // 20000))
            self._current_pw = int(pulse_width)

    def get_pw(self):
        """Get current pen motor pulse-width."""
        if self.virtual:
            return self.virtual_pw
        else:
            return self._current_pw

from dataclasses import dataclass
from typing import List, Union, Dict
import numpy as np
import matplotlib
import math

@dataclass
class Line:
    start: tuple  # (x, y)
    end: tuple    # (x, y)
    overlap_start: bool
    overlap_end: bool
    def get_length(self):
        return math.sqrt((self.start[0] - self.end[0])**2 + (self.start[1] - self.end[1])**2)


@dataclass
class PartialCircle:
    center: tuple       # (x, y)
    radius: float
    start_angle: float  # radians, 0 is up along y-axis
    end_angle: float    # radians, rotates clockwise
    overlap_start: bool
    overlap_end: bool
    def get_length(self) -> float:
        angle_diff = abs(self.end_angle - self.start_angle)
        if self.end_angle < self.start_angle:
            angle_diff = math.pi * 2 - angle_diff
        return self.radius * angle_diff

Shape = Union[Line, PartialCircle]
BoxChar = List[Shape]
BoxFont = Dict[str, BoxChar]

def create_letter_A() -> BoxChar:
    return [
        Line((0, 0), (0.5, 2), False, False),
        Line((1, 0), (0.5, 2), False, True),
        Line((0.25, 1), (0.75, 1), True, True),
    ]

def create_letter_B() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        Line((0, 0), (0.5, 0), True, False),
        Line((0, 1), (0.5, 1), True, False),
        Line((0, 2), (0.5, 2), True, False),
        PartialCircle((0.5, 1.5), 0.5, 0, math.pi, True, True),
        PartialCircle((0.5, 0.5), 0.5, 0, math.pi, True, True),
    ]

def create_letter_C() -> BoxChar:
    return [
        PartialCircle((0.5, 1), 0.9, math.radians(135), math.radians(45), False, False)
    ]

def create_letter_D() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        PartialCircle((0, 1), 1.0, 0, math.pi, True, True),
    ]

def create_letter_E() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        Line((0, 2), (1, 2), True, False),
        Line((0, 1), (0.8, 1), True, False),
        Line((0, 0), (1, 0), True, False),
    ]

def create_letter_F() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        Line((0, 2), (1, 2), True, False),
        Line((0, 1), (0.8, 1), True, False),
    ]

def create_letter_G() -> BoxChar:
    return [
        PartialCircle((0.5, 1), 0.9, math.radians(90), math.radians(45), False, False),
        Line((0.5, 1), (1.4, 1), False, True),
    ]

def create_letter_H() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        Line((1, 0), (1, 2), False, False),
        Line((0, 1), (1, 1), True, True),
    ]

def create_letter_I() -> BoxChar:
    return [
        Line((0.5, 0), (0.5, 2), False, False),
    ]

def create_letter_J() -> BoxChar:
    return [
        Line((1, 2), (1, 0.5), False, False),
        PartialCircle((0.5, 0.5), 0.5, math.radians(90), math.radians(270), False, True),
    ]

def create_letter_K() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        Line((0, 1), (1, 2), True, False),
        Line((0, 1), (1, 0), True, False),
    ]

def create_letter_L() -> BoxChar:
    return [
        Line((0, 2), (0, 0), False, False),
        Line((0, 0), (1, 0), True, False),
    ]

def create_letter_M() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        Line((0, 2), (0.5, 1), True, False),
        Line((0.5, 1), (1, 2), True, True),
        Line((1, 2), (1, 0), False, False),
    ]

def create_letter_N() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        Line((0, 2), (1, 0), True, True),
        Line((1, 0), (1, 2), False, False),
    ]

def create_letter_O() -> BoxChar:
    return [
        PartialCircle((0.5, 1), 0.9, 0, math.pi * 2, False, False),
    ]

def create_letter_P() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        Line((0, 1), (0.5, 1), True, True),
        Line((0, 2), (0.5, 2), True, True),
        PartialCircle((0.5, 1.5), 0.5, 0, math.pi, False, False),
    ]

def create_letter_Q() -> BoxChar:
    return [
        PartialCircle((0.5, 1), 0.9, 0, math.pi * 2, False, False),
        Line((0.5, 1), (1, 0), True, False),
    ]

def create_letter_R() -> BoxChar:
    return [
        Line((0, 0), (0, 2), False, False),
        Line((0, 1), (0.5, 1), True, True),
        Line((0, 2), (0.5, 2), True, True),
        PartialCircle((0.5, 1.5), 0.5, 0, math.pi, False, False),
        Line((0.5, 1), (1, 0), True, False),
    ]

def create_letter_S() -> BoxChar:
    return [
        PartialCircle((0.5, 1.5), 0.5, math.pi, math.radians(50), True, False),
        PartialCircle((0.5, 0.5), 0.5, 0, math.radians(230), False, False),
    ]

def create_letter_T() -> BoxChar:
    return [
        Line((0.5, 0), (0.5, 2), False, False),
        Line((0, 2), (1, 2), False, False),
    ]

def create_letter_U() -> BoxChar:
    return [
        Line((0, 2), (0, 0.5), False, False),
        Line((1, 2), (1, 0.5), False, False),
        PartialCircle((0.5, 0.5), 0.5, math.radians(90), math.radians(270), True, True),
    ]

def create_letter_V() -> BoxChar:
    return [
        Line((0, 2), (0.5, 0), False, False),
        Line((1, 2), (0.5, 0), False, True),
    ]

def create_letter_W() -> BoxChar:
    return [
        Line((0, 2), (0.25, 0), False, False),
        Line((0.25, 0), (0.5, 1), True, False),
        Line((0.5, 1), (0.75, 0), True, True),
        Line((0.75, 0), (1, 2), False, False),
    ]

def create_letter_X() -> BoxChar:
    return [
        Line((0, 0), (0.5, 1), False, True),
        Line((0, 2), (0.5, 1), False, True),
        Line((1, 0), (0.5, 1), False, True),
        Line((1, 2), (0.5, 1), False, False),
    ]

def create_letter_Y() -> BoxChar:
    return [
        Line((0, 2), (0.5, 1), False, True),
        Line((1, 2), (0.5, 1), False, True),
        Line((0.5, 1), (0.5, 0), False, False),
    ]

def create_letter_Z() -> BoxChar:
    return [
        Line((0, 2), (1, 2), False, False),
        Line((1, 2), (0, 0), True, True),
        Line((0, 0), (1, 0), False, False),
    ]

def create_font() -> BoxFont:
    return {
        "A": create_letter_A(),
        "B": create_letter_B(),
        "C": create_letter_C(),
        "D": create_letter_D(),
        "E": create_letter_E(),
        "F": create_letter_F(),
        "G": create_letter_G(),
        "H": create_letter_H(),
        "I": create_letter_I(),
        "J": create_letter_J(),
        "K": create_letter_K(),
        "L": create_letter_L(),
        "M": create_letter_M(),
        "N": create_letter_N(),
        "O": create_letter_O(),
        "P": create_letter_P(),
        "Q": create_letter_Q(),
        "R": create_letter_R(),
        "S": create_letter_S(),
        "T": create_letter_T(),
        "U": create_letter_U(),
        "V": create_letter_V(),
        "W": create_letter_W(),
        "X": create_letter_X(),
        "Y": create_letter_Y(),
        "Z": create_letter_Z(),
    }

box_font = create_font()

def interpolate_line(line: Line, num_points: int) -> List[tuple]:
    """Interpolate evenly spaced points along a line."""
    x0, y0 = line.start
    x1, y1 = line.end

    # Adjust number of points if overlap at ends
    n = num_points
    points = [(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t) for t in np.linspace(0, 1, n)]
    if line.overlap_start:
        points = points[1:]
    if line.overlap_end:
        points = points[:-1]
    return points

def interpolate_partial_circle(arc: PartialCircle, num_points: int) -> List[tuple]:
    """Interpolate evenly spaced points along a clockwise arc."""
    cx, cy = arc.center
    start_angle = arc.start_angle % (2 * math.pi)
    end_angle = arc.end_angle % (2 * math.pi)

    # special case for full circle
    if arc.end_angle == math.pi * 2 and arc.start_angle == 0:
        angle = math.pi * 2
    else:
    # Compute total angle traversed clockwise
        angle = (end_angle - start_angle) % (2 * math.pi)


    
    # Adjust number of points if overlap at ends
    n = num_points
    angles = [end_angle - angle * t for t in np.linspace(0, 1, n)]
    
    points = [(cx + arc.radius * math.sin(a), cy + arc.radius * math.cos(a)) for a in angles]

    if arc.overlap_start:
        points = points[1:]
    if arc.overlap_end:
        points = points[:-1]
    return points

def get_character_points(char: str, points_per_char: int = 30) -> List[tuple]:
    """Get evenly distributed points for a character, excluding overlap points."""
    shapes = box_font.get(char.upper(), [])
    all_lenghts = []
    for shape in shapes:
        if isinstance(shape, (Line, PartialCircle)):
            all_lenghts.append(shape.get_length())
    total_length = sum(all_lenghts)
    min_gap = 0.125
    point_count = min(total_length / min_gap, points_per_char)
    all_points = []
    for shape, lenght in zip(shapes, all_lenghts):
        points_for_shape = math.ceil(point_count * (lenght / total_length))
        if isinstance(shape, Line):
            all_points.extend(interpolate_line(shape, points_for_shape))
        elif isinstance(shape, PartialCircle):
            all_points.extend(interpolate_partial_circle(shape, points_for_shape))
    return all_points


def main():
    # Main function for testing
    import matplotlib.pyplot as plt
    import numpy as np

    def plot_character(char, points_per_char=30):
        points = get_character_points(char, points_per_char)
        x_vals, y_vals = zip(*points)
        plt.figure(figsize=(2, 4))
        plt.scatter(x_vals, y_vals, s=10)
        plt.title(f"Character: {char}")
        plt.gca().set_aspect('equal')
        plt.grid(True)
        plt.show()
    for c in box_font.keys():
        plot_character(c)

if __name__ == "__main__":
    main()
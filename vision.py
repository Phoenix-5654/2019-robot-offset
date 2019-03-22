#!/usr/bin/env python3
import itertools
from heapq import nsmallest
from math import sqrt, tan, radians

import cv2
import numpy as np
from cscore import CameraServer, UsbCamera, VideoSource
from networktables import NetworkTables

HORIZONTAL_RES = 240
VERTICAL_RES = 320
FPS = 90
ERROR_VALUE = 0

SET_POINT = (VERTICAL_RES // 2 - 8, HORIZONTAL_RES // 2 + 43)


class GripPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        # self.__hsv_threshold_hue = [0.0, 180.0]
        # self.__hsv_threshold_saturation = [147, 255.0]
        # self.__hsv_threshold_value = [103, 255.0]

        self.__hsv_threshold_hue = [0.0, 180.0]
        self.__hsv_threshold_saturation = [0, 255.0]
        self.__hsv_threshold_value = [110, 255.0]

        self.hsv_threshold_output = None

        self.__find_contours_input = self.hsv_threshold_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__filter_contours_contours = self.find_contours_output
        self.__filter_contours_min_area = 150.0
        self.__filter_contours_min_perimeter = 0.0
        self.__filter_contours_min_width = 0.0
        self.__filter_contours_max_width = 1000.0
        self.__filter_contours_min_height = 0.0
        self.__filter_contours_max_height = 1000.0
        self.__filter_contours_solidity = [0, 100]
        self.__filter_contours_max_vertices = 1000000.0
        self.__filter_contours_min_vertices = 0.0
        self.__filter_contours_min_ratio = 0.0
        self.__filter_contours_max_ratio = 1000.0

        self.filter_contours_output = None

        self.__convex_hulls_contours = self.filter_contours_output

        self.convex_hulls_output = None

    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step HSV_Threshold0:
        self.__hsv_threshold_input = source0
        (self.hsv_threshold_output) = self.__hsv_threshold(self.__hsv_threshold_input,
                                                           self.__hsv_threshold_hue, self.__hsv_threshold_saturation,
                                                           self.__hsv_threshold_value)

        # Step Find_Contours0:
        self.__find_contours_input = self.hsv_threshold_output
        (self.find_contours_output) = self.__find_contours(
            self.__find_contours_input, self.__find_contours_external_only)

        # Step Filter_Contours0:
        self.__filter_contours_contours = self.find_contours_output
        (self.filter_contours_output) = self.__filter_contours(self.__filter_contours_contours,
                                                               self.__filter_contours_min_area,
                                                               self.__filter_contours_min_perimeter,
                                                               self.__filter_contours_min_width,
                                                               self.__filter_contours_max_width,
                                                               self.__filter_contours_min_height,
                                                               self.__filter_contours_max_height,
                                                               self.__filter_contours_solidity,
                                                               self.__filter_contours_max_vertices,
                                                               self.__filter_contours_min_vertices,
                                                               self.__filter_contours_min_ratio,
                                                               self.__filter_contours_max_ratio)

        # Step Convex_Hulls0:
        self.__convex_hulls_contours = self.filter_contours_output
        (self.convex_hulls_output) = self.__convex_hulls(
            self.__convex_hulls_contours)

    @staticmethod
    def __hsv_threshold(input, hue, sat, val):
        """Segment an image based on hue, saturation, and value ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max value.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
        return cv2.inRange(out, (hue[0], sat[0], val[0]), (hue[1], sat[1], val[1]))

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if (external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy = cv2.findContours(
            input, mode=mode, method=method)
        return contours

    @staticmethod
    def __filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,
                          min_height, max_height, solidity, max_vertex_count, min_vertex_count,
                          min_ratio, max_ratio):
        """Filters out contours that do not meet certain criteria.
        Args:
            input_contours: Contours as a list of numpy.ndarray.
            min_area: The minimum area of a contour that will be kept.
            min_perimeter: The minimum perimeter of a contour that will be kept.
            min_width: Minimum width of a contour.
            max_width: MaxWidth maximum width.
            min_height: Minimum height.
            max_height: Maximimum height.
            solidity: The minimum and maximum solidity of a contour.
            min_vertex_count: Minimum vertex Count of the contours.
            max_vertex_count: Maximum vertex Count.
            min_ratio: Minimum ratio of width to height.
            max_ratio: Maximum ratio of width to height.
        Returns:
            Contours as a list of numpy.ndarray.
        """
        output = []
        for contour in input_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height):
                continue
            area = cv2.contourArea(contour)
            if (area < min_area):
                continue
            if (cv2.arcLength(contour, True) < min_perimeter):
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
                continue
            ratio = (float)(w) / h
            if (ratio < min_ratio or ratio > max_ratio):
                continue
            output.append(contour)
        return output

    @staticmethod
    def __convex_hulls(input_contours):
        """Computes the convex hulls of contours.
        Args:
            input_contours: A list of numpy.ndarray that each represent a contour.
        Returns:
            A list of numpy.ndarray that each represent a contour.
        """
        output = []
        for contour in input_contours:
            output.append(cv2.convexHull(contour))
        return output


def start_camera():
    inst = CameraServer.getInstance()
    camera = UsbCamera('Hatch Panels', '/dev/video0')

    # with open("cam.json", encoding='utf-8') as cam_config:
    #     camera.setConfigJson(json.dumps(cam_config.read()))
    camera.setResolution(VERTICAL_RES, HORIZONTAL_RES)
    camera.setFPS(FPS)
    camera.setBrightness(10)
    camera.setConfigJson("""
{
    "fps": """ + str(FPS) + """,
    "height": """ + str(HORIZONTAL_RES) + """,
    "pixel format": "mjpeg",
    "properties": [
        {
            "name": "brightness",
            "value": 0
        },
        {
            "name": "contrast",
            "value": 100
        },
        {
            "name": "saturation",
            "value": 40
        }
    ],
    "width": """ + str(VERTICAL_RES) + """
}
    """)

    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    return inst, camera


class Shape:
    def __init__(self, points, center, angle, width, height):
        self.points = points
        self.center = center
        self.angle = abs(angle)
        self.width = width
        self.height = height

    @property
    def lowest_point(self):
        return self.points[0]

    @property
    def second_highest_point(self):
        return nsmallest(2, self.points, key=lambda x: x[1])[-1]

    @property
    def highest_point(self):
        return min(self.points, key=lambda x: x[1])

    @property
    def approx_area(self):
        return self.width * self.height

    def get_middle_point(self, shape):
        return get_average_point(self.center, shape.center)


def find_alignment_center(shapes, k):
    min_val = 100000000
    point = None
    targets = None
    combinations = itertools.combinations(shapes, 2)
    for combination in combinations:
        first = combination[0]
        second = combination[1]
        if (distance(first.lowest_point, second.lowest_point)
                > distance(first.second_highest_point, second.second_highest_point)):
            val = ((distance(first.lowest_point, second.lowest_point) +
                    distance(first.second_highest_point, second.second_highest_point))
                   / (k * (first.approx_area + second.approx_area)))

            if (val < min_val):
                min_val = val
                point = first.get_middle_point(second)
                targets = (first, second)
    return point, targets


def distance(point1: tuple, point2: tuple):
    return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))


def get_average_point(point1, point2):
    return (((point1[0] + point2[0]) / 2,
             (point1[1] + point2[1]) / 2))


def main():
    NetworkTables.initialize(server='10.56.54.2')
    NetworkTables.setUpdateRate(0.015)

    sd = NetworkTables.getTable('Vision')
    sd.putNumber('k', 1e-09)

    inst, camera = start_camera()

    pipeline = GripPipeline()

    cvSink = inst.getVideo(camera=camera)

    outputStream = inst.putVideo(
        'Hatch Panels Convex Hulls', HORIZONTAL_RES, VERTICAL_RES)

    img = np.zeros(shape=(VERTICAL_RES, HORIZONTAL_RES, 3), dtype=np.uint8)

    while True:
        k = sd.getNumber("k", 10)

        shapes = []

        targets = None

        time, img = cvSink.grabFrame(img)

        if time == 0:
            outputStream.notifyError(cvSink.getError())

            continue

        pipeline.process(img)

        drawing = np.zeros((HORIZONTAL_RES, VERTICAL_RES, 3), np.uint8)

        if len(pipeline.convex_hulls_output) != 0:

            for hull in pipeline.convex_hulls_output:
                rect = cv2.minAreaRect(hull)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                x, y, w, h = cv2.boundingRect(hull)

                shapes.append(
                    (Shape(box, rect[0], rect[2], w, h)))

                cv2.drawContours(drawing, [box], 0, (0, 0, 255), 4)
                cv2.circle(drawing, tuple(map(int, shapes[-1].second_highest_point)), 5, (100, 60, 200), 5)
                cv2.circle(drawing, tuple(map(int, shapes[-1].lowest_point)), 5, (200, 30, 100), 5)

            for i in range(len(pipeline.convex_hulls_output)):
                cv2.drawContours(
                    drawing, pipeline.convex_hulls_output, i, (255, 0, 0), 3)

            if len(shapes) == 1:
                # alignment_center = None
                if shapes[0].angle > 45:
                    alignment_center = (VERTICAL_RES, shapes[0].center[1])
                else:
                    alignment_center = (0, shapes[0].center[1])
            else:
                alignment_center, targets = find_alignment_center(shapes, k)

            if alignment_center is not None:

                cv2.circle(drawing, tuple(
                    map(int, alignment_center)), 5, (0, 255, 0), 5)

                sd.putNumber('X Error', SET_POINT[0] - alignment_center[0])
                sd.putNumber('Y Error', SET_POINT[1] - alignment_center[1])
                target_y_angle = abs((160 - alignment_center[1]) * 24.4) / 120

                sd.putNumber(
                    'X Angle', (abs(160 - alignment_center[0]) * 31.1) / 160)
                sd.putNumber('Y Angle', target_y_angle)

                sd.putNumber(
                    'Distance', 11.5 / tan(radians(target_y_angle + 15)))
            else:
                sd.putNumber("X Error", ERROR_VALUE)

            if targets is not None:

                if targets[0].lowest_point[0] < targets[1].lowest_point[0]:
                    sd.putNumber(
                        'Target Difference', targets[0].lowest_point[1] - targets[1].lowest_point[1])
                else:
                    sd.putNumber(
                        'Target Difference', targets[1].lowest_point[1] - targets[0].lowest_point[1])
                sd.putNumber("Dist Up",
                             abs(targets[0].highest_point[0]
                                 - targets[1].highest_point[0]))
                sd.putNumber("Dist Second highest",
                             abs(targets[0].second_highest_point[0]
                                 - targets[1].second_highest_point[0]))
                sd.putNumber("Dist Down",
                             abs(targets[0].lowest_point[0]
                                 - targets[1].lowest_point[0]))
                sum1 = (abs(targets[0].second_highest_point[0]
                            - targets[1].second_highest_point[0])
                        + abs(targets[0].lowest_point[0]
                              - targets[1].lowest_point[0]))
                sd.putNumber("SUM 1", sum1)
                sum2 = (abs(targets[0].highest_point[1] - targets[0].lowest_point[1])
                        + abs(targets[1].highest_point[1] - targets[1].highest_point[1]))
                sd.putNumber("SUM 2", sum2)
                sd.putNumber("FRACTION 1", sum1 / sum2)
                sd.putNumber("FRACTION 2", sum2 / sum1)
                sd.putNumber("FRACTION 3", abs(targets[0].second_highest_point[0]
                                               - targets[1].second_highest_point[0]) / sum2)
                sd.putNumber("Multiply 1", sum1 * sum2)
                sd.putNumber("Multiply 2", abs(targets[0].second_highest_point[0]
                                               - targets[1].second_highest_point[0]) * sum2)
                sd.putNumber("approx yaw", ((sum2 / sum1) - 0.3587) / 0.0053)

        else:
            sd.putNumber("X Error", ERROR_VALUE)
        outputStream.putFrame(drawing)


if __name__ == "__main__":
    main()

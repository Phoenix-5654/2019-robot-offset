#!/usr/bin/env python3
from heapq import nsmallest
from math import sqrt

import cv2
import numpy as np
from networktables import NetworkTables

from cscore import CameraServer, UsbCamera, VideoSource

HORIZONTAL_RES = 240
VERTICAL_RES = 320


class GripPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """
    
    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsv_threshold_hue = [64.74820143884892, 101.67235494880545]
        self.__hsv_threshold_saturation = [220.14388489208633, 255.0]
        self.__hsv_threshold_value = [43.57014388489208, 200.6058020477816]

        self.hsv_threshold_output = None

        self.__find_contours_input = self.hsv_threshold_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__filter_contours_contours = self.find_contours_output
        self.__filter_contours_min_area = 100.0
        self.__filter_contours_min_perimeter = 0
        self.__filter_contours_min_width = 0
        self.__filter_contours_max_width = 1000
        self.__filter_contours_min_height = 0
        self.__filter_contours_max_height = 1000
        self.__filter_contours_solidity = [0, 100]
        self.__filter_contours_max_vertices = 1000000
        self.__filter_contours_min_vertices = 0
        self.__filter_contours_min_ratio = 0
        self.__filter_contours_max_ratio = 1000

        self.filter_contours_output = None

        self.__convex_hulls_contours = self.filter_contours_output

        self.convex_hulls_output = None


    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step HSV_Threshold0:
        self.__hsv_threshold_input = source0
        (self.hsv_threshold_output) = self.__hsv_threshold(self.__hsv_threshold_input, self.__hsv_threshold_hue, self.__hsv_threshold_saturation, self.__hsv_threshold_value)

        # Step Find_Contours0:
        self.__find_contours_input = self.hsv_threshold_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input, self.__find_contours_external_only)

        # Step Filter_Contours0:
        self.__filter_contours_contours = self.find_contours_output
        (self.filter_contours_output) = self.__filter_contours(self.__filter_contours_contours, self.__filter_contours_min_area, self.__filter_contours_min_perimeter, self.__filter_contours_min_width, self.__filter_contours_max_width, self.__filter_contours_min_height, self.__filter_contours_max_height, self.__filter_contours_solidity, self.__filter_contours_max_vertices, self.__filter_contours_min_vertices, self.__filter_contours_min_ratio, self.__filter_contours_max_ratio)

        # Step Convex_Hulls0:
        self.__convex_hulls_contours = self.filter_contours_output
        (self.convex_hulls_output) = self.__convex_hulls(self.__convex_hulls_contours)


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
        return cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if(external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
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
            x,y,w,h = cv2.boundingRect(contour)
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
    inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    return inst, camera


def find_alignment_center(shapes):
    dists = {}

    for i in range(len(shapes)):
        for j in range(i + 1, len(shapes)):
            dists[distance(shapes[i][0], shapes[j][0])] = get_average_point(shapes[i][1][0],
                                                                            shapes[j][1][0])
    try:
        return dists[min(dists.keys())]
    except ValueError:
        return None


def distance(point1: tuple, point2: tuple):
    return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))


def get_average_point(point1, point2):
    return (int(point1[0] + point2[0]) // 2,
            int(point1[1] + point2[1]) // 2)


def main():
    NetworkTables.initialize(server='10.56.54.2')

    widgets = {}

    setpoint = (VERTICAL_RES // 2, HORIZONTAL_RES // 2)

    sd = NetworkTables.getTable('Vision')

    inst, camera = start_camera()

    pipeline = GripPipeline()

    cvSink = inst.getVideo(camera=camera)

    outputStream = inst.putVideo(
        'Hatch Panels Convex Hulls', HORIZONTAL_RES, VERTICAL_RES)

    img = np.zeros(shape=(VERTICAL_RES, HORIZONTAL_RES, 3), dtype=np.uint8)

    while True:

        shapes = []

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

                shapes.append((nsmallest(2, box, key=lambda x: x[1])[-1], rect))
                
                cv2.drawContours(drawing, [box], 0, (0, 0, 255), 4)

            for i in range(len(pipeline.convex_hulls_output)):
                cv2.drawContours(
                    drawing, pipeline.convex_hulls_output, i, (255, 0, 0), 3)

            if len(shapes) == 1:
                if abs(shapes[0][1][2]) > 45:
                    alignment_center = (VERTICAL_RES, int(shapes[0][1][0][1]))
                else:
                    alignment_center = (0, int(shapes[0][1][0][1]))
            else:
                alignment_center = find_alignment_center(shapes)

            if alignment_center is not None:

                cv2.circle(drawing, alignment_center, 5, (0, 255, 0), 5)

                sd.putNumber('X Error', setpoint[0] - alignment_center[0])
                sd.putNumber('Y Error', setpoint[1] - alignment_center[1])

        outputStream.putFrame(drawing)


if __name__ == "__main__":
    main()

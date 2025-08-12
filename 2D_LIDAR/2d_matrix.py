#!/usr/bin/env python3
"""
RPLIDAR A2M8 realtime viewer using PyQt5

Features:
- Reads scans from RPLIDAR via rplidar library in a background thread
- Emits scans to main GUI thread
- Paints points on a QWidget using QPainter (fast enough for typical rates)
- Configurable serial port and scale

Requirements:
    pip install pyqt5 rplidar numpy

Usage:
    python rplidar_pyqt5_viewer.py --port /dev/ttyUSB0
    or on Windows: --port COM3

Note: adjust SCALE (pixels per meter) for zoom level.
"""

import sys
import math
import argparse
import time
from collections import deque

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QVBoxLayout, QLabel, QHBoxLayout, QPushButton, QSpinBox, QFileDialog
from PyQt5.QtGui import QPainter, QColor, QPen

try:
    from rplidar import RPLidar
except Exception as e:
    RPLidar = None

import numpy as np


class LidarThread(QThread):
    scan_ready = pyqtSignal(list)  # emits list of (angle_deg, distance_mm)

    def __init__(self, port, baudrate=115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self._running = True
        self._lidar = None

    def run(self):
        if RPLidar is None:
            print("rplidar library not installed. Install via: pip install rplidar")
            return

        try:
            self._lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=1)
            self._lidar.start_motor()
            time.sleep(0.5)  # đợi ổn định
            # start motor if needed (library normally handles it)
            for scan in self._lidar.iter_scans():
                if not self._running:
                    break
                # scan is list of tuples (quality, angle, distance)
                points = [(s[1], s[2]) for s in scan]
                self.scan_ready.emit(points)
        except Exception as e:
            print("Lidar thread error:", e)
        finally:
            try:
                if self._lidar:
                    self._lidar.stop()
                    self._lidar.stop_motor()
                    self._lidar.disconnect()
            except Exception:
                pass

    def stop(self):
        self._running = False
        self.wait(1000)


class LidarCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(600, 600)
        self.points = []  # list of (x_pixels, y_pixels)
        self.bg_color = QColor(10, 10, 10)
        self.point_color = QColor(0, 255, 0)
        self.trail = deque(maxlen=5)  # keep a few recent frames if desired

    def update_scan(self, points_xy):
        # points_xy: list of (x_m, y_m)
        self.trail.append(points_xy)
        self.points = points_xy
        # trigger repaint
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), self.bg_color)

        w = self.width()
        h = self.height()
        cx = w // 2
        cy = h // 2

        # draw crosshair
        pen = QPen(QColor(80, 80, 80))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawLine(cx, 0, cx, h)
        painter.drawLine(0, cy, w, cy)

        # draw grid lines
        grid_size = self.property('pixels_per_meter') or 100
        pen = QPen(QColor(60, 60, 60))
        pen.setStyle(Qt.DotLine)
        painter.setPen(pen)

        # Vertical lines with labels
        for x in range(cx % grid_size, w, grid_size):
            painter.drawLine(x, 0, x, h)
            # distance from center (meters)
            dist_m = abs((x - cx) / grid_size)
            if dist_m > 0:
                painter.drawText(x + 2, cy - 2, f"{dist_m:.1f}m")

        # Horizontal lines with labels
        for y in range(cy % grid_size, h, grid_size):
            painter.drawLine(0, y, w, y)
            dist_m = abs((cy - y) / grid_size)
            if dist_m > 0:
                painter.drawText(cx + 4, y - 2, f"{dist_m:.1f}m")

        # draw points
        pen = QPen(self.point_color)
        pen.setWidth(4)
        painter.setPen(pen)

        ppm = self.property('pixels_per_meter') or 100

        # optional: draw trails from recent frames with fading
        alpha_step = 200 // max(1, len(self.trail))
        for t_idx, frame in enumerate(self.trail):
            a = 255 - (len(self.trail) - 1 - t_idx) * alpha_step
            if a < 40:
                a = 40
            pen.setColor(QColor(self.point_color.red(), self.point_color.green(), self.point_color.blue(), a))
            painter.setPen(pen)
            for x_m, y_m in frame:
                px = int(cx + x_m * ppm)
                py = int(cy - y_m * ppm)  # y axis flip
                painter.drawPoint(px, py)

        painter.end()



class MainWindow(QMainWindow):
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.setWindowTitle('RPLIDAR A2M8 — PyQt5 Live Viewer')

        self.port = port
        self.baudrate = baudrate

        self.canvas = LidarCanvas(self)
        self.canvas.setProperty('pixels_per_meter', 100)  # default scale

        # controls
        controls = QWidget()
        hl = QHBoxLayout()
        controls.setLayout(hl)

        self.scale_label = QLabel('Scale (px/m):')
        self.scale_spin = QSpinBox(); self.scale_spin.setRange(10, 1000); self.scale_spin.setValue(100)
        self.scale_spin.valueChanged.connect(self.on_scale_changed)

        self.start_btn = QPushButton('Start')
        self.stop_btn = QPushButton('Stop')
        self.stop_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.start_lidar)
        self.stop_btn.clicked.connect(self.stop_lidar)

        hl.addWidget(self.scale_label)
        hl.addWidget(self.scale_spin)
        hl.addWidget(self.start_btn)
        hl.addWidget(self.stop_btn)
        hl.addStretch()

        container = QWidget()
        vbox = QVBoxLayout()
        container.setLayout(vbox)
        vbox.addWidget(self.canvas)
        vbox.addWidget(controls)
        self.setCentralWidget(container)

        self.lidar_thread = None

        # refresh timer to repaint at ~30 FPS if needed
        self.repaint_timer = QTimer(self)
        self.repaint_timer.timeout.connect(self.canvas.update)
        self.repaint_timer.start(33)

    def on_scale_changed(self, val):
        self.canvas.setProperty('pixels_per_meter', val)
        self.canvas.update()

    def start_lidar(self):
        if self.lidar_thread and self.lidar_thread.isRunning():
            return
        self.lidar_thread = LidarThread(self.port, baudrate=self.baudrate)
        self.lidar_thread.scan_ready.connect(self.on_new_scan)
        self.lidar_thread.start()
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_lidar(self):
        if self.lidar_thread:
            self.lidar_thread.stop()
            self.lidar_thread = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def closeEvent(self, event):
        self.stop_lidar()
        super().closeEvent(event)

    def on_new_scan(self, scan_points):
        # convert polar -> cartesian (meters) and send to canvas
        pts = []
        for angle_deg, dist_mm in scan_points:
            if dist_mm <= 0:
                continue
            #get the angle from 0 to 180
            if not (0 <=angle_deg <= 180):
                continue
            r_m = dist_mm / 1000.0
            theta = math.radians(angle_deg)
            print(angle_deg)
            x = r_m * math.cos(theta)
            y = r_m * math.sin(theta)
            pts.append((x, y))
        self.canvas.update_scan(pts)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=str, default=None, help='Serial port, e.g. /dev/ttyUSB0 or COM3')
    p.add_argument('--baud', type=int, default=115200, help='Baudrate for RPLidar')
    return p.parse_args()


def main():
    args = parse_args()
    if args.port is None:
        print('Please supply --port (e.g. /dev/ttyUSB0 or COM3)')
        sys.exit(1)

    app = QApplication(sys.argv)
    w = MainWindow(args.port, baudrate=args.baud)
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

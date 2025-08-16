import sys
import math
import argparse
import time
from collections import deque

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QPoint
from PyQt5.QtWidgets import (
    QApplication, QWidget, QMainWindow, QVBoxLayout, QLabel,
    QHBoxLayout, QPushButton, QSpinBox
)
from PyQt5.QtGui import QPainter, QColor, QPen

from planning_path import *
try:
    from rplidar import RPLidar
except Exception:
    RPLidar = None

import numpy as np


class LidarCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(600, 600)
        # Giữ (x_m, y_m) sau khi đã đổi từ polar
        self.points = []                 # list[(x_m, y_m)]
        self.trail = deque(maxlen=5)     # các frame gần nhất (vệt mờ)
        self.bg_color = QColor(10, 10, 10)
        self.point_color = QColor(0, 255, 0)

    def update_scan(self, points_xy):
        """points_xy: list of (x_m, y_m)"""
        self.trail.append(points_xy)
        self.points = points_xy
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.fillRect(self.rect(), self.bg_color)

            w = self.width()
            h = self.height()
            cx = w // 2
            cy = h // 2

            # pixels per meter
            ppm = self.property('pixels_per_meter') or 100

            # === Trục chính (X ngang, Y dọc) ===
            pen = QPen(QColor(80, 80, 80))
            pen.setWidth(1)
            painter.setPen(pen)
            painter.drawLine(0, cy, w, cy)  # trục X (0°–180°)
            painter.drawLine(cx, 0, cx, h)  # trục Y (90°–270°)

            # === Lưới mét ===
            pen = QPen(QColor(60, 60, 60))
            pen.setStyle(Qt.DotLine)
            painter.setPen(pen)

            grid_size = ppm  # mỗi 1m
            # dọc
            for x in range(cx % grid_size, w, grid_size):
                painter.drawLine(x, 0, x, h)
                dist_m = abs((x - cx) / grid_size)
                if dist_m > 0:
                    painter.drawText(x + 2, cy - 2, f"{dist_m:.1f}m")
            # ngang
            for y in range(cy % grid_size, h, grid_size):
                painter.drawLine(0, y, w, y)
                dist_m = abs((cy - y) / grid_size)
                if dist_m > 0:
                    painter.drawText(cx + 4, y - 2, f"{dist_m:.1f}m")

            # === Vòng tròn tham chiếu + nhãn góc (đúng CW, 0° tại X+) ===
            ref_radius_m = 1.2
            ref_radius_px = int(ppm * ref_radius_m)

            pen = QPen(QColor(120, 120, 120))
            pen.setWidth(1)
            painter.setPen(pen)
            painter.drawEllipse(QPoint(cx, cy), ref_radius_px, ref_radius_px)

            # Nhãn góc mỗi 30°
            pen = QPen(QColor(150, 150, 150))
            painter.setPen(pen)
            font = painter.font()
            font.setPointSize(9)
            painter.setFont(font)

            for ang in range(0, 360, 30):
                # Đổi sang CW: dùng -ang khi tính vị trí
                theta = math.radians(ang)
                # Vị trí chữ
                x_txt = cx + math.sin(theta) * (ref_radius_px + 10)  # ngang
                y_txt = cy - math.cos(theta) * (ref_radius_px + 10)  # dọc (lật trục Y)
                painter.drawText(int(x_txt) - 10, int(y_txt), f"{ang}°")

            # === Vẽ điểm quét (trail + frame hiện tại) ===
            pen = QPen(self.point_color)
            pen.setWidth(4)

            # Trail (mờ dần)
            if len(self.trail) > 0:
                alpha_step = 200 // max(1, len(self.trail))
                for t_idx, frame in enumerate(self.trail):
                    a = 255 - (len(self.trail) - 1 - t_idx) * alpha_step
                    if a < 40:
                        a = 40
                    pen.setColor(QColor(self.point_color.red(),
                                        self.point_color.green(),
                                        self.point_color.blue(), a))
                    painter.setPen(pen)
                    for x_m, y_m in frame:
                        px = int(cx + x_m * ppm)   
                        py = int(cy + y_m * ppm)
                        #print("Px: ",px)
                        #print("Py: ",py)
                        painter.drawPoint(px, py)
        finally:
            painter.end()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.canvas_raw = LidarCanvas()
        self.canvas_smooth = LidarCanvas()

        container = QWidget()
        layout = QHBoxLayout(container)
        layout.addWidget(self.canvas_raw)
        layout.addWidget(self.canvas_smooth)
        self.setCentralWidget(container)

        self.setWindowTitle("Lidar Raw vs Filtered Demo")

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_custom_data)
        self.timer.start(500)

        self.angle = 0

    def update_custom_data(self):
        # --- Tạo data giả (raw) ---
        points_raw = []
        dist_m = 1.0 + 0.4 * math.sin(math.radians(self.angle))
        x = math.cos(math.radians(self.angle)) * dist_m
        y = math.sin(math.radians(self.angle)) * dist_m
        points_raw.append((x, y))

        # --- Xử lý giả (filtered) ---
        # Ở đây mình demo = giảm biên độ nhiễu
        dist_m_smooth = 1.0 + 0.2 * math.sin(math.radians(self.angle))
        x_s = math.cos(math.radians(self.angle)) * dist_m_smooth
        y_s = math.sin(math.radians(self.angle)) * dist_m_smooth
        points_smooth = [(x_s, y_s)]

        # Update 2 canvas
        self.canvas_raw.update_scan(points_raw)
        self.canvas_smooth.update_scan(points_smooth)

        self.angle += 15


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
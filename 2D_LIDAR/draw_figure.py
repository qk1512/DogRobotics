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


import numpy as np

class LidarCanvas(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setMinimumSize(600,600)
        
        self.points = []
        self.trail = deque(maxlen=5)
        self.bg_color = QColor(10,10,10)
        self.point_color = QColor(0,255,0)

    def update_scan(self,points_xy):
        self.trail.append(points_xy)
        self.points = points_xy
        self.update()

    def paintEvent(self,event):
        painter = QPainter(self)
        try:
            painter.fillRect(self.rect(), self.bg_color)

            w = self.width()
            h = self.height()
            cx = w //2 
            cy = h //2

            #pixels per meter
            ppm = self.property('pixels_per_meter') or 100

            pen = QPen(QColor(80,60,60))
            pen.setStyle(Qt.DotLine)
            painter.setPen(pen)

            grid_size = ppm

            for x in range(cx % grid_size,w, grid_size):
                painter.drawLine(x, 0, x, h)
                dist_m = abs((x-cx)/grid_size)
                if dist_m > 0:
                    painter.drawText(x + 2, cy - 2, f"{dist_m:.1f}m")
            
            for y in range(cy % grid_size, h , grid_size):
                painter.drawLine(0, y, w, y)
                dist_m = abs((cy-y)/grid_size)
                if dist_m > 0:
                    painter.drawText(cx + 4, y - 2, f"{dist_m:.1f}m")


            ref_radius_m = 1.2
            ref_radius_px = int(ppm*ref_radius_m)

            pen = QPen(QColor(150,150,150))
            painter.setPen(pen)
            font = painter.font()
            font.setPointSize(9)
            painter.setFont(font)

            for ang in range(0,360, 30):
                theta = math.radians(ang)
                
                x_txt = cx + math.sin(theta) * (ref_radius_px + 10)
                y_txt = cy - math.cos(theta) * (ref_radius_px + 10)

                painter.drawText(int(x_txt) - 10, int(y_txt), f"{ang}")

            pen = QPen(self.point_color)
            pen.setWidth(4)

            if len(self.trail) > 0:
                alpha_step = 200 // max(1,len(self.trail))
                for t_idx, frame in enumerate(self.trail):
                    a = 255 - (len(self.trail) - 1 - t_idx) * alpha_step
                    if a < 40:
                        a = 40
                    pen.setColor(QColor(self.point_color.red(),
                                        self.point_color.green(),
                                        self.point_color.blue(),a))
                    painter.setPen(pen)
                    for x_m, y_m in frame:
                        px = int(cx + x_m * ppm)
                        py = int(cy + y_m * ppm)
                        painter.drawPoint(px,py)
        finally:
            painter.end()



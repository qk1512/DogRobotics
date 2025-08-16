class MainWindown(QMainWindow):
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.setWindowTitle('RPLIDAR A2M8 - PyQt5 Live Viewer')

        self.port = port 
        self.baudrate = baudrate

        self.canvas_raw = LidarCanvas(self)
        self.canvas_smooth = LidarCanvas(self)

        self.canvas_smooth.setProperty('pixels_per_meter', 100)

        canvases = QWidget()
        canv_layout = QHBoxLayout(canvases)
        canv_layout.addWidget(self.canvas_raw)
        canv_layout.addWidget(self.canvas_smooth)

        controls = QWidget()
        ctrl_layout = QHBoxLayout(controls)

        self.scale_label = QLabel('Scale (px/m):')
        self.scale_spin = QSpinBox()
        self.scale_spin.setRange(10,1000)
        self.scale_spin.setValue(100)
        self.scale_spin.valueChanged.connect(self.on_scale_changed)

        self.start_btn = QPushButton('Start')
        self.stop_btn = QPushButton('Stop')
        self.stop_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.start_lidar)
        self.stop_btn.clicked.connect(self.stop_lidar)

        ctrl_layout.addWidget(self.scale_label)
        ctrl_layout.addWidget(self.scale_spin)
        ctrl_layout.addWidget(self.start_btn)
        ctrl_layout.addWidget(self.stop_btn)
        ctrl_layout.addStretch()


        container = QWidget()
        vbox = QVBoxLayout(container)
        vbox.addWidget(canvases)
        vbox.addWidget(controls)
        self.setCentralWidget(container)

        self.lidar_thread = None
        
        #self.planner = SimpleReactivePlanner()
        #self.smoother = LidarSmoother()

    def update_canvases(self):
        self.canvas_raw.update()
        self.canvas_smooth.update()

    def on_scale_changed(self,val):
        self.canvas_smooth.setProperty('pixels_per_meter',val)
        self.canvas_smooth.update()

    def start_lidar(self):
        if self.lidar_thread and self.lidar_thread.isRunning():
            
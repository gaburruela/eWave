import sys
import numpy as np
import pyqtgraph as pg
import os
from PySide6.QtWidgets import QLabel
from PySide6.QtGui import QPixmap, QFontDatabase, QFont, QPainter, QImage
from PySide6.QtCore import Qt, QRectF
from PySide6.QtSvg import QSvgRenderer

from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QDialog,
    QLineEdit,
    QFormLayout,
    QDialogButtonBox
)
from PySide6.QtCore import QTimer

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Monitoreo de Sensores")

        self.central = QWidget()
        self.setCentralWidget(self.central)

        self.main_layout = QVBoxLayout(self.central)

        self.main_layout.setContentsMargins(40, 10, 40, 70)
        self.main_layout.setSpacing(15)

        # Fuentes
        font_id_poppins = QFontDatabase.addApplicationFont(
            "Fonts/Poppins/Poppins-Bold.ttf"
        )

        font_id_inter = QFontDatabase.addApplicationFont(
            r"C:\eWave\eWave\Python Code\New interface\Fonts\Inter\Inter-VariableFont_opsz,wght.ttf"
        )
        
        family_poppins = QFontDatabase.applicationFontFamilies(font_id_poppins)[0]
        family_inter = QFontDatabase.applicationFontFamilies(font_id_inter)[0]

        self.family_poppins = family_poppins
        self.family_inter = family_inter

        font_poppins = QFont(family_poppins)
        font_poppins.setPointSize(35)
        font_poppins.setBold(True)

        font_inter = QFont(family_inter)
        font_inter.setPointSize(12)
        font_inter.setBold(False)

        # Fondo
        self.background = QLabel(self)

        self.pixmap_original = QPixmap("Graphic Components/Background.jpg")

        self.background.setPixmap(
            self.pixmap_original.scaled(
                self.size(),
                Qt.KeepAspectRatioByExpanding,
                Qt.SmoothTransformation
            )
        )

        self.background.setGeometry(self.rect())
        self.background.lower()  # envía el fondo detrás de todo

        # Patrón SVG inferior
        self.svg_renderer = QSvgRenderer("Graphic Components/Patrón 2.svg")

        self.tile_w = 200
        self.tile_h = 100

        self.tile_image = None
        self.bottom_pattern = QLabel(self)
        self.background.lower()
        self.bottom_pattern.raise_()
        self.central.raise_()
        
        # Título

        self.titulo = QLabel("MONITOREO DE SENSORES", self)
        self.main_layout.addWidget(self.titulo)

        self.titulo.setFont(font_poppins)

        self.titulo.setAlignment(Qt.AlignCenter)

        self.titulo.setStyleSheet("""
            color: #FCAF08;
            background: transparent;
        """)

        
        # Widget contenedor
        self.panel_graficas = QWidget()
        self.main_layout.addWidget(
            self.panel_graficas,
            stretch=3
        )

        # Layout dentro del widget
        layout_graficas = QHBoxLayout()
        self.panel_graficas.setLayout(layout_graficas)

        # Crear gráficas
        white = (252, 255, 255)
        self.Bond_graph = pg.PlotWidget(title="Sensor Bond")
        self.noBond_graph = pg.PlotWidget(title="Sensor noBond")

        self.Bond_graph.setTitle(
            "Sensor Bond",
            color="#FFFFFF",
        )

        self.noBond_graph.setTitle(
            "Sensor noBond",
            color="#FFFFFF",
        )


        self.Bond_graph.setTitle(
            '<span style="font-family: Inter; font-size: 12pt; color: white;">Sensor Bond</span>'
        )

        self.noBond_graph.setTitle(
            '<span style="font-family: Inter; font-size: 12pt; color: white;">Sensor No Bond</span>'
        )

        axis_left_Bond = self.Bond_graph.getAxis('left')
        axis_bottom_Bond = self.Bond_graph.getAxis('bottom')

        axis_left_Bond.setStyle(tickFont=font_inter)
        axis_bottom_Bond.setStyle(tickFont=font_inter)

        axis_left_noBond = self.noBond_graph.getAxis('left')
        axis_bottom_noBond = self.noBond_graph.getAxis('bottom')

        axis_left_noBond.setStyle(tickFont=font_inter)
        axis_bottom_noBond.setStyle(tickFont=font_inter)

        

        self.Bond_graph.setBackground(None)
        self.noBond_graph.setBackground(None)

        self.Bond_graph.setStyleSheet("""
            background: transparent;
            border: none;
        """)

        self.noBond_graph.setStyleSheet("""
            background: transparent;
            border: none;
        """)

        self.Bond_graph.setLabel(
        'left',
        'Altura Pico-Pico',
        units='mm'
        )

        self.Bond_graph.setLabel(
            'bottom',
            'Tiempo',
        units='s'
        )

        self.noBond_graph.setLabel(
        'left',
        'Altura Pico-Pico',
        units='mm'
        )

        self.noBond_graph.setLabel(
        'bottom',
        'Tiempo',
        units='s'
        )

        # Agregar al layout
        layout_graficas.addWidget(self.Bond_graph)
        layout_graficas.addWidget(self.noBond_graph)


        layout_graficas.setContentsMargins(
            0,    # izquierda
            0,  # arriba
            0,    # derecha
            0     # abajo
        )
        layout_graficas.setSpacing(0)        

        axis_pen = pg.mkPen(color=white, width=2)

        self.Bond_graph.getAxis('left').setPen(axis_pen)
        self.Bond_graph.getAxis('bottom').setPen(axis_pen)

        self.noBond_graph.getAxis('left').setPen(axis_pen)
        self.noBond_graph.getAxis('bottom').setPen(axis_pen)

        self.Bond_graph.getAxis('left').setTextPen(white)
        self.Bond_graph.getAxis('bottom').setTextPen(white)

        self.noBond_graph.getAxis('left').setTextPen(white)
        self.noBond_graph.getAxis('bottom').setTextPen(white)

        # Datos
        self.n = 200
        self.x = np.arange(self.n)

        self.Bond_data = np.zeros(self.n)
        self.noBond_data = np.zeros(self.n)

        # Curvas
        self.Bond_curve = self.Bond_graph.plot(self.x, self.Bond_data, pen=pg.mkPen(color=white, width=4))
        self.noBond_curve = self.noBond_graph.plot(self.x, self.noBond_data, pen=pg.mkPen(color=white, width=4))

        # Tiempo de simulación
        self.t = 0

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(40)





        # Extra sensor data

        self.extra_data_panel = QWidget(self)
        self.extra_data_grid = QGridLayout()
        self.extra_data_panel.setLayout(self.extra_data_grid)   

        self.bottom_section = QWidget()

        bottom_layout = QHBoxLayout(self.bottom_section)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        bottom_layout.setSpacing(20)

        self.main_layout.addWidget(
            self.bottom_section,
            stretch=2
        )

        self.data_container = QWidget(self)
        bottom_layout.addStretch()

        self.data_container.setObjectName("dataContainer")  

        container_layout = QVBoxLayout(self.data_container)

        self.data_title = QLabel("DATOS DEL EXPERIMENTO")
        self.data_title.setAlignment(Qt.AlignCenter)
        self.data_title.setFont(font_poppins)
        self.data_title.setStyleSheet("""
            color: white;
        """)

        container_layout.addWidget(self.data_title)
        container_layout.addWidget(self.extra_data_panel)

        self.data_container.setStyleSheet("""
        #dataContainer {
            background-color: rgba(98,98,98,100);
            border-radius: 15px;
        }
        """)
        
        self.extra_data_grid.setContentsMargins(0, 0, 0, 0)
        self.extra_data_grid.setHorizontalSpacing(10)
        self.extra_data_grid.setVerticalSpacing(0)

        self.extra_data_labels = []

        extra_data_titles = [
            "Humedad [%]",
            "Temperatura ambiente [°C]",
            "Temperatura del agua [°C]",
            "Temperatura del motor [°C]",
            "Velocidad del motor [rpm]",
            "Altura PP Bond [mm]",
            "Altura PP noBond [mm]",
            "Frecuencia Bond [Hz]",
            "Frecuencia noBond [Hz]",
            "Longitud de onda [m]",
            "Número de olas"
        ]

        self.extra_data_title_labels = []

        for i, title in enumerate(extra_data_titles):

            container = QWidget()

            layout = QVBoxLayout(container)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)

            title_label = QLabel(title)

            if title in [
                "Altura PP Bond [mm]",
                "Altura PP noBond [mm]",
                "Frecuencia Bond [Hz]",
                "Frecuencia noBond [Hz]",
                "Longitud de onda [m]"
            ]:
                value_label = QLabel("0.00±0.00")
            else:
                value_label = QLabel("0.00")

            title_label.setAlignment(Qt.AlignCenter)
            value_label.setAlignment(Qt.AlignCenter)

            title_label.setStyleSheet("""
                color: white;
                background: transparent;
            """)

            value_label.setStyleSheet("""
                color: white;
                background: transparent;
            """)

            layout.addWidget(title_label)
            layout.addWidget(value_label)

            row = i // 3
            col = i % 3

            self.extra_data_grid.addWidget(container, row, col)
            self.extra_data_title_labels.append(title_label)
            self.extra_data_labels.append(value_label)


        self.params_button = QPushButton(
            "DEFINIR\nPARÁMETROS\nDEL\nEXPERIMENTO"
        )

        self.params_button.setMinimumSize(180, 90)

        self.params_button.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding
        )

        self.params_button.setStyleSheet("""
            QPushButton{
                background-color: rgba(0, 161, 169, 255);
                color: white;
                border-radius: 15px;
            }

            QPushButton:hover{
                background-color: #2196F3;
            }

            QPushButton:pressed{
                background-color: #0D47A1;
            }
        """)

        self.params_button.clicked.connect(
            self.open_parameters_dialog
        )

        self.start_button = QPushButton(
        "INICIAR\nEXPERIMENTO"
        )

        self.start_button.setMinimumSize(180, 45)
        self.start_button.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding
        )

        self.start_button.setStyleSheet("""
            QPushButton{
                background-color: #FCAF08;
                color: white;
                border-radius: 15px;
                font-weight: bold;
            }

            QPushButton:hover{
                background-color: #ffbe2e;
            }

            QPushButton:pressed{
                background-color: #d89200;
            }
        """)

        self.stop_button = QPushButton(
            "DETENER\nEXPERIMENTO"
        )

        self.stop_button.setMinimumSize(180, 90)

        self.stop_button.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding
        )


        self.stop_button.setStyleSheet("""
            QPushButton{
                background-color: #D32F2F;
                color: white;
                border-radius: 15px;
                font-weight: bold;
            }

            QPushButton:hover{
                background-color: #E53935;
            }

            QPushButton:pressed{
                background-color: #B71C1C;
            }
        """)

        bottom_layout.addStretch()
        bottom_layout.addWidget(self.params_button)
        bottom_layout.addWidget(self.start_button)
        bottom_layout.addWidget(self.stop_button)

        bottom_layout.addSpacing(20)

        bottom_layout.addWidget(self.data_container)

        QTimer.singleShot(0, self.initialize_ui)

    def initialize_ui(self):
        self.update_pattern_size()

        self.main_layout.setContentsMargins(
            40,
            10,
            40,
            self.tile_h + 20
        )

        self.data_container.setMaximumWidth(
            int(self.width() * 0.7)
        )

        self.bottom_pattern.setGeometry(
            0,
            self.height() - self.tile_h - 3,
            self.width(),
            self.tile_h
        )

        self.draw_bottom_pattern()

        self.update_fonts()

    def resizeEvent(self, event):

        self.update_pattern_size()
        self.main_layout.setContentsMargins(
            40,
            10,
            40,
            self.tile_h + 20
        )

        self.background.setGeometry(self.rect())

        self.background.setPixmap(
            self.pixmap_original.scaled(
            self.size(),
            Qt.KeepAspectRatioByExpanding,
            Qt.SmoothTransformation
            )
        )

        self.data_container.setMaximumWidth(
            int(self.width() * 0.7)
        )

        #self.data_container.setMinimumWidth(825)

        self.bottom_pattern.setGeometry(
            0,
            self.height() - self.tile_h - 3,
            self.width(),
            self.tile_h
        )


        self.draw_bottom_pattern()                          
        self.update_fonts()


        super().resizeEvent(event)

    def update_fonts(self):

        scale = min(
            self.width() / 1400,
            self.height() / 700
        )

        title_size = max(18, int(35 * scale))

        data_title_size = max(16, int(25 * scale))

        data_label_size = max(9, int(14 * scale))

        data_value_size = max(8, int(12 * scale))

        graph_size = max(8, int(12 * scale))

        # Título principal
        font = QFont(self.family_poppins)
        font.setBold(True)
        font.setPointSize(title_size)

        self.titulo.setFont(font)

        # Título datos
        font = QFont(self.family_poppins)
        font.setBold(True)
        font.setPointSize(data_title_size)

        self.data_title.setFont(font)

        # Etiquetas de datos
        for lbl in self.extra_data_title_labels:

            f = QFont(self.family_inter)
            f.setBold(False)
            f.setPointSize(data_label_size)

            lbl.setFont(f)

        # Valores
        for lbl in self.extra_data_labels:

            f = QFont(self.family_inter)
            f.setPointSize(data_value_size)

            lbl.setFont(f)

        # Ejes gráficas
        axis_font = QFont(self.family_inter)
        axis_font.setPointSize(graph_size)

        self.Bond_graph.getAxis('left').setStyle(
            tickFont=axis_font
        )

        self.Bond_graph.getAxis('bottom').setStyle(
            tickFont=axis_font
        )

        self.noBond_graph.getAxis('left').setStyle(
            tickFont=axis_font
        )

        self.noBond_graph.getAxis('bottom').setStyle(
            tickFont=axis_font
        )

        button_font = QFont(self.family_poppins)
        button_font.setBold(True)
        button_font.setPointSize(max(10, int(18 * scale)))

        self.params_button.setFont(button_font)
        self.start_button.setFont(button_font)
        self.stop_button.setFont(button_font)

    def draw_bottom_pattern(self):
        w = self.bottom_pattern.width()
        h = self.bottom_pattern.height()

        if w <= 1 or h <= 1:
            return

        final = QImage(w, h, QImage.Format_ARGB32)
        final.fill(Qt.transparent)

        painter = QPainter(final)

        x = 0
        while x < w:
            painter.drawImage(x, 0, self.tile_image)
            x += self.tile_w

        painter.end()

        self.bottom_pattern.setPixmap(QPixmap.fromImage(final))

    def update_pattern_size(self):

        svg_size = self.svg_renderer.defaultSize()

        self.tile_h = max(
            60,
            min(
                180,
                int(self.height() * 0.1)
            )
        )

        self.tile_w = int(
            self.tile_h *
            svg_size.width() /
            svg_size.height()
        )

        self.tile_image = QImage(
            self.tile_w,
            self.tile_h,
            QImage.Format_ARGB32
        )

        self.tile_image.fill(Qt.transparent)

        painter = QPainter(self.tile_image)

        self.svg_renderer.render(
            painter,
            QRectF(
                0,
                0,
                self.tile_w,
                self.tile_h
            )
        )

        painter.end()

    def open_parameters_dialog(self):

        dialog = QDialog(self)
        dialog.setWindowTitle("Parámetros del experimento")
        dialog.resize(400, 250)

        layout = QFormLayout(dialog)

        self.freq_input = QLineEdit()
        self.duration_input = QLineEdit()
        self.speed_input = QLineEdit()

        layout.addRow(
            "Frecuencia del variador [Hz]:",
            self.freq_input
        )

        layout.addRow(
            "Brazo de la paleta [mm]:",
            self.duration_input
        )

        layout.addRow(
            "Cantidad de olas:",
            self.speed_input
        )

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok |
            QDialogButtonBox.Cancel
        )

        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)

        layout.addWidget(buttons)

        if dialog.exec():

            self.sample_frequency = self.freq_input.text()
            self.experiment_duration = self.duration_input.text()
            self.motor_speed = self.speed_input.text()

            print("Frecuencia:", self.sample_frequency)
            print("Duración:", self.experiment_duration)
            print("Velocidad:", self.motor_speed)

            self.sample_frequency = float(
                self.freq_input.text()
            )

            self.experiment_duration = float(
                self.duration_input.text()
            )

            self.motor_speed = float(
                self.speed_input.text()
            )
         
    def update_data(self):

        # Simulación de temperatura
        temperatura = (
            25
            + 2*np.sin(self.t/50)
            + np.random.normal(0, 0.1)
        )

        # Simulación de presión
        presion = (
            25
            + 2*np.sin(3.14-self.t/50)
            + np.random.normal(0, 0.1)
        )

        # Desplazar datos a la izquierda
        self.Bond_data[:-1] = self.Bond_data[1:]
        self.noBond_data[:-1] = self.noBond_data[1:]

        # Agregar nueva muestra
        self.Bond_data[-1] = temperatura
        self.noBond_data[-1] = presion

        # Actualizar gráficas
        self.Bond_curve.setData(self.x, self.Bond_data)
        self.noBond_curve.setData(self.x, self.noBond_data)


        

        self.t += 1

app = QApplication(sys.argv)

window = MainWindow()
window.showMaximized()

app.exec()
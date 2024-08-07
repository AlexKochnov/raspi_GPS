import sys
import traceback

import numpy as np
from PyQt5.QtGui import QFont, QTextCharFormat, QBrush, QColor, QCursor
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit, \
    QLineEdit, QLabel, QMessageBox, QTableWidget, QTableWidgetItem, QHeaderView, QComboBox, QToolTip, QDialog
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, Qt, QSize, QObject
import pandas as pd

import Settings
import UBXMessages
from GNSS import GNSS
# мои собственные готовые модули, которые имортируются и не зменяются
from Storage import Storage, get_DynStorage
from Reader import Reader

from table_description import TABLE_DESCRIPTIONS, TABLE_DESCRIPTIONS2, TABLE_DESCRIPTIONS3

import logging

# Настройка логирования
logging.basicConfig(filename='error.log',
                    level=logging.ERROR,
                    format='%(asctime)s - %(levelname)s - %(message)s')

margin = 5

optimize_methods = [
    ('LM', 'Levenberg-Marquardt'),
    ('SQP', 'SQP'),
    ('TRF', 'trf'),
    ('TC', 'trust-constr'),
    ('DB', 'DogBox'),
]

grey_power = 243
color_power = 230
white = QColor(255, 255, 255)
light_grey = QColor(grey_power, grey_power, grey_power)
red = QColor(255, color_power, color_power)
green = QColor(color_power, 255, color_power)
yellow = QColor(255, 255, color_power)
turquoise = QColor(color_power, 255, 255)
purple = QColor(250, color_power, 255)

data_cols = ['svId', 'gnssId', 'xyz_stamp', 'pr_stamp', 'X', 'Y', 'Z', 'lat', 'lon', 'alt', 'azim', 'polar', 'radius',
             'prMes', 'prRes', 'prRMSer', 'prStedv', 'coord_score', 'nav_score', 'used', 'real_rho', 'Dt', 'af_dt']


def process_columns(column_list, title):
    def check_column(column):
        if title == "Навигационные данные":
            match column:
                case 'receiving_stamp':
                    return column + ', s'
                case 'cno':
                    return column + ', dBHz'
                case 'azim' | 'elev':
                    return column + ', °'
                case 'prMes' | 'prRes' | 'prRMSer' | 'prStedv':
                    return column + ', m'
                case 'ephUsability':
                    return column + ', hours'
                case 'almUsability':
                    return column + ', days'
                case _:
                    return column
        elif title == "Параметры эфемерид" or title == "Параметры альманах":
            match column:
                case 'Toe' | 'Toc' | 'Toa' | 'Tgd':
                    return column + ', s'
                case 'af0':
                    return column + ', s'
                case 'af1':
                    return column + ', s/s'
                case 'af2':
                    return column + ', s/s^2'
                case 'sqrtA':
                    return column + ', m^0.5'
                case 'Crs' | 'Crc':
                    return column + ', m'
                case 'M0' | 'W0' | 'i0' | 'w' | 'delta_i' | 'Cuc' | 'Cus' | 'Cic' | 'Cis':
                    return column + ', rad'
                case 'dn' | 'Wdot' | 'IDOT':
                    return column + ', rad/s'
                case _:
                    return column
        elif title == "Данные эфемерид" or title == 'Данные альманах':
            match column:
                case 'xyz_stamp' | 'pr_stamp':
                    return column + ', s'
                case 'X' | 'Y' | 'Z' | 'real_rho' | 'prMes' | 'prRes' | 'prRMSer' | 'radius' | 'prStedv':
                    return column + ', m'
                case 'alt':
                    return column + ', km'
                case 'lat' | 'lon':
                    return column + ', °'
                case 'polar' | 'azim':
                    return column + ', °'
                # case 'lon': return 'azim, °'
                case 'Dt':
                    return column + ', ms'
        elif title == 'Результаты по эфемеридам' or title == 'Результаты по альманах':
            match column:
                case 'X' | 'Y' | 'Z' | 'cdt' | 'error' | 'alt':
                    return column + ', m'
                case 'lat' | 'lon':
                    return column + ', °'
                case 'dt' | 'calc_time':
                    return column + ', ms'
                case 'fval':
                    return column + ', m'
                case 'calc_stamp':
                    return column + ', s'
        elif title == 'Общие данные':
            match column:
                case 'ecefX' | 'ecefY' | 'ecefZ' | 'pAcc':
                    return column + ', m'
                case 'receiving_stamp' | 'leapS' | 'tau_c' | 'tau_GPS' | 'B1' | 'rcvTow':
                    return column + ', s'
                case 'B2':
                    return column + ', s/asd'
                case 'fTOW' | 'tAcc':
                    return column + ', ns'
                case 'iTOW':
                    return column + ', us'
        elif 'Отфильтровано' in title:
            match column:
                case 'receiving_stamp': return column + ', s'
                case 'P_rec' | 'P_eph' | 'P_alm': return 'diag ' + column + ', m'
                case 'lat_rec' | 'lat_eph' | 'lat_alm' | 'lon_rec' | 'lon_eph' | 'lon_alm': return column + ', °'
                case _: return column + ', m'
        return column

    # try:
    cols = [check_column(col) for col in column_list]
    return cols
    # except Exception as e:
    #     print(e)
    #     a=0
    # return columns


def get_color(val, N1, N2, reverse=False):
    if val <= N1:
        return red if reverse else green
    if val <= N2:
        return yellow
    return green if reverse else red


def get_QTableWidgetItem(data, column, table_title, base_color):
    try:
        if np.isnan(data) or data == 'nan':
            item = QTableWidgetItem(str('—'))
            item.setBackground(base_color)
            item.setTextAlignment(Qt.AlignCenter)
            return item
    except Exception as e:
        pass
    color = base_color
    if column in ['svId', 'gnssId', 'receiving_stamp', 'xyz_stamp', 'pr_stamp', 'calc_stamp']:
        match column:
            case 'svId':
                data = int(data)
            case 'gnssId':
                data = data.name
            case 'receiving_stamp' | 'xyz_stamp' | 'pr_stamp' | 'calc_stamp':
                data = data.TOW
    elif table_title == "Навигационные данные":
        match column:
            case 'cno':
                data = int(data)
                color = purple if data == 0 else get_color(data, 20, 40, reverse=True)
            case 'ephUsability' | 'almUsability':
                data = int(data)
                data, color = ('-', red) if data == 0 else (('?', turquoise) if data == 31 else (data, green))
                if isinstance(data, int) and column == 'ephUsability':
                    data  = f'{data*0.25:.2f}'
            case 'ephSource' | 'almSource':
                data, color = ('GNSS', green) if data == 1 else (('-', base_color) if data == 0 else ('other', red))
            case 'prMes':
                data = f'{data:.2f}'
            case 'prRes':
                data = f'{data:.1f}'
                color = get_color(abs(float(data)), 10, 40)
            case 'prRMSer':
                color = purple if int(data) == 120 else get_color(abs(data), 10, 40)
                data = f'{data:.1f}'
            case 'prStedv':
                color = get_color(data, 5, 25)
                data = f'{data:.2f}'
            case 'qualityInd':
                data = int(data)
                color = get_color(data, 3, 4, reverse=True)
            case 'mpathIndic':
                data = int(data)
                color = get_color(data, 1, 2) if data else turquoise
            case 'orbitSourse':
                match int(data):
                    case 0:
                        data, color = '-', turquoise
                    case 1:
                        data, color = 'E', green
                    case 2:
                        data, color = 'A', green
                    case _:
                        data = '?'
            case 'health':
                data, color = [('?', turquoise), ('+', green), ('-', red)][int(data)]
            case 'ephAvail' | 'almAvail':
                data, color = ('+', green) if data else ('-', red)
            case 'visibility':
                match int(data):
                    case 0:
                        data, color = '?', turquoise
                    case 1:
                        data, color = '-', red
                    case 2:
                        data, color = '±', yellow
                    case 3:
                        data, color = '+', green
            case 'prValid' | 'svUsed' | 'diffCorr' | 'smoothed':
                data, color = ('+', green) if data else ('-', red)
    elif table_title == "Параметры эфемерид" or table_title == "Параметры альманах":
        match column:
            case 'week' | 'Toe' | 'Toc' | 'Toa' | 'IODE1' | 'IODE2' | 'IODC' | 'Data_ID':
                data = int(data)
            case 'exist' | 'is_old':
                data, color = ('+', green) if data else ('-', red)
            case 'health':
                data = int(data)
                color = green if data == 0 else red
            case 'accuracy':
                data = int(data)
                color = green if data == 0 else yellow
            case _:
                if 1e4 > data > 1:
                    data = f'{data:.3f}'
                elif data > 1e-4:
                    data = f'{data:.5f}'
                else:
                    data = f'{data:.5e}'
    elif table_title == "Данные эфемерид" or table_title == 'Данные альманах':
        match column:
            case 'X' | 'Y' | 'Z' | 'real_rho' | 'prMes':
                data = f'{data:.2f}'
            case 'lat' | 'lon' | 'radius':
                data = f'{data:.3f}'
            case 'alt':
                data = f'{data / 1000:.1f}'
            case 'polar' | 'azim':
                data = f'{data:.5f}'
            case 'Dt':
                data = f'{data * 1000:.4f}'
            case 'prStedv':
                color = get_color(data, 5, 25)
                data = f'{data:.2f}'
            case 'used':
                data, color = ('+', green) if data else ('-', red)
            case 'coord_score' | 'nav_score' | 'prRes' | 'prRMSer':
                if column == 'prRes':
                    color = get_color(abs(float(data)), 10, 40)
                elif column == 'prRMSer':
                    color = purple if int(data) == 120 else get_color(abs(data), 5, 30)
                else:
                    if data < 1:
                        color = red
                data = f'{data:.1f}'
    elif table_title == 'Результаты по эфемеридам' or table_title == 'Результаты по альманах':
        match column:
            case 'X' | 'Y' | 'Z' | 'cdt' | 'error' | 'alt':
                data = f'{data:.2f}'
            case 'lat' | 'lon':
                data = f'{data:.5f}'
            case 'dt' | 'calc_time':
                try:
                    data = f'{data * 1000:.5f}'
                except:
                    if 'limit' in data:
                        color = yellow
                    if 'error' in data:
                        color = red
            case 'fval':
                data = f'{data:.2e}'
            case 'success':
                data, color = ('+', green) if data else ('-', red)
            case 'sat_count':
                data = int(data)
                if data < Settings.MinimumMinimizingSatellitesCount:
                    color = yellow
    elif table_title == 'Общие данные':
        match column:
            case 'ecefX' | 'ecefY' | 'ecefZ' | 'pAcc':
                data = f'{data:.2f}'
            case 'leapS' | 'iTOW' | 'fTOW' | 'tAcc' | 'week' | 'N4' | 'NA' | 'KP':
                data = int(data)
            case 'receiving_stamp':
                data = int(data.TOW)
            case 'towValid' | 'weekValid' | 'leapSValid':
                data, color = ('+', green) if data else ('-', red)
    elif 'Отфильтровано' in table_title:
        if 'X_' in column or 'Y_' in column or 'Z_' in column or 'alt_' in column:
            data = f'{data:.2f}'
        if 'lat_' in column or 'lon_' in column:
            data = f'{data:.6f}'
        elif 'normP_' in column:
            data = f'{data:.5e}'
        elif 'P_' in column:
            data = '[' + ', '.join([f'{elem:.3e}' for elem in np.diag(data)]) + ']'


    item = QTableWidgetItem(str(data))
    item.setBackground(color)
    item.setTextAlignment(Qt.AlignCenter)
    return item


class DataReaderThread(QThread):
    data_received = pyqtSignal(object)

    def __init__(self, reader, parent=None):
        super(DataReaderThread, self).__init__(parent)
        self.reader = reader

    def run(self):
        for parsed in self.reader:
            self.data_received.emit(parsed)


class DescriptionTableWindow(QDialog):
    def __init__(self, parent=None):
        super(DescriptionTableWindow, self).__init__(parent)
        self.setWindowTitle("Описание таблицы")

        self.layout = QVBoxLayout(self)

        self.table_widget = QTableWidget(self)
        self.table_widget.setColumnCount(2)
        self.table_widget.setHorizontalHeaderLabels(["Название колонки", "Описание"])

        self.layout.addWidget(self.table_widget)

    def set_table_data(self, columns, descriptions):
        self.table_widget.setRowCount(len(columns))
        for i, (col, desc) in enumerate(zip(columns, descriptions)):
            item1 = QTableWidgetItem(col)
            item2 = QTableWidgetItem(desc)
            if i % 2 == 0:
                item1.setBackground(light_grey)
                item2.setBackground(light_grey)
            self.table_widget.setItem(i, 0, item1)
            self.table_widget.setItem(i, 1, item2)
        self.table_widget.resizeColumnsToContents()
        self.resize_table_to_content()

    def resize_table_to_content(self):
        self.table_widget.resizeColumnsToContents()
        self.table_widget.resizeRowsToContents()
        table_width = self.table_widget.horizontalHeader().length() + self.table_widget.verticalHeader().width()
        table_height = self.table_widget.verticalHeader().length() + self.table_widget.horizontalHeader().height()
        self.resize(table_width + 40, table_height + 40)


class StorageWorker(QObject):
    result_ready = pyqtSignal(object)  # Сигнал для передачи результата в основной поток
    storage: Storage = None

    def __init__(self, APP):
        super().__init__()
        self.APP = APP
        # self.result_ready.connect(self.APP.handle_result_of_storage_update)

    @pyqtSlot(object)
    def update(self, parsed):
        if self.storage:
            result = self.storage.update(parsed)
            if result:
                self.DynStorage = get_DynStorage(self.storage)
                # self.result_ready.emit(DynStorage)

    def set_storage(self, storage: Storage):
        super().__init__()
        self.storage = storage


class App(QMainWindow):
    storage_update_signal = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.description_window = None
        self.DynStorage = None
        self.data_reader_thread = None
        self.setWindowTitle("GPS Data Interface for UBX")

        self.messages = []
        self.max_messages = 2000
        self.max_message_len = 500
        self.max_solves = 200

        self.storage: Storage = None
        self.reader: Reader = None

        self.current_table: pd.DataFrame = None
        self.current_table_name = ""

        self.init_ui()
        # self.main_layout.setEnabled(False)
        self.showMaximized()

        # Создание рабочего класса и потока
        self.worker = StorageWorker(self)
        self.thread_storage = QThread()
        self.worker.moveToThread(self.thread_storage)
        # Подключение сигналов и слотов
        self.storage_update_signal.connect(self.worker.update)
        # self.worker.result_ready.connect(self.handle_result_of_storage_update)

        # Запуск потока
        self.thread_storage.start()

        # self.storage_update_signal.connect(self.run_storage_update)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QVBoxLayout(central_widget)

        menu_font = QFont()
        menu_font.setPointSize(12)

        self.port_entry = QLineEdit()
        self.port_entry.setFont(menu_font)
        self.port_entry.setPlaceholderText("Введите порт")
        self.port_entry.returnPressed.connect(self.connect)
        self.port_entry.setMaximumWidth(500)

        self.connect_button = QPushButton("Подключиться")
        self.connect_button.setFont(menu_font)
        self.connect_button.clicked.connect(self.connect)

        # self.menu_layout.addWidget(self.port_entry)
        # self.menu_layout.addWidget(self.connect_button)

        self.menu_line1 = QHBoxLayout()
        self.buttons1 = {
            "Сообщения": self.show_chat,
            "Навигационные данные": self.show_navigation_data,
            "Общие данные": self.show_general_data,
            "Отфильтровано XYZ": self.show_filtered_xyz,
            "Отфильтровано LLA": self.show_filtered_lla,
        }
        self.menu_line1.addWidget(self.port_entry)
        self.menu_line1.addWidget(self.connect_button)
        for text, command in self.buttons1.items():
            btn = QPushButton(text)
            btn.setFont(menu_font)
            btn.clicked.connect(command)
            self.menu_line1.addWidget(btn)

        self.menu_line2 = QHBoxLayout()
        self.GPStitle = QLabel()
        self.GPStitle.setFont(menu_font)
        self.GPStitle.setText("GPS system:")
        self.menu_line2.addWidget(self.GPStitle)
        self.buttons2 = {
            "Параметры эфемерид": self.show_ephemeris_parameters,
            "Параметры альманах": self.show_almanac_parameters,
            "Данные эфемерид": self.show_ephemeris_data,
            "Данные альманах": self.show_almanac_data,
            "Результаты по эфемеридам": self.show_ephemeris_solves,
            "Результаты по альманах": self.show_almanac_solves,
        }
        for text, command in self.buttons2.items():
            btn = QPushButton(text)
            btn.setFont(menu_font)
            btn.clicked.connect(command)
            self.menu_line2.addWidget(btn)

        self.menu_line3 = QHBoxLayout()
        self.GLOtitle = QLabel()
        self.GLOtitle.setFont(menu_font)
        self.GLOtitle.setText("GLONASS system:")
        self.menu_line3.addWidget(self.GLOtitle)
        self.buttons3 = {
            "GLO Параметры эфемерид": self.show_ephemeris_parameters_glo,
            "GLO Параметры альманах": self.show_almanac_parameters_glo,
        }
        for text, command in self.buttons3.items():
            btn = QPushButton(text)
            btn.setFont(menu_font)
            btn.clicked.connect(command)
            self.menu_line3.addWidget(btn)

        self.menu_layout = QVBoxLayout()
        self.menu_layout.addLayout(self.menu_line1)
        self.menu_layout.addLayout(self.menu_line2)
        self.menu_layout.addLayout(self.menu_line3)

        self.main_layout.addLayout(self.menu_layout)

        chat_font = QFont()
        chat_font.setPointSize(10)
        self.chat_display = QTextEdit()
        self.chat_display.setFont(chat_font)
        self.chat_display.setReadOnly(True)
        self.chat_display.setWordWrapMode(False)  # Отключение автопереноса строк

        self.table_display = QTableWidget()
        self.table_display.setColumnCount(0)
        self.table_display.setRowCount(0)
        self.table_display.setHorizontalScrollMode(QTableWidget.ScrollPerPixel)

        # строка над таблицей

        # Создаем QComboBox
        self.combo_box = QComboBox()
        self.combo_box.setFont(menu_font)
        self.combo_box.setFixedWidth(500)
        self.combo_box.addItems([met for (name, met) in optimize_methods])
        self.combo_box.currentIndexChanged.connect(self.combo_changed)

        # нвзавние таблицы
        self.table_buttons_layout = QHBoxLayout()
        self.table_title = QLabel()
        self.table_title.setFont(menu_font)
        self.table_title.setMaximumWidth(500)

        self.method_title = QLabel('Method: ')
        self.method_title.setFont(menu_font)
        self.method_title.setMaximumWidth(150)

        # описание таблицы
        self.desc_button = QPushButton("Описание таблицы")
        self.desc_button.setFont(menu_font)
        self.desc_button.clicked.connect(self.show_column_descriptions)
        self.desc_button.setMaximumWidth(500)

        self.table_buttons_layout.addWidget(self.table_title)
        self.table_buttons_layout.addStretch()
        self.table_buttons_layout.addWidget(self.method_title)
        self.table_buttons_layout.addWidget(self.combo_box)
        self.table_buttons_layout.addStretch()
        self.table_buttons_layout.addWidget(self.desc_button)

        self.main_layout.addWidget(self.chat_display)
        self.main_layout.addLayout(self.table_buttons_layout)
        self.main_layout.addWidget(self.table_display)

        # main_layout.addWidget(self.chat_display)
        # main_layout.addWidget(self.table_title)
        # main_layout.addWidget(self.table_display)

        self.show_chat()

    def combo_changed(self, index):
        # for name, title in optimize_methods.items():
        #     if title == index:
        # self.storage.method = optimize_methods[index][0]
        # print(f'method: {optimize_methods[index][0]}')
        Settings.used_method = optimize_methods[index][0]
        # return

    def hide_table_buttons_layout(self):
        for i in range(self.table_buttons_layout.count()):
            widget = self.table_buttons_layout.itemAt(i).widget()
            if widget is not None:
                widget.hide()

    def show_table_buttons_layout(self):
        for i in range(self.table_buttons_layout.count()):
            widget = self.table_buttons_layout.itemAt(i).widget()
            if widget is not None:
                widget.show()

    def show_column_descriptions(self):
        if self.current_table is not None:
            match self.current_table_name:
                case "Навигационные данные" | "Параметры эфемерид" | "Параметры альманах" | "Данные эфемерид" | \
                     "Данные альманах":
                    descr_source = TABLE_DESCRIPTIONS
                case "Общие данные":
                    descr_source = TABLE_DESCRIPTIONS3
                case _:
                    descr_source = TABLE_DESCRIPTIONS2
            descriptions = [descr_source.get(col.split(',')[0], "Описание недоступно")
                            for col in self.current_table.columns]
            # desc_text = "<br>".join(f"<b>{col}</b>: {desc}" for col, desc in zip(self.current_table.columns,
            #                                                                      descriptions))
            # QMessageBox.information(self, f"Описание таблицы \"{self.current_table_name}\"", desc_text)
            self.description_window = DescriptionTableWindow(self)
            self.description_window.set_table_data(self.current_table.columns, descriptions)
            self.description_window.exec_()

    @pyqtSlot()
    def connect(self):
        port = self.port_entry.text()
        try:
            self.main_layout.setEnabled(False)
            self.reader = Reader(port)
            self.storage = Storage()

            self.worker.set_storage(self.storage)

            # self.storage_thread = StorageThread(self.storage)
            # self.storage_thread.storage_updated.connect(self.update_current_table)

            self.data_reader_thread = DataReaderThread(self.reader)
            self.data_reader_thread.data_received.connect(self.process_data)

            self.data_reader_thread.start()
            # self.storage_thread.start()

            self.connect_button.setEnabled(False)
            self.port_entry.setEnabled(False)
            self.main_layout.setEnabled(True)
        except Exception as e:
            QMessageBox.warning(self, "Ошибка подключения",
                                f"Не удалось подключиться к {port}: {e}" + traceback.format_exc())
            logging.error(f"Failed to connect to port {port}: {e}", exc_info=True)

    def format_message(self, message):
        # Создаем формат для текста
        format = QTextCharFormat()
        format.setFontWeight(QFont.Bold)
        format.setForeground(QBrush(QColor("blue")))
        if isinstance(message, UBXMessages.UbxMessage):
            head, plb = message.format_message(max_len=self.max_message_len)
            self.chat_display.setCurrentCharFormat(format)
            self.chat_display.insertPlainText(head)
        else:
            plb = str(message)
            if len(plb) > self.max_message_len:
                plb = plb[:self.max_message_len]
        self.chat_display.setCurrentCharFormat(QTextCharFormat())  # Сброс форматирования
        self.chat_display.insertPlainText(plb + '\n')
        if message == None:
            a = 0

    def on_button_click(self, command, button):
        for btn in self.menu_layout.findChildren(QPushButton):
            btn.setEnabled(True)
        button.setEnabled(False)
        command()

    @pyqtSlot(object)
    def process_data(self, parsed):
        if len(self.messages) > self.max_messages:
            self.messages = []  # self.messages[-self.max_messages:]  # Сохраняем последние max_messages сообщений
            self.chat_display.clear()  # Очистка чата
            # for msg in self.messages:
            # # #     self.chat_display.append(msg)
            # self.format_message(msg)

        scrolled_to_bottom = self.is_scrolled_to_bottom(self.chat_display)
        # message_line = str(parsed)
        # self.chat_display.append(message_line[:min(self.max_message_len, len(message_line))])
        self.messages.append(parsed)
        self.format_message(parsed)

        if scrolled_to_bottom:
            self.chat_display.verticalScrollBar().setValue(self.chat_display.verticalScrollBar().maximum())

        self.storage_update_signal.emit(parsed)

        if self.storage.flush_flag:
            self.DynStorage = self.storage.DynStorage
            self.storage.flush_flag = False
            self.update_current_table()

        # if .update(parsed):
        #     if self.current_table is not None:
        #         self.update_current_table()
        # pass

    def handle_result_of_storage_update(self, result):
        # self.DynStorage = result
        # if self.current_table is not None:
        #     self.update_current_table()
        pass

    def show_chat(self):
        self.chat_display.show()
        self.table_display.hide()
        self.hide_table_buttons_layout()
        self.current_table = None
        self.current_table_name = ""

    def show_table(self, table, title):
        # try:
        self.chat_display.hide()
        self.table_display.show()
        self.table_title.setText(title)
        self.show_table_buttons_layout()
        self.current_table = table
        self.current_table_name = title
        self.update_table_display(table)
        if title in ["Результаты по эфемеридам", "Результаты по альманах"]:
            self.table_display.scrollToBottom()
            self.combo_box.show()
            self.method_title.show()
        else:
            self.combo_box.hide()
            self.method_title.hide()
        # except Exception as e:
        #     print(e)
        #     a = 0

    def update_table_display(self, table):
        if table is not None:
            scrolled_to_bottom = self.is_scrolled_to_bottom(self.table_display)

            self.table_display.setColumnCount(len(table.columns))
            self.table_display.setRowCount(len(table.index))  # + 1)  # +1 for the bottom header row
            try:
                columns = process_columns(table.columns, self.current_table_name)
                self.table_display.setHorizontalHeaderLabels(columns)
            except Exception as e:
                print(e)
                a=0

            for i in range(len(table.index)):
                for j in range(len(table.columns)):
                    item = get_QTableWidgetItem(table.iat[i, j], table.columns[j], self.current_table_name,
                                                light_grey if i % 2 == 0 else white)
                    self.table_display.setItem(i, j, item)

            # Add bottom header row
            # bottom_row = self.table_display.rowCount() - 1
            # for j, column_name in enumerate(table.columns):
            #     self.table_display.setItem(bottom_row, j, QTableWidgetItem(column_name))
            #     self.table_display.item(bottom_row, j).setBackground(light_grey)
            #     self.table_display.item(bottom_row, j).setFlags(Qt.ItemIsEnabled)

            # Set tooltips for horizontal header items
            # header = self.table_display.horizontalHeader()
            # for j, column_name in enumerate(table.columns):
            #     description = TABLE_DESCRIPTIONS.get(column_name, "")
            #     header.setToolTip(description)

            self.correct_table_width()

            # self.table_header = self.table_display.horizontalHeader()
            # self.table_header.sectionEntered.connect(self.show_tooltip)

            # header = self.table_display.horizontalHeader()
            # header.setSectionResizeMode(QHeaderView.Stretch)
            # header.setSectionResizeMode(QHeaderView.Interactive)

            if scrolled_to_bottom:
                self.table_display.verticalScrollBar().setValue(self.table_display.verticalScrollBar().maximum())

    # def show_tooltip(self, logicalIndex):
    #     column_name = self.table_display.horizontalHeaderItem(logicalIndex).text()
    #     if self.current_table_name in ["Навигационные данные", "Параметры эфемерид", "Параметры альманах",
    #                                    "Данные эфемерид", "Данные альманах"]:
    #         description = TABLE_DESCRIPTIONS.get(column_name, "Описание недоступно")
    #     else:
    #         description = TABLE_DESCRIPTIONS2.get(column_name, "Описание недоступно")
    #     QToolTip.showText(QCursor.pos(), description, self.table_display)
    #     print('TOOLTIP')

    def correct_table_width(self):
        self.table_display.resizeColumnsToContents()
        for column in range(self.table_display.columnCount()):
            header = self.table_display.horizontalHeaderItem(column).text()
            current_width = self.table_display.columnWidth(column)
            if len(header) < 10:
                self.table_display.setColumnWidth(column, current_width + margin)

        # Если общая ширина столбцов меньше ширины таблицы, растягиваем столбцы
        total_column_width = sum(
            self.table_display.columnWidth(column) for column in range(self.table_display.columnCount()))
        table_width = self.table_display.viewport().width()

        # Учитываем ширину вертикальной полосы прокрутки, если она есть
        if self.table_display.verticalScrollBar().isVisible():
            table_width -= self.table_display.verticalScrollBar().width()

        # Если общая ширина столбцов меньше ширины таблицы, растягиваем столбцы
        if total_column_width < table_width:
            extra_space = table_width - total_column_width - 50
            additional_width_per_column = extra_space // self.table_display.columnCount()

            for column in range(self.table_display.columnCount()):
                new_width = self.table_display.columnWidth(column) + additional_width_per_column
                self.table_display.setColumnWidth(column, new_width)

    def show_navigation_data(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.navigation_parameters1, "Навигационные данные")

    def show_general_data(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.general_data1, "Общие данные")

    def show_filtered_xyz(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.FK_coordinates_xyz, "Отфильтровано XYZ")

    def show_filtered_lla(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.FK_coordinates_lla, "Отфильтровано LLA")

    def show_ephemeris_parameters(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.ephemeris_parameters1, "Параметры эфемерид")

    def show_almanac_parameters(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.almanac_parameters1, "Параметры альманах")

    def show_ephemeris_parameters_glo(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.SFRBX_GLONASS_eph, "GLO Параметры эфемерид")
            print('GLO eph')

    def show_almanac_parameters_glo(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.SFRBX_GLONASS_alm, "GLO Параметры альманах")
            print('GLO alm')

    def show_ephemeris_data(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.ephemeris_data1[data_cols], "Данные эфемерид")

    def show_ephemeris_solves(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.ephemeris_solves1, "Результаты по эфемеридам")

    def show_almanac_data(self):
        self.show_table(self.DynStorage.almanac_data1[data_cols], "Данные альманах")

    def show_almanac_solves(self):
        if self.DynStorage:
            self.show_table(self.DynStorage.almanac_solves1, "Результаты по альманах")

    def update_current_table(self):
        table_map = {
            "Навигационные данные": self.DynStorage.navigation_parameters1,
            "Общие данные": self.DynStorage.general_data1,
            "Отфильтровано XYZ": self.DynStorage.FK_coordinates_xyz,
            "Отфильтровано LLA": self.DynStorage.FK_coordinates_lla,
            "Параметры эфемерид": self.DynStorage.ephemeris_parameters1,
            "Параметры альманах": self.DynStorage.almanac_parameters1,
            "Данные эфемерид": self.DynStorage.ephemeris_data1[data_cols],
            "Результаты по эфемеридам": self.DynStorage.ephemeris_solves1,
            "Данные альманах": self.DynStorage.almanac_data1[data_cols],
            "Результаты по альманах": self.DynStorage.almanac_solves1,
            "GLO Параметры эфемерид": self.DynStorage.SFRBX_GLONASS_eph,
            "GLO Параметры альманах": self.DynStorage.SFRBX_GLONASS_alm,
        }

        if self.current_table_name in table_map:
            table = table_map[self.current_table_name]
            if self.current_table_name in \
                    ["Результаты по эфемеридам", "Результаты по альманах", "Общие данные", "Отфильтровано XYZ", "Отфильтровано LLA"]:
                print(self.current_table_name)
                self.add_new_rows(table)
            else:
                self.update_table_cells(table)
                # for row in range(self.table_display.rowCount()):
                #     self.table_display.setRowHeight(row, 26)

    def add_new_rows(self, table):
        scrolled_to_bottom = self.is_scrolled_to_bottom(self.table_display)

        current_row_count = self.table_display.rowCount()  # - 1  # -1 for the bottom header row
        new_row_count = len(table.index)

        for i in range(current_row_count, new_row_count):
            self.table_display.insertRow(i)
            # self.table_display.setRowHeight(i, 26)
            for j in range(len(table.columns)):
                item = get_QTableWidgetItem(table.iat[i, j], table.columns[j], self.current_table_name,
                                            light_grey if i % 2 == 0 else white)
                self.table_display.setItem(i, j, item)

        # Update bottom header row
        # bottom_row = self.table_display.rowCount() - 1
        # for j, column_name in enumerate(table.columns):
        #     self.table_display.setItem(bottom_row, j, QTableWidgetItem(column_name))
        #     self.table_display.item(bottom_row, j).setBackground(light_grey)
        #     self.table_display.item(bottom_row, j).setFlags(Qt.ItemIsEnabled)

        self.correct_table_width()

        # header = self.table_display.horizontalHeader()
        # header.setSectionResizeMode(QHeaderView.Stretch)
        # header.setSectionResizeMode(QHeaderView.Interactive)

        for row in range(self.table_display.rowCount()):
            self.table_display.setRowHeight(row, 20)

        if scrolled_to_bottom:
            self.table_display.scrollToBottom()
            # self.table_display.verticalScrollBar().setValue(self.table_display.verticalScrollBar().maximum() + 1)

    def update_table_cells(self, table):
        scrolled_to_bottom = self.is_scrolled_to_bottom(self.table_display)
        self.table_display.clearContents()

        self.table_display.setRowCount(len(table.index))  # + 1)  # +1 for the bottom header row
        self.table_display.setColumnCount(len(table.columns))
        self.table_display.setHorizontalHeaderLabels(process_columns(table.columns, self.current_table_name))

        for i in range(len(table.index)):
            self.table_display.setRowHeight(i, 20)
            for j in range(len(table.columns)):
                item = get_QTableWidgetItem(table.iat[i, j], table.columns[j], self.current_table_name,
                                            light_grey if i % 2 == 0 else white)
                self.table_display.setItem(i, j, item)

        # Add bottom header row
        # bottom_row = self.table_display.rowCount() - 1
        # for j, column_name in enumerate(table.columns):
        #     self.table_display.setItem(bottom_row, j, QTableWidgetItem(column_name))
        #     self.table_display.item(bottom_row, j).setBackground(light_grey)
        #     self.table_display.item(bottom_row, j).setFlags(Qt.ItemIsEnabled)

        # for row in range(self.table_display.rowCount()):
        #     self.table_display.setRowHeight(row, 26)

        if scrolled_to_bottom:
            self.table_display.scrollToBottom()
            # self.table_display.verticalScrollBar().setValue(self.table_display.verticalScrollBar().maximum())

    def is_scrolled_to_bottom(self, widget):
        return widget.verticalScrollBar().value() == widget.verticalScrollBar().maximum()

    def get_column_widths(self):
        column_widths = []
        for column in range(self.table_display.columnCount()):
            column_widths.append(self.table_display.columnWidth(column))
        return column_widths

    def set_column_widths(self, widths):
        for column, width in enumerate(widths):
            self.table_display.setColumnWidth(column, width)


if __name__ == "__main__":
    try:
        Settings.GUI_ON = True
        app = QApplication(sys.argv)
        window = App()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(e)
        print(traceback.format_exc())
        logging.error("Unhandled exception occurred", exc_info=True)


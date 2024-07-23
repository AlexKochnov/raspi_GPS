import sys
import traceback

from PyQt5.QtGui import QFont, QTextCharFormat, QBrush, QColor
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit, \
    QLineEdit, QLabel, QMessageBox, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, Qt
import pandas as pd

import UBXMessages
# мои собственные готовые модули, которые имортируются и не зменяются
from Storage import Storage
from Reader import Reader

from table_description import TABLE_DESCRIPTIONS

import logging

# Настройка логирования
logging.basicConfig(filename='error.log',
                    level=logging.ERROR,
                    format='%(asctime)s - %(levelname)s - %(message)s')


class DataReaderThread(QThread):
    data_received = pyqtSignal(object)

    def __init__(self, reader, parent=None):
        super(DataReaderThread, self).__init__(parent)
        self.reader = reader

    def run(self):
        for parsed in self.reader:
            self.data_received.emit(parsed)


class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.data_reader_thread = None
        self.setWindowTitle("GPS Data Interface for UBX")

        self.messages = []
        self.max_messages = 1000
        self.max_message_len = 600
        self.max_solves = 200

        self.storage: Storage = None
        self.reader: Reader = None

        self.current_table: pd.DataFrame = None
        self.current_table_name = ""

        self.init_ui()
        self.showMaximized()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        menu_font = QFont()
        menu_font.setPointSize(12)

        self.menu_layout = QHBoxLayout()
        self.port_entry = QLineEdit()
        self.port_entry.setFont(menu_font)
        self.port_entry.setPlaceholderText("Введите порт")
        self.port_entry.returnPressed.connect(self.connect)

        self.connect_button = QPushButton("Подключиться")
        self.connect_button.setFont(menu_font)
        self.connect_button.clicked.connect(self.connect)

        self.menu_layout.addWidget(self.port_entry)
        self.menu_layout.addWidget(self.connect_button)

        self.buttons = {
            "Сообщения": self.show_chat,
            "Навигационные данные": self.show_navigation_data,
            "Параметры эфемерид": self.show_ephemeris_parameters,
            "Параметры альманах": self.show_almanac_parameters,
            "Данные эфемерид": self.show_ephemeris_data,
            "Результаты по эфемеридам": self.show_ephemeris_solves,
            "Данные альманах": self.show_almanac_data,
            "Результаты по альманах": self.show_almanac_solves
        }

        for text, command in self.buttons.items():
            btn = QPushButton(text)
            btn.setFont(menu_font)
            btn.clicked.connect(command)
            # btn.clicked.connect(lambda command: self.on_button_click(command, btn))
            # btn.clicked.connect(lambda checked, b=btn: self.on_button_click(b))
            self.menu_layout.addWidget(btn)

        main_layout.addLayout(self.menu_layout)

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

        self.table_title = QLabel()
        self.table_title.setFont(menu_font)

        main_layout.addWidget(self.chat_display)
        main_layout.addWidget(self.table_title)
        main_layout.addWidget(self.table_display)

        self.show_chat()

    @pyqtSlot()
    def connect(self):
        port = self.port_entry.text()
        try:
            self.reader = Reader(port)
            self.storage = Storage()
            self.data_reader_thread = DataReaderThread(self.reader)
            self.data_reader_thread.data_received.connect(self.process_data)
            self.data_reader_thread.start()
            self.connect_button.setEnabled(False)
            self.port_entry.setEnabled(False)
        except Exception as e:
            QMessageBox.warning(self, "Ошибка подключения", f"Не удалось подключиться к {port}: {e}" + traceback.format_exc())
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
            a=0

    def on_button_click(self, command, button):
        for btn in self.menu_layout.findChildren(QPushButton):
            btn.setEnabled(True)
        button.setEnabled(False)
        command()


    @pyqtSlot(object)
    def process_data(self, parsed):
        if len(self.messages) > self.max_messages:
            self.messages = [] #self.messages[-self.max_messages:]  # Сохраняем последние max_messages сообщений
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

        if self.storage.update(parsed):
            if self.current_table is not None:
                self.update_current_table()
                # pass

    def show_chat(self):
        self.chat_display.show()
        self.table_display.hide()
        self.table_title.hide()
        self.current_table = None
        self.current_table_name = ""

    def show_table(self, table, title):
        self.chat_display.hide()
        self.table_display.show()
        self.table_title.setText(title)
        self.table_title.show()
        self.update_table_display(table)
        if title in ["Результаты по эфемеридам", "Результаты по альманах"]:
            self.table_display.scrollToBottom()

        self.current_table = table
        self.current_table_name = title
        a=0

    def update_table_display(self, table):
        if table is not None:
            scrolled_to_bottom = self.is_scrolled_to_bottom(self.table_display)

            self.table_display.setColumnCount(len(table.columns))
            self.table_display.setRowCount(len(table.index) + 1)  # +1 for the bottom header row
            self.table_display.setHorizontalHeaderLabels(table.columns)

            for i in range(len(table.index)):
                for j in range(len(table.columns)):
                    item = QTableWidgetItem(str(table.iat[i, j]))
                    if i % 2 == 0:
                        item.setBackground(Qt.lightGray)
                    self.table_display.setItem(i, j, item)


            # Add bottom header row
            bottom_row = self.table_display.rowCount() - 1
            for j, column_name in enumerate(table.columns):
                self.table_display.setItem(bottom_row, j, QTableWidgetItem(column_name))
                self.table_display.item(bottom_row, j).setBackground(Qt.lightGray)
                self.table_display.item(bottom_row, j).setFlags(Qt.ItemIsEnabled)

            # Set tooltips for horizontal header items
            # header = self.table_display.horizontalHeader()
            # for j, column_name in enumerate(table.columns):
            #     description = TABLE_DESCRIPTIONS.get(column_name, "")
            #     header.setToolTip(description)

            self.table_display.resizeColumnsToContents()
            # header = self.table_display.horizontalHeader()
            # header.setSectionResizeMode(QHeaderView.Stretch)
            # header.setSectionResizeMode(QHeaderView.Interactive)

            if scrolled_to_bottom:
                self.table_display.verticalScrollBar().setValue(self.table_display.verticalScrollBar().maximum())

    def show_navigation_data(self):
        self.show_table(self.storage.navigation_parameters1, "Навигационные данные")

    def show_ephemeris_parameters(self):
        self.show_table(self.storage.ephemeris_parameters1, "Параметры эфемерид")

    def show_almanac_parameters(self):
        self.show_table(self.storage.almanac_parameters1, "Параметры альманах")

    def show_ephemeris_data(self):
        self.show_table(self.storage.ephemeris_data1, "Данные эфемерид")

    def show_ephemeris_solves(self):
        self.show_table(self.storage.ephemeris_solves1, "Результаты по эфемеридам")

    def show_almanac_data(self):
        self.show_table(self.storage.almanac_data1, "Данные альманах")

    def show_almanac_solves(self):
        self.show_table(self.storage.almanac_solves1, "Результаты по альманах")

    def update_current_table(self):
        table_map = {
            "Навигационные данные": self.storage.navigation_parameters1,
            "Параметры эфемерид": self.storage.ephemeris_parameters1,
            "Параметры альманах": self.storage.almanac_parameters1,
            "Данные эфемерид": self.storage.ephemeris_data1,
            "Результаты по эфемеридам": self.storage.ephemeris_solves1,
            "Данные альманах": self.storage.almanac_data1,
            "Результаты по альманах": self.storage.almanac_solves1,
        }
        if self.current_table_name in table_map:
            table = table_map[self.current_table_name]
            if self.current_table_name in ["Результаты по эфемеридам", "Результаты по альманах"]:
                self.add_new_rows(table)
            else:
                self.update_table_cells(table)
                for row in range(self.table_display.rowCount()):
                    self.table_display.setRowHeight(row, 26)

    def add_new_rows(self, table):
        scrolled_to_bottom = self.is_scrolled_to_bottom(self.table_display)

        current_row_count = self.table_display.rowCount() - 1  # -1 for the bottom header row
        new_row_count = len(table.index)

        for i in range(current_row_count, new_row_count):
            self.table_display.insertRow(i)
            for j in range(len(table.columns)):
                item = QTableWidgetItem(str(table.iat[i, j]))
                if i % 2 == 0:
                    item.setBackground(Qt.lightGray)
                self.table_display.setItem(i, j, item)

        # Update bottom header row
        bottom_row = self.table_display.rowCount() - 1
        for j, column_name in enumerate(table.columns):
            self.table_display.setItem(bottom_row, j, QTableWidgetItem(column_name))
            self.table_display.item(bottom_row, j).setBackground(Qt.lightGray)
            self.table_display.item(bottom_row, j).setFlags(Qt.ItemIsEnabled)


        self.table_display.resizeColumnsToContents()
        # header = self.table_display.horizontalHeader()
        # header.setSectionResizeMode(QHeaderView.Stretch)
        # header.setSectionResizeMode(QHeaderView.Interactive)

        if scrolled_to_bottom:
            self.table_display.scrollToBottom()
            # self.table_display.verticalScrollBar().setValue(self.table_display.verticalScrollBar().maximum() + 1)

    def update_table_cells(self, table):
        scrolled_to_bottom = self.is_scrolled_to_bottom(self.table_display)
        self.table_display.clearContents()

        self.table_display.setRowCount(len(table.index) + 1)  # +1 for the bottom header row
        self.table_display.setColumnCount(len(table.columns))
        self.table_display.setHorizontalHeaderLabels(table.columns)

        for i in range(len(table.index)):
            for j in range(len(table.columns)):
                item = QTableWidgetItem(str(table.iat[i, j]))
                if i % 2 == 0:
                    item.setBackground(Qt.lightGray)
                self.table_display.setItem(i, j, item)

        # Add bottom header row
        bottom_row = self.table_display.rowCount() - 1
        for j, column_name in enumerate(table.columns):
            self.table_display.setItem(bottom_row, j, QTableWidgetItem(column_name))
            self.table_display.item(bottom_row, j).setBackground(Qt.lightGray)
            self.table_display.item(bottom_row, j).setFlags(Qt.ItemIsEnabled)

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
        app = QApplication(sys.argv)
        window = App()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        logging.error("Unhandled exception occurred", exc_info=True)










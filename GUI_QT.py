import sys
import pandas as pd
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, \
    QTextEdit, QTableView, QHeaderView, QLineEdit, QSizePolicy
from PyQt5.QtGui import QIcon, QFont, QTextCursor
from PyQt5.QtCore import Qt, QSize, QAbstractTableModel, QVariant, pyqtSignal, QObject, QThread
import time

import UBXMessages
from Reader import Reader
from Storage import Storage


# Основное окно приложения
class MainWindow(QMainWindow):
    max_lines = 200

    chat_messages = []

    def __init__(self):
        super().__init__()
        self.initUI()
        self.additional_windows = []
        self.storage = None  # Инициализация переменной storage
        self.additional_windows_threads = []


    def initUI(self):
        self.setWindowTitle('Main Window')

        # Устанавливаем геометрию главного окна
        self.resize(QApplication.primaryScreen().size().width() // 2, QApplication.primaryScreen().size().height() // 2)
        self.move(0, 0)
        # self.move(QApplication.primaryScreen().size().width() // 2, 0)

        # Поле ввода текста
        self.input_text = QLineEdit(self)
        self.input_text.setMaximumWidth(200)
        self.input_text.returnPressed.connect(self.add_text_to_chat)

        # Кнопка для добавления текста в чат
        self.add_button = QPushButton('Add', self)
        self.add_button.setMaximumWidth(80)
        self.add_button.clicked.connect(self.add_text_to_chat)

        # Лейбл для чата
        self.chat_label = QLabel('Chat:', self)

        # Текстовое поле для чата (прокручиваемое)
        self.chat_text = QTextEdit(self)
        self.chat_text.setGeometry(10, 70, 780, 200)
        self.chat_text.setReadOnly(True)
        self.chat_text.setLineWrapMode(QTextEdit.NoWrap)  # Отключаем автоперенос строк
        self.chat_text.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.chat_text.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

        # Начальный текст в чате
        self.add_to_chat('')

        # Кнопки для открытия новых окон
        self.button1 = QPushButton('Данные альманах', self)
        self.button2 = QPushButton('Данных эфемерид', self)
        self.button3 = QPushButton('Общие данные', self)

        # Помещаем поле ввода, кнопку добавления текста и кнопки в одну строку
        top_layout = QHBoxLayout()
        top_layout.addWidget(self.input_text)
        top_layout.addWidget(self.add_button)
        top_layout.addWidget(self.button1)
        top_layout.addWidget(self.button2)
        top_layout.addWidget(self.button3)

        # Создаем основной виджет и устанавливаем layout
        central_widget = QWidget()
        central_layout = QVBoxLayout()
        central_layout.addLayout(top_layout)
        central_layout.addWidget(self.chat_label)
        central_layout.addWidget(self.chat_text)
        central_widget.setLayout(central_layout)

        self.setCentralWidget(central_widget)

        # Привязываем действия к кнопкам
        self.button1.clicked.connect(lambda: self.open_new_window(1))
        self.button2.clicked.connect(lambda: self.open_new_window(2))
        self.button3.clicked.connect(lambda: self.open_new_window(3))

        # Создаем и запускаем поток Worker
        self.worker_thread = QThread()
        self.worker = Worker()
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.run)
        self.worker.data_changed.connect(self.handle_data_changed)
        self.worker.message_received.connect(self.receive_message)  # Подключаем слот

        self.worker_thread.start()

    def open_window1(self):
        self.open_new_window(1)

    def open_window2(self):
        self.open_new_window(2)

    def open_window3(self):
        self.open_new_window(3)

    def open_new_window(self, type):
        if not self.storage:
            return
        # thread = QThread()
        # new_window = AdditionalWindowWorker(type, self.storage)
        # new_window.moveToThread(thread)
        # thread.started.connect(new_window.run)
        # thread.start()
        # self.additional_windows_threads.append(thread)

        new_window = AdditionalWindow(type, self.storage)
        new_window.resize(self.width(), self.height())
        new_window.move(self.x(), self.y() + self.height())
        new_window.show()
        self.additional_windows.append(new_window)



    def add_text_to_chat(self):
        new_text = self.input_text.text()
        if new_text:
            self.add_to_chat(new_text)
            self.input_text.clear()
    @staticmethod
    def msg_to_html(message):
        if not isinstance(message, UBXMessages.UbxMessage) or isinstance(message, str):
            formatted_text = str(message).replace('<', '|').replace('>', '|')
        else:
            formatted_text = str(message)
        return formatted_text

    def add_to_chat(self, messages):
        return
        if not isinstance(messages, list):
            messages = [messages]
        # self.chat_messages += messages
        # if len(self.chat_messages) > self.max_lines:
        #     self.chat_messages = self.chat_messages[:self.max_lines]
        # # formatted_text = '<br>'.join(map(self.msg_to_html, self.chat_messages))
        # text = self.chat_text.toPlainText()
        # if text.count('\n') > self.max_lines:
        #     self.chat_text.clear()
        #     self.chat_text.append(text[text.find('\n', self.max_lines//2):])
        # for message in self.chat_messages:
        self.chat_text.clear()
        for message in self.chat_messages + messages:
            self.chat_text.append(str(message))
        self.chat_messages = messages

        # # Добавляем текст в чат
        # cursor = self.chat_text.textCursor()
        # cursor.movePosition(QTextCursor.End)
        # cursor.insertHtml(formatted_text + '<br>')
        # self.chat_text.setTextCursor(cursor)
        # self.limit_chat_lines()
        # self.scroll_to_bottom()

        # # Очищаем текущий чат и заново отрисовываем сообщения
        # self.chat_text.clear()
        # cursor = self.chat_text.textCursor()
        # cursor.insertHtml(formatted_text)
        # self.chat_text.setTextCursor(cursor)
        # self.scroll_to_bottom()

    def limit_chat_lines(self):
        document = self.chat_text.document()
        block_count = document.blockCount()

        if block_count > self.max_lines:
            cursor = QTextCursor(document)
            cursor.movePosition(QTextCursor.Start)
            cursor.select(QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()  # Удаление блока
            self.chat_text.setTextCursor(cursor)

    def scroll_to_bottom(self):
        # Прокрутка текста вниз
        cursor = self.chat_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.movePosition(QTextCursor.StartOfLine)
        self.chat_text.setTextCursor(cursor)

    def handle_data_changed(self, data):
        self.storage = data

    def receive_message(self, message):
        # Слот для обработки сообщения, полученного из Worker
        self.add_to_chat(message)

    def closeEvent(self, event):
        # При закрытии основного окна останавливаем поток Worker
        exit()
        # self.worker_thread.quit()
        # self.worker_thread.wait()
        # event.accept()


# Класс Worker для выполнения длительной операции в фоновом потоке
class Worker(QObject):
    data_changed = pyqtSignal(Storage)
    message_received = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.initialize("COM3")

    def initialize(self, port):
        self.storage = Storage()
        self.reader = Reader(port=port)

    def run(self):
        for parsed in self.reader:
            messages = self.storage.update(parsed)
            if messages:
                self.message_received.emit(messages)
                self.data_changed.emit(self.storage)


class AdditionalWindowWorker(QObject):
    def __init__(self, type: int, storage: Storage):
        self.type = type
        self.storage = storage
        super().__init__()

    def run(self):
        self.additional_window = AdditionalWindow(self.type, self.storage)
        self.additional_window.show()
        # self.exec_()
        QApplication.exec_()

# Окно для таблицы и текстового поля
class AdditionalWindow(QMainWindow):
    def __init__(self, type: int, storage: Storage):
        super().__init__()
        match type:
            case 1:
                self.title = 'Данные альманах'
                self.df = storage.almanac_parameters
            case 2:
                self.title = 'Данные эфемерид'
                self.df = storage.ephemeris_parameters
            case 3:
                self.title = 'Общие данные'
                self.df = storage.navigation_parameters
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)

        # Устанавливаем геометрию дополнительного окна
        self.resize(QApplication.primaryScreen().size().width() // 2, QApplication.primaryScreen().size().height() // 2)
        self.move(QApplication.primaryScreen().size().width() // 4, QApplication.primaryScreen().size().height() // 4)

        # Создаем QLabel для отображения текста в разных положениях
        self.label_top_left = QLabel('', self)
        self.label_top_center = QLabel('', self)
        self.label_top_right = QLabel('', self)

        # Устанавливаем максимальную высоту для QLabel
        max_height = 100  # Максимальная высота в пикселях
        self.label_top_left.setMaximumHeight(max_height)
        self.label_top_center.setMaximumHeight(max_height)
        self.label_top_right.setMaximumHeight(max_height)

        # Устанавливаем политику изменения размеров для QLabel
        self.label_top_left.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.label_top_center.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.label_top_right.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # Создаем горизонтальный layout для текстовых полей
        hbox = QHBoxLayout()
        hbox.addWidget(self.label_top_left)
        hbox.addWidget(self.label_top_center)
        hbox.addWidget(self.label_top_right)

        # Создаем таблицу на основе DataFrame
        self.table_view = QTableView(self)
        self.model = PandasModel(self.df)
        self.table_view.setModel(self.model)
        self.table_view.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        # Устанавливаем минимальную ширину столбцов
        header = self.table_view.horizontalHeader()
        header.setMinimumSectionSize(100)  # Устанавливаем минимальную ширину в пикселях
        header.setSectionResizeMode(QHeaderView.ResizeToContents)  # Меняем режим изменения размеров столбцов

        # Создаем вертикальный layout и добавляем виджеты
        layout = QVBoxLayout()
        layout.addLayout(hbox)
        layout.addWidget(self.table_view)

        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

    def update_top_left_text(self, new_text):
        self.label_top_left.setText(new_text)


# Класс для модели данных Pandas для QTableView
class PandasModel(QAbstractTableModel):
    def __init__(self, data):
        super().__init__()
        self._data = data

    def data(self, index, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            return str(self._data.iloc[index.row(), index.column()])
        return QVariant()

    def rowCount(self, parent=None):
        return self._data.shape[0]

    def columnCount(self, parent=None):
        return self._data.shape[1]

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return str(self._data.columns[section])
            elif orientation == Qt.Vertical:
                return str(self._data.index[section])
        return QVariant()


# Основная функция для запуска приложения
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
    # try:
    #     main()
    # except Exception as e:
    #     print(e)
    #     a = 0

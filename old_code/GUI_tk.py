import tkinter as tk
from tkinter import ttk, scrolledtext

import numpy as np
import pandas as pd
import threading

from Messages.Reader import Reader
from old_code.Storage import Storage

#TODO: delete (only for opening excel app)

class MainWindow(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Main Window")
        self.geometry(f"{self.winfo_screenwidth() // 2}x{self.winfo_screenheight() // 2}")
        self.storage = None  # Инициализация переменной storage
        self.chat_messages = []
        self.max_chat_size = 200
        self.chat_size = 0

        self.create_widgets()
        self.additional_windows = []

        # Запуск фонового потока
        self.worker_thread = threading.Thread(target=self.worker, daemon=True)
        self.worker_thread.start()

    def create_widgets(self):
        top_frame = tk.Frame(self)
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        self.input_text = tk.Entry(top_frame, width=20)
        self.input_text.pack(side=tk.LEFT, padx=5)
        self.input_text.bind("<Return>", self.add_text_to_chat)

        self.add_button = tk.Button(top_frame, text="Add", command=self.add_text_to_chat)
        self.add_button.pack(side=tk.LEFT, padx=5)

        self.button1 = tk.Button(top_frame, text="Данные альманах", command=lambda: self.open_new_window(1))
        self.button1.pack(side=tk.LEFT, padx=5)
        self.button2 = tk.Button(top_frame, text="Данных эфемерид", command=lambda: self.open_new_window(2))
        self.button2.pack(side=tk.LEFT, padx=5)
        self.button3 = tk.Button(top_frame, text="Общие данные", command=lambda: self.open_new_window(3))
        self.button3.pack(side=tk.LEFT, padx=5)

        self.chat_label = tk.Label(self, text="Chat:")
        self.chat_label.pack(side=tk.TOP, anchor=tk.W, padx=5)

        chat_frame = tk.Frame(self)
        chat_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.chat_text = scrolledtext.ScrolledText(chat_frame, wrap=tk.NONE, height=10)
        self.chat_text.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        chat_scrollbar_x = tk.Scrollbar(chat_frame, orient=tk.HORIZONTAL, command=self.chat_text.xview)
        chat_scrollbar_x.pack(side=tk.BOTTOM, fill=tk.X)
        self.chat_text.config(xscrollcommand=chat_scrollbar_x.set)

    def open_new_window(self, type):
        if not self.storage:
            return
        new_window = AdditionalWindow(type, self.storage)
        new_window.geometry(
            f"{self.winfo_width()}x{self.winfo_height()}+{self.winfo_x()}+{self.winfo_y()}")
        # self.geometry(f"{self.winfo_screenwidth() // 2}x{self.winfo_screenheight() // 2}" +
        #               f"+{self.winfo_screenwidth() // 4}+{self.winfo_screenheight() // 4}")
        self.additional_windows.append(new_window)

    def add_text_to_chat(self, event=None):
        new_text = self.input_text.get()
        if new_text:
            self.add_to_chat(new_text)
            self.input_text.delete(0, tk.END)

    def add_to_chat(self, message):
        # self.chat_messages.append(message)
        self.chat_size += 1
        # if len(self.chat_messages) > self.max_chat_size:
        if self.chat_size > self.max_chat_size:
            self.chat_text.config(state=tk.NORMAL)
            self.chat_text.delete('1.0', '100.0')
            self.chat_size -= 100
            # self.chat_messages = []# self.chat_messages[:self.max_chat_size//2]

            # self.chat_text.config(state=tk.NORMAL)
            # self.chat_text.delete('1.0', tk.END)
            # self.chat_text.config(state=tk.DISABLED)
            #
            # for msg in self.chat_messages:
            #     self.chat_text.config(state=tk.NORMAL)
            #     self.chat_text.insert(tk.END, str(msg) + "\n")
            #     self.chat_text.config(state=tk.DISABLED)
            #     self.chat_text.yview(tk.END)
            # return
        self.chat_text.config(state=tk.NORMAL)
        self.chat_text.insert(tk.END, str(message) + "\n")
        self.chat_text.config(state=tk.DISABLED)
        self.chat_text.yview(tk.END)
    def worker(self):
        self.storage = Storage()
        reader = Reader(port="COM3")
        for parsed in reader:
            self.storage.update(parsed)
            # if messages:
            self.receive_message(parsed)

    def receive_message(self, messages):
        self.add_to_chat(messages)



class AdditionalWindow(tk.Toplevel):
    def __init__(self, type, storage):
        super().__init__()
        self.type = type
        self.storage = storage
        self.title(self.get_title(type))
        self.df = self.get_data(type)
        # self.text = ['', '']
        self.text = self.get_text(type)
        self.create_widgets()


    def get_text(self, type):
        if type == 1:
            d = self.storage.alm_res
            if d:
                return self.check_text(d[0], 'LM'), self.check_text(d[1])
            return ['', '']
        elif type == 2:
            d = self.storage.eph_res
            if d:
                return self.check_text(d[0], 'LM'), self.check_text(d[1])
            return ['', '']
        elif type == 3:
            return ['', '']
        return 'Unknown'

    def get_title(self, type):
        if type == 1:
            return 'Данные альманах'
        elif type == 2:
            return 'Данные эфемерид'
        elif type == 3:
            return 'Общие данные'
        return 'Unknown'

    def get_data(self, type):
        if type == 1:
            table = self.storage.almanac_parameters
            return table
        elif type == 2:
            table = self.storage.ephemeris_parameters
            return table
        elif type == 3:
            table = self.storage.navigation_parameters.copy()
            table.drop(columns=
                       ['NAV_ORB_stamp', 'NAV_SAT_stamp', 'RXM_RAWX_stamp', 'RXM_SVSI_stamp', 'RXM_MEASX_stamp'],
                       inplace=True)
            round_columns = ['nav_score', 'prRes', 'prMes', 'cpMes', 'doMes']
            table[round_columns] = table[round_columns].round(2)

            return table
        return pd.DataFrame()

    @staticmethod
    def check_text(obj, types='SQP'):
        if type(obj) == str: #isinstance(obj, str):
            return obj
        s = (f'{types}:\nx: {np.array(obj.x).round(1)}\nfunc: {obj.fun}\nsuccess:{obj.success}\nError: {obj.ERROR}\n' +
             f'dt{obj.dt}\nlla:{obj.lla}')
        return s

    def create_widgets(self):
        if self.type != 3 and self.text[0] and self.text[1]:
            self.top_frame = tk.Frame(self, height=70)
            self.top_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
            # TODO: добавить решения навигационной задачи
            self.label_top_left = tk.Label(self.top_frame, text=self.text[0], anchor=tk.W)
            self.label_top_left.pack(side=tk.LEFT, fill=tk.X, padx=5, pady=5)

            self.label_top_center = tk.Label(self.top_frame, text=self.text[1], anchor=tk.W)
            self.label_top_center.pack(side=tk.RIGHT, fill=tk.X, padx=5, pady=5)

            # self.label_top_right = tk.Label(self, text="", anchor=tk.W)
            # self.label_top_right.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        table_frame = tk.Frame(self)
        table_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Создаем Treeview и Scrollbar для горизонтальной прокрутки
        self.table = ttk.Treeview(table_frame, show='headings', height=20)
        self.table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        y_scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.table.yview)
        y_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.table.configure(yscrollcommand=y_scrollbar.set)

        x_scrollbar = ttk.Scrollbar(table_frame, orient=tk.HORIZONTAL, command=self.table.xview)
        x_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.table.configure(xscrollcommand=x_scrollbar.set)

        # self.table.bind("<Shift-MouseWheel>", self.on_shift_mouse_wheel)
        self.bind("<Left>", self.scroll_left)
        self.bind("<Right>", self.scroll_right)

        self.update_table()

        # if self.df:
        #     name = f'csv_tables/{datetime.now()}.csv'
        #     self.df.to_csv(name)
            # subprocess.run(['start', '', name], shell=True)

    # def on_shift_mouse_wheel(self, event):
    #     if event.delta:
    #         self.table.xview_scroll(-1 * int(event.delta / 120), "units")

    def scroll_left(self, event):
        self.table.xview_scroll(-100, "units")

    def scroll_right(self, event):
        self.table.xview_scroll(100, "units")

    def update_table(self):
        # Загрузка данных в таблицу
        self.table["columns"] = list(self.df.columns)
        self.table["show"] = "headings"

        # # Определение максимальной ширины для каждого столбца
        # total_width = self.table.winfo_width()  # Ширина виджета таблицы
        # column_widths = {col: max(self.df[col].astype(str).apply(len).max(), len(col)) for col in self.df.columns}
        max_widths = {}
        for col in self.df.columns:
            max_widths[col] = max(self.df[col].astype(str).apply(len).max(), len(col))

        for col in self.df.columns:
            self.table.heading(col, text=col)
            # self.table.column(col, anchor=tk.W, width=int(total_width / len(self.df.columns)))
            self.table.column(col, anchor=tk.W, width=max_widths[col] * 8)  # Множитель для ширины


        rows = self.df.to_numpy().tolist()
        for row in rows:
            self.table.insert("", "end", values=row)


def main():
    app = MainWindow()
    app.mainloop()


if __name__ == "__main__":
    main()

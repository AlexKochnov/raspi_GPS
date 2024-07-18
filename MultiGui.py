import tkinter as tk
from tkinter import messagebox, scrolledtext
from threading import Thread
from queue import Queue
import pandas as pd

from Storage import Storage
from Reader import Reader


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("GPS Data Interface")

        self.messages = []
        self.max_messages = 500
        self.max_solves = 40

        self.storage = None
        self.reader = None
        self.queue = Queue()

        self.current_table: pd.DataFrame = None
        self.current_table_name = ""

        self.create_widgets()

    def create_widgets(self):
        # Создание меню
        self.menu_frame = tk.Frame(self.root)
        self.menu_frame.pack(side=tk.TOP, fill=tk.X)

        self.port_entry = tk.Entry(self.menu_frame, width=15)
        self.port_entry.pack(side=tk.LEFT, padx=5)
        self.port_entry.bind('<Return>', lambda event: self.connect())

        self.connect_button = tk.Button(self.menu_frame, text="Подключиться", command=self.connect)
        self.connect_button.pack(side=tk.LEFT, padx=5)

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
            btn = tk.Button(self.menu_frame, text=text, command=command)
            btn.pack(side=tk.LEFT, padx=5)

        self.chat_frame = tk.Frame(self.root)
        self.chat_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.chat_display = scrolledtext.ScrolledText(self.chat_frame, wrap=tk.NONE, height=15)
        self.chat_display.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.table_frame = tk.Frame(self.root)
        self.table_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        self.table_frame.pack_forget()

    def connect(self):
        port = self.port_entry.get()
        try:
            self.reader = Reader(port)
            self.storage = Storage()
            self.start_reading_thread()
            self.connect_button.config(state=tk.DISABLED)
            self.port_entry.config(state=tk.DISABLED)
        except Exception as e:
            messagebox.showwarning("Ошибка подключения", f"Не удалось подключиться к {port}: {e}")

    def start_reading_thread(self):
        self.reading_thread = Thread(target=self.read_data)
        self.reading_thread.daemon = True
        self.reading_thread.start()

    def read_data(self):
        for parsed in self.reader:
            self.queue.put(parsed)
            self.root.after(0, self.process_data, parsed)

    def process_data(self, parsed):
        # if len(self.messages) >= self.max_messages:
        #     self.messages = self.messages[self.max_messages//2:]
        #     self.chat_display.delete(1.0, float(f'{self.max_messages//2}.0'))

        self.messages.append(parsed)
        scrolled_to_bottom = self.is_scrolled_to_bottom(self.chat_display)
        self.chat_display.insert(tk.END, str(parsed) + "\n")
        # self.chat_display.yview(tk.END)
        if scrolled_to_bottom:
            self.chat_display.yview(tk.END)

        if self.storage.update(parsed):
            if self.current_table is not None:
                self.update_current_table()

    def show_chat(self):
        self.chat_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        self.table_frame.pack_forget()
        self.current_table = None
        self.current_table_name = ""

    def show_table(self, table, title):
        self.chat_frame.pack_forget()
        self.table_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        for widget in self.table_frame.winfo_children():
            widget.destroy()

        label = tk.Label(self.table_frame, text=title)
        label.pack(side=tk.TOP)

        if table is not None:
            table_display = scrolledtext.ScrolledText(self.table_frame, wrap=tk.NONE, height=20)
            table_display.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            table_display.insert(tk.END, table.to_string(index=False))
            # table_display.yview(tk.END)
            # table_display.xview_moveto(1.0)

            self.table_display = table_display
        self.current_table = table
        self.current_table_name = title

    def show_navigation_data(self):
        table = self.storage.navigation_parameters.copy()
        self.show_table(table, "Навигационные данные")

    def show_ephemeris_parameters(self):
        self.show_table(self.storage.ephemeris_parameters, "Параметры эфемерид")

    def show_almanac_parameters(self):
        self.show_table(self.storage.almanac_parameters, "Параметры альманах")

    def show_ephemeris_data(self):
        self.show_table(self.storage.ephemeris_data, "Данные эфемерид")

    def show_ephemeris_solves(self):
        table = self.storage.ephemeris_solves.tail(self.max_solves)
        self.show_table(table, "Результаты по эфемеридам")

    def show_almanac_data(self):
        self.show_table(self.storage.almanac_data, "Данные альманах")

    def show_almanac_solves(self):
        table = self.storage.almanac_solves.tail(self.max_solves)
        self.show_table(table, "Результаты по альманах")

    def update_current_table(self):
        table_map = {
            "Навигационные данные": self.storage.navigation_parameters,
            "Параметры эфемерид": self.storage.ephemeris_parameters,
            "Параметры альманах": self.storage.almanac_parameters,
            "Данные эфемерид": self.storage.ephemeris_data,
            "Результаты по эфемеридам": self.storage.ephemeris_solves,
            "Данные альманах": self.storage.almanac_data,
            "Результаты по альманах": self.storage.almanac_solves,
        }
        if self.current_table_name in table_map:
            table = table_map[self.current_table_name]
            if self.current_table_name in ["Результаты по эфемеридам", "Результаты по альманах"]:
                # self.show_table(table, self.current_table_name)
                self.add_new_rows(table)
            else:
                self.update_table_cells(table)

    def add_new_rows(self, table):
        table_display = self.table_display
        current_content = table_display.get("1.0", tk.END).strip()
        new_content = table.to_string(index=False).strip()

        if current_content != new_content:
            current_lines = current_content.split('\n')
            new_lines = new_content.split('\n')

            if len(new_lines) > len(current_lines):
                new_rows = '\n'.join(new_lines[len(current_lines):])
                scrolled_to_bottom = self.is_scrolled_to_bottom(table_display)
                table_display.insert(tk.END, '\n' + new_rows)
                if scrolled_to_bottom:
                    table_display.yview(tk.END)

    def update_table_cells(self, table):
        table_display = self.table_display
        current_content = table_display.get("1.0", tk.END)
        new_content = table.to_string(index=False)

        if current_content.strip() != new_content.strip():
            scrolled_to_bottom = self.is_scrolled_to_bottom(table_display)
            table_display.delete("1.0", tk.END)
            table_display.insert(tk.END, new_content)
            # table_display.yview(tk.END)
            # table_display.xview_moveto(1.0)
            if scrolled_to_bottom:
                table_display.yview(tk.END)

    def is_scrolled_to_bottom(self, widget):
        return widget.yview()[1] == 1.0

    # def truncate_table_content(self, table):
    #     if len(table) > self.max_messages:
    #         return table.tail(self.max_messages)
    #     return table

    # def handle_row_limit(self, table_display):
    #     lines = table_display.get("1.0", tk.END).strip().split('\n')
    #     if len(lines) > self.max_messages:
    #         table_display.delete("1.0", f"{self.max_messages//2}.0")


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()

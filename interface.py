from flask import Flask, render_template, jsonify
from threading import Thread
import time

from pyubx2 import UBXMessage

from Messages.Reader import Reader
from Messages.UBXMessages import UbxMessage
from Storage.DataStore import DataStore
from Storage.Filters import LocalKalmanFilter, FederatedKalmanFilter
from Storage.serialize import serialize_entry_to_table
from Utils.GNSS import GNSS, NavDataType, Source

app = Flask(__name__)

# Инициализация объектов
reader = Reader(file='raw_kuzm2.log')
storage = DataStore(GNSS.GPS, GNSS.GLONASS, multi_gnss_task=True)
msgs = []

# Функция для обновления данных в фоне
def update_data():
    for parsed in reader:
        msgs.append(parsed)
        storage.update(parsed)

# Запуск функции обновления данных в отдельном потоке
data_thread = Thread(target=update_data)
data_thread.daemon = True
data_thread.start()

def get_menu_items():
    fk_tables = ['FFK', f'default_default']
    for type in ['EPH', 'ALM']:
        for gnss in ['GPS', 'GLONASS', 'ALL']:
            fk_tables.append('_'.join([type, gnss]))

    menu_items = [
        ('Основное', {
            'Главная': '/',
        }),
        ('Таблицы XYZ', {'receiver' if 'default' in name else name: f'/table/XYZ_{name}' for name in fk_tables}),
        ('Таблицы LLA', {'receiver' if 'default' in name else name: f'/table/LLA_{name}' for name in fk_tables}),
    ]
    return menu_items

@app.context_processor
def inject_menu_items():
    return {'menu_items': get_menu_items()}
@app.route('/')
def main_page():
    return render_template('main.html')

@app.route('/data_msgs')
def data_msgs():
    data = [msg.format_message() if False and hasattr(msg, 'format_message') else str(msg) for msg in msgs[-300::-1]]
    # data = [str(msg) for msg in msgs[-1000::-1]]
    return jsonify(data)

@app.route('/data/<table_name>')
def data_api(table_name):
    if 'XYZ' in table_name:
        ffk = storage.filter_xyz
        format = 'XYZ'
    elif 'LLA' in table_name:
        ffk = storage.filter_lla
        format = 'LLA'
    else:
        ffk = None
        format = ''


    if 'FFK' in table_name:
        table = ffk
    else:
        if 'GPS' in table_name:
            gnss = GNSS.GPS
        elif 'GLONASS' in table_name:
            gnss = GNSS.GLONASS
        elif 'ALL' in table_name:
            gnss = GNSS.ALL
        else:
            gnss = GNSS.default

        if 'EPH' in table_name:
            type = NavDataType.EPH
        elif 'ALM' in table_name:
            type = NavDataType.ALM
        else:
            type = NavDataType.default

        source = Source(gnss, type)

        if ffk is not None:
            table = ffk.filters[source]
        else:
            table = None
    if table is not None:
        data = [serialize_entry_to_table(en, format) for en in table.history[::-1]]
    else:
        data = []
    return jsonify(data)

@app.route('/table/<table_name>')
def table_page(table_name):

    # if 'XYZ' not in table_name or 'LLA' not in table_name:
    #     return "Таблица не найдена", 404
    columns = ['TOW', 'week'] + (['X', 'Y', 'Z'] if 'XYZ' in table_name else ['lat', 'lon', 'alt']) + \
              ['normP', 'GDOP', 'fval', 'scores', 'norm_derivative', 'sat_count'] + \
              (['gnss', 'dataType'] if 'FFK' in table_name else [])
    data_url = f"/data/{table_name}"
    return render_template('table.html', table_name=table_name.capitalize(), data_url=data_url, columns=columns)


if __name__ == '__main__':
    app.run(debug=True)

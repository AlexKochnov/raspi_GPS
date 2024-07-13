import requests
from skyfield.api import Topos, load
from skyfield.sgp4lib import EarthSatellite
from datetime import datetime

# Получение TLE данных для всех GPS спутников
url = 'https://celestrak.com/NORAD/elements/gps-ops.txt'
response = requests.get(url)
tle_data = response.text.split('\n')

# Выбор спутника по номеру
sat_number = 13
lines = []
for i in range(len(tle_data)):
    if 'GPS' in tle_data[i] and f'PRN {sat_number}' in tle_data[i]:
        lines = tle_data[i:i+3]
        break

if not lines:
    print("Спутник не найден")
    exit()

# Создание объекта спутника
satellite = EarthSatellite(lines[1], lines[2], lines[0])

# Задание времени наблюдения
ts = load.timescale()
time = ts.utc(2024, 6, 21, 12, 0, 0)  # Замените на нужное время

# Получение координат спутника
geocentric = satellite.at(time)
subpoint = geocentric.subpoint()

print(f'Широта: {subpoint.latitude.degrees}')
print(f'Долгота: {subpoint.longitude.degrees}')
print(f'Высота: {subpoint.elevation.km} км')


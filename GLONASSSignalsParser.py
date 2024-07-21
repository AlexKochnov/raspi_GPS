from numpy import pi

def parse(msg: bytes):
    StringN, superframeN, frameN, line = cut_line(msg)
    id, update_data = parse_line(line, StringN, frameN)
    return id, update_data


def cut_line(line: bytes):
    words = [int.from_bytes((line[i:i + 4])[::-1]) for i in range(0, len(line), 4)]
    idle = words[0] >> 31
    if idle != 0:  # не корректный заголовок
        return None
    StringN = (words[0] >> 27) & 0xF
    superframeN = (words[3] >> 16) & 0xFFFF
    frameN = (words[3]) & 0xFF
    Data = (((words[0] << 32) + words[1]) << 21) + (words[2] >> 11)
    return StringN, superframeN, frameN, Data


def get_bytes(data, l, r, sign=False):
    k = 1
    if sign is True:
        r -= 1
        k = -1 if ((data >> r) & 0x1) else 1
    return k * ((data >> (l - 1)) & ((1 << (r - l + 1)) - 1))


def parse_line(data, StringN, fN) -> (int, dict):  # номер спутника для альманах или 0 + данные
    parse = lambda l, r, sign=False: get_bytes(data, l, r, sign)
    result = {}

    if StringN in [14, 15] and fN == 5:
        if StringN == 14:
            return 0, {
                'B1': parse(70, 80, True) * 2 ** (-10),  # dUT1 на начало суток
                'B2': parse(60, 69, True) * 2 ** (-16),  # скорость изменения dUT1 сек / средние солнечные сутки (???)
                'KP': parse(58, 59),  # Признак коррекции времени
            }
        else:
            # TODO: понять, от какого спутника конкретно эта метка ln
            ln = 0  # признак неприготности спутника (0 -> хорошо, 1 -> плохо) - наверное
            return 0, {}  # {'ln': 0}
    elif StringN > 5:
        svId_a = (fN - 1) * 5 + ((StringN - 4) // 2)
        if StringN % 2 == 0:
            return svId_a, {
                'n': parse(73, 77),  # должен быть равен svId_a, но в нечетных не передается, поэтому не используется
                'lambda_n': parse(42, 62, True) * 2 ** (-20) * pi,
                'delta_i_n': parse(24, 41, True) * 2 ** (-20) * pi,
                'eps_n': parse(9, 23) * 2 ** (-20),
                'M_n': parse(78, 79),
                'tau_n': parse(63, 72, True) * 2 ** (-18),
                'Cn': parse(80, 80),
            }
        else:
            Hn = parse(10, 14)
            if Hn > 16:
                Hn = Hn - 32  # преобразование слова Hn в номер несущей частоты
            return svId_a, {
                'Hn': Hn,
                't_lambda_n': parse(44, 64) * 2 ** (-5),
                'delta_T_n': parse(22, 43, True) * 2 ** (-9),
                'delta_T_dot_n': parse(15, 21, True) * 2 ** (-14),
                'omega_n': parse(65, 80, True) * 2 ** (-15) * pi, # в радианах
                'ln': parse(9, 9),
            }
    else:
        if StringN == 5:  # общие данные для систем глонасс (???) , но  точно нужны для альманаха
            return 0, {
                'tau_c': parse(38, 69, True) * 2 ** (-31),  # может быть другая длинна
                'tau_GPS': parse(10, 31, True) * 2 ** (-30),
                'N4': parse(32, 36),
                'N': parse(70, 80),
                #'ln': parse(9, 9)  # TODO: понять, от какого спутника это
            }
        else:
            # эфемериды, строки 1-4
            if StringN == 1:
                return 0, {
                    'tk': parse(65, 70) * 3600 + parse(70, 76) * 60 + parse(76, 76) * 30, # sec
                    'x': parse(9, 35, True) * 2 ** (-11),       # km
                    'dx': parse(41, 64, True) * 2 ** (-20),     # km/s
                    'ddx': parse(36, 40, True) * 2 ** (-30),    # km/s^2
                    'P1': parse(77, 78), # признак величины времени между tb TODO: чзх?
                }
            if StringN == 2:
                return 0, {
                    'tb': parse(70, 76) * 60, # sec
                    'y': parse(9, 35, True) * 2 ** (-11),       # km
                    'dy': parse(41, 64, True) * 2 ** (-20),     # km/s
                    'ddy': parse(36, 40, True) * 2 ** (-30),    # km/s^2
                    'Bn': parse(78, 80),
                    'P2': parse(77, 77),
                }
            if StringN == 3:
                return 0, {
                    'gamma': parse(69, 79, True) * 2 ** (-40),
                    'z': parse(9, 35, True) * 2 ** (-11),       # km
                    'dz': parse(41, 64, True) * 2 ** (-20),     # km/s
                    'ddz': parse(36, 40, True) * 2 ** (-30),    # km/s^2
                    'P': parse(66, 67),
                    'P3': parse(80, 80),
                    'ln': parse(65, 65),
                }
            if StringN == 4:
                return 0, {
                    'M': parse(9, 10),
                    'tau': parse(59, 80, True) * 2 ** (-30),
                    'N_T': parse(16, 26),
                    'n': parse(11, 15),
                    'F_T': parse(30, 33), # индекс ошибки псевдодальности
                    'E': parse(49, 53),
                    'P4': parse(34, 34),
                    'dTau': parse(54, 58, True) * 2 ** (-30),
                }
        return result


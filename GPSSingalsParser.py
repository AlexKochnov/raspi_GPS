import traceback

from numpy import pi

GPS_PREAMBLE = 0b10001011


def check_preamble(preamble: int):
    return preamble == GPS_PREAMBLE


def get_bits_gps(data, l, n, sign=False):
    res = ((data >> (300 - (l + n - 1))) & ((1 << n) - 1))
    if sign is True and res >> (n - 1) == 1:
        res -= 2 ** n
    return res


def gps_join_sf(msg: bytes, reverse_mod: str = 'FULL') -> int:
    if reverse_mod.upper() == 'AID':
        aid_reverse = lambda word: [word[2], word[1], word[0], word[3]]
        words = [0, 0] + [(int.from_bytes(aid_reverse(msg[i:i + 4])) >> 2) & 0x3FFFFFFF for i in
                          range(0, len(msg), 4)]
    elif reverse_mod == 'FULL':
        words = [int.from_bytes(msg[i:i + 4][::-1]) & 0x3FFFFFFF for i in range(0, len(msg), 4)]
    # elif reverse_mod == 'PSTM':
    #     arr = [int.from_bytes(msg[i:i + 4]) for i in range(0, len(msg), 4)][::-1]
    else:
        # raise TypeError(reverse_mod)
        return 0
    return sum((word << 30 * (10 - i - 1)) for i, word in enumerate(words))


def get_parse_function(subframe: int):
    return lambda l, n, sign=False: get_bits_gps(subframe, l, n, sign)


def get_parse_function2(subframe: int):
    def parse2(l1, n1, l2, n2, sign=False):
        d1 = get_bits_gps(subframe, l1, n1, False)
        d2 = get_bits_gps(subframe, l2, n2, False)
        res = (d1 << n2) + d2
        if sign is True and res >> (n1 + n2 - 1) == 1:
            res -= 2 ** (n1 + n2)
        return res

    return parse2


def parse_aid(subframe_data: int, aid_subframe_number: int) -> dict:
    return {
        1: parse_sf_1,
        2: parse_sf_2,
        3: parse_sf_3,
        5: parse_sf_alm
    }[aid_subframe_number](subframe_data)


def parse(subframe_data: int) -> dict:
    preamble, TOW_CM, sf_id = get_head_stats(subframe_data)
    additional_dict = {'TOW_CM': TOW_CM}
    if not check_preamble(preamble):
        return {}  # неверная преамбула -> данные некорректные
    if sf_id in [1, 2, 3]:  # данные эфемерид для RXM-SFRBX
        return {
            1: parse_sf_1,
            2: parse_sf_2,
            3: parse_sf_3,
        }[sf_id](subframe_data) | additional_dict
    # subframe 4/5
    data_id = get_bits_gps(subframe_data, 61, 2)
    if data_id == 1:
        page_id = get_bits_gps(subframe_data, 63, 6)
        if sf_id == 5 and page_id != 25 or sf_id == 4 and page_id in [2, 3, 4, 5, 7, 8, 9, 10]:
            return parse_sf_alm(subframe_data) | additional_dict
        else:
            print(f'Unrecognized data: {subframe_data}, data id: {data_id}, page_id: {page_id}, TOW_CM: {TOW_CM}')
            a = 0
        # if data_id == 2:
        #     page_id = get_bits_gps(subframe_data, 63, 6)
        if sf_id == 5 and page_id == 51:  # 25:
            res = parse_health(subframe_data) | additional_dict
            print('NONAME DATA:', res)
            return res
        elif page_id == 56:  # 18:
            res = parse_ion_utc(subframe_data) | additional_dict
            print('NONAME DATA:', res)
            return res
        elif page_id == 63:  # 25:
            res = parse_config_health(subframe_data) | additional_dict
            print('NONAME DATA:', res)
            return res
        elif page_id == 52:  # 13:
            res = parse_nmct(subframe_data) | additional_dict
            print('NONAME DATA:', res)
            return res
    # data_id == 3 - что-то там другое
    return {} | additional_dict


def get_head_stats(data) -> tuple[int, int, int]:
    preamble = data >> 292
    TOW_CM = (data >> 253) & 0x1FFFF
    sf_id = (data >> 248) & 0x7
    return preamble, TOW_CM, sf_id


def parse_sf_1(subframe) -> dict:
    parse = get_parse_function(subframe)
    parse2 = get_parse_function2(subframe)
    return {
        'week': parse(61, 10) + 1024 * 2,
        'accuracy': parse(73, 4),
        # 'CA': parse(71, 2),
        'health': parse(7, 6),
        'IODC': parse2(83, 2, 211, 8),
        'Tgd': parse(197, 8, True) * 2 ** (- 31),
        'Toc': parse(219, 16) * 2 ** 4,
        'af2': parse(241, 8, True) * 2 ** (- 55),
        'af1': parse(249, 16, True) * 2 ** (- 43),
        'af0': parse(271, 22, True) * 2 ** (- 31),
    }


def parse_sf_2(subframe) -> dict:
    parse = get_parse_function(subframe)
    parse2 = get_parse_function2(subframe)
    return {
        'IODE1': parse(61, 8),
        'Crs': parse(69, 16, True) * 2 ** (- 5),
        'dn': parse(91, 16, True) * 2 ** (- 43) * pi,
        'M0': parse2(107, 8, 121, 24, True) * 2 ** (- 31) * pi,
        'Cuc': parse(151, 16, True) * 2 ** (- 29),
        'e': parse2(167, 8, 181, 24) * 2 ** (- 33),
        'Cus': parse(211, 16, True) * 2 ** (- 29),
        'sqrtA': parse2(227, 8, 241, 24) * 2 ** (- 19),
        'Toe': parse(271, 16) * 2 ** 4,
    }


def parse_sf_3(subframe) -> dict:
    parse = get_parse_function(subframe)
    parse2 = get_parse_function2(subframe)
    return {
        'Cic': parse(61, 16, True) * 2 ** (- 29),
        'W0': parse2(77, 8, 91, 24, True) * 2 ** (- 31) * pi,
        'Cis': parse(121, 16, True) * 2 ** (- 29),
        'i0': parse2(137, 8, 151, 24, True) * 2 ** (- 31) * pi,
        'Crc': parse(181, 16, True) * 2 ** (- 5),
        'w': parse2(197, 8, 211, 24, True) * 2 ** (- 31) * pi,
        'Wdot': parse(241, 24, True) * 2 ** (- 43) * pi,
        'IODE2': parse(271, 8),
        'IDOT': parse(279, 14, True) * 2 ** (- 43) * pi,
    }


def parse_sf_alm(subframe) -> dict:
    parse = get_parse_function(subframe)
    parse2 = get_parse_function2(subframe)
    return {
        'Data_ID': parse(61, 2),
        'SV_ID': parse(63, 6),
        'e': parse(69, 16) * 2 ** (-21),
        'Toa': parse(91, 8) * 2 ** 12,
        'delta_i': parse(99, 16, True) * 2 ** (-19) * pi,
        'Wdot': parse(121, 16, True) * 2 ** (-38) * pi,
        'health': parse(137, 8),
        'sqrtA': parse(151, 24) * 2 ** (-11),
        'W0': parse(181, 24, True) * 2 ** (-23) * pi,
        'w': parse(211, 24, True) * 2 ** (-23) * pi,
        'M0': parse(241, 24, True) * 2 ** (-23) * pi,
        'af1': parse(279, 11, True) * 2 ** (-38),
        'af0': parse2(271, 8, 290, 3, True) * 2 ** (-20),
    }


def parse_health(subframe) -> dict:
    parse = get_parse_function(subframe)
    health = {}
    for i in range(6):
        for j in range(4):
            svId = i * 4 + (j + 1)
            ind = 91 + 30 * i + 6 * j
            health[svId] = parse(ind, 6)
    return {
        'name': 'health',
        'Toa': parse(69, 8),
        'week': parse(77, 8),
        'health': health,
    }


def parse_ion_utc(subframe) -> dict:
    parse = get_parse_function(subframe)
    parse2 = get_parse_function2(subframe)
    return {
        'name': 'ion_utc',
        'a0': parse(69, 8, True) * 2 ** (-30),
        'a1': parse(77, 8, True) * 2 ** (-27),
        'a2': parse(91, 8, True) * 2 ** (-24),
        'a3': parse(99, 8, True) * 2 ** (-24),
        'b0': parse(107, 8, True) * 2 ** 11,
        'b1': parse(121, 8, True) * 2 ** 14,
        'b2': parse(129, 8, True) * 2 ** 16,
        'b3': parse(137, 8, True) * 2 ** 16,
        'A1': parse(151, 24, True) * 2 ** (-50),
        'A0': parse2(181, 24, 211, 8, True) * 2 ** (-30),
        'Tot': parse(219, 8) * 2 ** 12,
        'week_t': parse(227, 8),
        'dT_ls': parse(241, 8, True),
        'week_lsf': parse(249, 8),
        'DN': parse(257, 8),
        'dT_lsf': parse(271, 8, True),
    }


def parse_config_health(subframe) -> dict:
    parse = get_parse_function(subframe)
    config = {}
    for j in range(4):
        config[(j + 1)] = parse(69 + 4 * j, 4)  # sv 1-4
        config[(j + 1) + 28] = parse(211 + 4 * j, 4)  # sv 29-32
    for i in range(4):
        for j in range(6):
            svId = 4 + i * 6 + (j + 1)
            ind = 91 + 30 * i + 4 * j
            config[svId] = parse(ind, 4)  # sv 5-29
    health = {}
    health[25] = parse(229, 6)
    for j in range(4):
        health[(j + 1) + 25] = parse(241 + 6 * j, 6)
    for j in range(3):
        health[(j + 1) + 29] = parse(271 + 6 * j, 6)
    return {'name': 'config_health', 'config': config, 'health': health}


def parse_nmct(subframe) -> dict:
    try:
        parse = get_parse_function(subframe)
        parse2 = get_parse_function2(subframe)
        ERD = {}
        # TODO тут номера спутников не соответствуют релаьным (см док gps про сдвиг на +svId и %32)
        for i in range(2):
            ERD[i + 1] = parse(71 + 6 * i, 6)
        for i in range(7):
            ind = 91 + 30 * i
            sv = 3 + 4 * i
            ERD[sv] = parse2(ind - 8, 2, ind, 4)
            for j in range(1, 4):
                ERD[sv + j] = parse(ind - 2 + 6 * j, 6)
        return {'name': 'nmct', 'ERD': ERD}
    except Exception as e:
        print(e)
        print(traceback.format_exc())
        a=0

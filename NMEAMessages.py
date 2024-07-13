def calc_nmea_checksum(cmd):
    if cmd[0] == '$':
        cmd = cmd[1:]
    if '*' in cmd:
        cmd = cmd.split('*')[0]
    checksum = 0
    for sym in cmd:
        checksum ^= ord(sym)
    return format(checksum, '02X')


def tune_baudRate_message(baudRate):
    cmd = f'$PUBX,41,1,0007,0003,{baudRate},0'
    cmd = cmd + '*' + calc_nmea_checksum(cmd) + '\r\n'
    cmd = cmd.encode('utf-8')
    return cmd

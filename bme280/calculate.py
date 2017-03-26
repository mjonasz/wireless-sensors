#!/usr/bin/env python3
# coding: utf-8

import time
import binascii
import re
import sys

digT = []
digP = []
digH = []

t_fine = 0.0


def get_calib_param(calib):
    digT.append((calib[1] << 8) | calib[0])
    digT.append((calib[3] << 8) | calib[2])
    digT.append((calib[5] << 8) | calib[4])
    digP.append((calib[7] << 8) | calib[6])
    digP.append((calib[9] << 8) | calib[8])
    digP.append((calib[11] << 8) | calib[10])
    digP.append((calib[13] << 8) | calib[12])
    digP.append((calib[15] << 8) | calib[14])
    digP.append((calib[17] << 8) | calib[16])
    digP.append((calib[19] << 8) | calib[18])
    digP.append((calib[21] << 8) | calib[20])
    digP.append((calib[23] << 8) | calib[22])
    digH.append(calib[24])
    digH.append((calib[26] << 8) | calib[25])
    digH.append(calib[27])
    digH.append((calib[28] << 4) | (0x0F & calib[29]))
    digH.append((calib[30] << 4) | ((calib[29] >> 4) & 0x0F))
    digH.append(calib[31])

    for i in range(1, 2):
        if digT[i] & 0x8000:
            digT[i] = (-digT[i] ^ 0xFFFF) + 1

    for i in range(1, 8):
        if digP[i] & 0x8000:
            digP[i] = (-digP[i] ^ 0xFFFF) + 1

    for i in range(0, 6):
        if digH[i] & 0x8000:
            digH[i] = (-digH[i] ^ 0xFFFF) + 1


def parse_hex_line(line):
    data_hex = line
    data_hex = re.sub('0[xX]', '', data_hex)
    data_hex = data_hex.split()
    data = []
    for hexByte in data_hex:
        raw_byte = binascii.unhexlify(hexByte)
        data.append(int.from_bytes(raw_byte, byteorder='little'))
    return data


def read_data():
    data = parse_hex_line(input())
    assert len(data) == 8
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw = (data[6] << 8) | data[7]

    temperature = compensate_T(temp_raw)
    pressure = compensate_P(pres_raw)
    humidity = compensate_H(hum_raw)
    return dict(temperature=temperature,pressure=pressure,humidity=humidity)


def read_calib(filename):
    # remember to ignore 0xA0 register byte
    f = open(filename, 'r')
    line = f.read()
    calib = parse_hex_line(line)
    assert len(calib) == 32
    get_calib_param(calib)


def compensate_P(adc_P):
    global t_fine
    pressure = 0.0

    v1 = (t_fine / 2.0) - 64000.0
    v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
    v2 = v2 + ((v1 * digP[4]) * 2.0)
    v2 = (v2 / 4.0) + (digP[3] * 65536.0)
    v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8) + ((digP[1] * v1) / 2.0)) / 262144
    v1 = ((32768 + v1) * digP[0]) / 32768

    if v1 == 0:
        return 0
    pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
    if pressure < 0x80000000:
        pressure = (pressure * 2.0) / v1
    else:
        pressure = (pressure / v1) * 2
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0
    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)

    # print "pressure : %7.2f hPa" % (pressure/100)
    return "{:.2f}".format(pressure / 100)


def compensate_T(adc_T):
    global t_fine
    v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
    t_fine = v1 + v2
    temperature = t_fine / 5120.0
    # print "temp : %-6.2f ℃" % (temperature)
    return "{:.2f}".format(temperature)


def compensate_H(adc_H):
    global t_fine
    var_h = t_fine - 76800.0
    if var_h != 0:
        var_h = (adc_H - (digH[3] * 64.0 + digH[4] / 16384.0 * var_h)) * (
        digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)))
    else:
        return 0
    var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)
    if var_h > 100.0:
        var_h = 100.0
    elif var_h < 0.0:
        var_h = 0.0
    # print "hum : %6.2f ％" % (var_h)
    return "{:.2f}".format(var_h)


if __name__ == '__main__':
    try:
        calib_filename = sys.argv[1]
        read_calib(calib_filename)
        while True:
          print(read_data())

    except KeyboardInterrupt:
        pass

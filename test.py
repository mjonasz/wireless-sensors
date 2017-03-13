#!/usr/bin/env python3
import sys
import struct
import matplotlib.pyplot as plt

signal_loc = 0
last_ones = 0
last_zeros = 0

bits = [0]*8
bi = 0
real_bits = 0

in_progress = False

THRESHOLD = 73

def is_start():
  btc = [1,1,1,0,0,0,1,0]
  for x in range(8):
    if bits[(bi+x)%8] != btc[x]:
      return False
  return True

def is_end():
  for i in range(8):
    if bits[i] != 0:
      return False
  return True

def parse_signal(signal):
  global signal_loc, bits, last_ones, last_zeros, bi, real_bits
  
  if signal>5000:
    #if last_zeros > 0:
    #  print("0:" + str(last_zeros))
    last_ones+=1
    last_zeros=0
    #if in_progress:
    #print("1:" + str(signal_loc) + ":" + str(signal))
  else:
    #if last_ones > 0:
    #  print("1:" +  str(last_ones))
    last_ones=0
    last_zeros+=1
    #if in_progress:
    #print("0:" + str(signal_loc) + ":" + str(signal))
  signal_loc += 1

  if last_zeros > THRESHOLD:
    bits[bi%8] = 0
    #print("0")
    bi+=1
    real_bits+=1
    last_zeros = 0

  if last_ones > THRESHOLD:
    bits[bi%8] = 1
    #print("1")
    bi+=1
    real_bits+=1
    last_ones = 0

def order_bits():
  ordered = []
  for i in range(8):
    ordered.append(bits[(bi+i)%8])
  return ordered

def bits_to_byte():
  b = order_bits()
  a = 0
  for i in range (8):
    a |= b[7 - i] << i
  return a^0xaa

def process(signal):
  global in_progress, real_bits
  parse_signal(signal)
  if in_progress:
    if real_bits == 8:
      #print(order_bits())
      print(chr(bits_to_byte()))
      real_bits = 0
    if is_end():
      in_progress = False
  else:
    if is_start():
      in_progress = True
      real_bits = 0


buff = None
while buff != b'':
  buff = sys.stdin.buffer.read(8192)
  if buff == b'':
    break;
  for i in range(0, len(buff), 2):
    signal = struct.unpack("<h", buff[i:i+2])[0]
    process(signal)

#print(order_bits())
#plt.plot(signal_list[3784910:3793490])
#plt.savefig('out.png')

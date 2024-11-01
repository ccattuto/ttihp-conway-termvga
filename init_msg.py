#!/usr/bin/env python
import sys

init_seq = "\x1bc" + "\x1b[92m"
for c in init_seq:
    print("%02X" % ord(c))
    #print("%d" % ord(c))

f = open(sys.argv[1], "rb")
for c in f.read():
    print("%02X" % c)

#print("00")

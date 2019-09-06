#!/usr/bin/python

#reads map files (.yml) and converts the map contents to binary data

import sys
import struct
import yaml
import numpy as np

if 3 != len (sys.argv):
    print ("usage to save bin file: " + sys.argv[0] + " <filename.yml>" + " <filename.bin>")
    sys.exit (1)

IMG_WIDTH = 640
IMG_HEIGHT = 480

with open(sys.argv[1], 'r') as f:
    doc = yaml.load(f)

xy_data = doc["rmap0"]["data"]
frac_data = doc["rmap1"]["data"]

bin_file = open(sys.argv[2],"wb")
max_concat_data = 0
for i in range(IMG_HEIGHT*IMG_WIDTH):
    #              y_coordinate (11 bits)           x_coordinate (11 bits)      {frac_y,frac_x} (10 bits)
    concat_data = (max(0,xy_data[2*i+1]) << 21) + (max(0,xy_data[2*i]) << 10)  + max(0,frac_data[i])
    # '<' little endian format 'I' unsigned integer
    y_x_frac = struct.pack( '<I' , concat_data )
    bin_file.write( y_x_frac )

bin_file.close()

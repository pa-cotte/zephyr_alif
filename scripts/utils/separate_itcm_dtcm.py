# Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
# Use, distribution and modification of this code is permitted under the
# terms stated in the Alif Semiconductor Software License Agreement
#
# You should have received a copy of the Alif Semiconductor Software
# License Agreement with this file. If not, please write to:
# contact@alifsemi.com, or visit: https://alifsemi.com/license
#

import sys
import os

filename = 'zephyr.map'
search_string = '.last_section '
result=""

print("\033[1;32;40m Generating ITCM and DTCM bins... \033[0m")
scriptPath=sys.argv[1]

with open(os.path.join(scriptPath,filename), 'r') as file:
    for line in file:
        if search_string in line:
            result=line.strip()

lastSec=result.strip().split()
isz=int(lastSec[1],16)+int(lastSec[2],16)
isz=isz+16-(isz%16)

binFileName= 'zephyr.bin'
sz=os.path.getsize(os.path.join(scriptPath,binFileName))
if sz < 1048576 :
    print("\033[1;33;40m separation not needed !!!... \033[0m")
    exit(0)

f = open(os.path.join(scriptPath,binFileName), 'rb')
itcm = f.read(isz)
f.seek(536870912)
dtcm = f.read()
dsz=len(dtcm)
f.close()

n=16-(dsz%16)

itcmFN= 'itcm.bin'
f = open(os.path.join(scriptPath,itcmFN), 'wb')
f.write(itcm)
f.close()

dtcmFN= 'dtcm.bin'
f = open(os.path.join(scriptPath,dtcmFN), 'wb')
f.write(dtcm)
f.write(b'\0' * n)
f.close()
print("\033[1;32;40m itcm.bin and dtcm.bins Done !!!... \033[0m")

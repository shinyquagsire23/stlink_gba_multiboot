import serial
import struct
import time
import sys

def GBA_Send32(val):
    val = int(val) & 0xFFFFFFFF

    send = struct.pack("<LL", 0, val);
    ser.write(send)

    data = ser.read(8)
    cmd,val = struct.unpack("<LL", data);

    #time.sleep(0.000001)

    #print (hex(val))

    return val

def GBA_Send32Fast(val):

    val = int(val) & 0xFFFFFFFF

    send = struct.pack("<LL", 1, val);
    ser.write(send)

    time.sleep(0.0005)

def LED(val):
    val = int(val) & 0xFFFFFFFF

    send = struct.pack("<LL", 2, val);
    ser.write(send)

def wait_mb():
    resp = 0
    while resp != 0x72026202:
        resp = GBA_Send32(0x00006202)

if len(sys.argv) < 3:
    print ("Usage: upload.py /dev/tty.usbmodemWhatever your_file_to_send_mb.gba")
    exit(-1)

port = sys.argv[1]
to_send = sys.argv[2]

ser = serial.Serial()
ser.port=port
ser.open()

GBbuffer = open(to_send, "rb").read()
GBbuffer_length = len(GBbuffer)
print (hex(GBbuffer_length))
read_from_gba = 0
fp = 0
bit = 0
fcnt = 0
mbfsize = GBbuffer_length - 0xC0

wait_mb()

#LED(1)

read_from_gba = GBA_Send32(0x00006202);
read_from_gba = GBA_Send32(0x00006102);

fp=0;
for i in range(0, 32):
    w = GBbuffer[fp];
    w = GBbuffer[fp+1] << 8 | w;
    fcnt += 2;
    fp += 2;
    r = GBA_Send32(w);


for i in range(0, 32):
    w = GBbuffer[fp];
    w = GBbuffer[fp+1] << 8 | w;
    fcnt += 2;
    fp += 2;
    r = GBA_Send32(w);

old_size = mbfsize
mbfsize = (mbfsize+0x0f)&0xfffffff0;    #align length to xfer to 16

for i in range(0, mbfsize - old_size):
    GBbuffer += b'\x00'

for i in range(0, 32):
    w = GBbuffer[fp];
    w = GBbuffer[fp+1] << 8 | w;
    fcnt += 2;
    fp += 2;
    r = GBA_Send32(w);



r = GBA_Send32(0x00006200);
r = GBA_Send32(0x00006202);
r = GBA_Send32(0x000063d1);
r = GBA_Send32(0x000063d1);
m = ((r & 0x00ff0000) >>  8) + 0xffff00d1;
h = ((r & 0x00ff0000) >> 16) + 0xf;
r = GBA_Send32((((r >> 16) + 0xf) & 0xff) | 0x00006400);
r = GBA_Send32((mbfsize - 0x190) // 4);
f = (((r & 0x00ff0000) >> 8) + h) | 0xffff0000;
c = 0x0000c387;

led_state = 0;

fp=0xC0;
while(fcnt < mbfsize):
    if (fp % 0x1000 == 0):
        led_state = (0 if led_state == 1 else 1)
        print ("sending...", hex(fp))
        LED(led_state)

    w = GBbuffer[fp];
    w = GBbuffer[fp+1] <<  8 | w;
    w = GBbuffer[fp+2] << 16 | w;
    w = GBbuffer[fp+3] << 24 | w;
    fp += 4;
    w2 = w;
    for bit in range(0, 32):
        if((c ^ w) & 0x01):
            c = (c >> 1) ^ 0x0000c37b;
        else:
            c = c >> 1;
        w = w >> 1;
    m = (0x6f646573 * m) + 1;
    GBA_Send32Fast(w2 ^ ((~(0x02000000 + fcnt)) + 1) ^m ^0x43202f2f);
    fcnt = fcnt + 4;

LED(1);


for bit in range(0, 32):
    if((c ^ f) & 0x01):
        c =( c >> 1) ^ 0x0000c37b;
    else:
        c = c >> 1;
    f = f >> 1;


while(read_from_gba!=0x00750065):
    read_from_gba = GBA_Send32(0x00000065);

r = GBA_Send32(0x00000066);
r = GBA_Send32(c);

LED(0)

#time.sleep(5)
#while True:
#    print(hex(GBA_Send32(0)))

#ser.write(b'\x01')
#data = ser.read(13)
#print (data)

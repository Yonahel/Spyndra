import serial                     # we need to import the pySerial stuff to use

# important AX-12 constants
AX_WRITE_DATA = 3
AX_READ_DATA = 4

s = serial.Serial()               # create a serial port object
s.baudrate = 1000000              # baud rate, in bits/second
#s.port = "/dev/ttyUSB0"           # servo's port
#s.open()

# set register values
def setReg(ID,reg,values):
    length = 3 + len(values)
    checksum = 255-((index+length+AX_WRITE_DATA+reg+sum(values))%256)          
    s.write(chr(0xFF)+chr(0xFF)+chr(ID)+chr(length)+chr(AX_WRITE_DATA)+chr(reg))
    for val in values:
        s.write(chr(val))
    s.write(chr(checksum))

def getReg(index, regstart, rlength):
    s.flushInput()   
    checksum = 255 - ((6 + index + regstart + rlength)%256)
    s.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(0x04)+chr(AX_READ_DATA)+chr(regstart)+chr(rlength)+chr(checksum))
    vals = list()
    s.read()   # 0xff
    s.read()   # 0xff
    s.read()   # ID
    length = ord(s.read()) - 1
    s.read()   # toss error    
    while length > 0:
        vals.append(ord(s.read()))
        length = length - 1
    if rlength == 1:
        return vals[0]
    return vals

setReg(1,30,((512%256),(512>>8)))  # this will move servo 1 to centered position (512)
print getReg(1,43,1)               # get the temperature
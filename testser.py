# Python imports used
#################################
import time
import serial

COMPORT = "COM8"

def ser_open():

    global ser
    ser = serial.Serial(COMPORT,9600, timeout=1)#, rtscts=0) # open serial port #7

    # Power up the ADC device
    ser.setRTS(1)             #set RTS on  (must be positive)
    ser.setDTR(0)             #set DTR off (must be negative)

    # check which port was realy used
    print "Serial port ",ser.portstr," Opened"

    # allow the ADC to power up
    time.sleep(2)
    ser.read(10)    #Clear garbage
    print "ready"


def ser_receive():
    
    global nData, nData1
    global i, time1, time2
    global rawData
    data = ser.read(200)
    nData = nData + len(data)
    rawData = rawData + data
    i = i+1
    #print "%s" %data
    #print "%d" %len(data)
    if i % 2000 == 0:
        time2=time1
        time1=time.time()
        #print("Raw data: {0}".format(data))
        print "Total kbytes received: %d" % int(nData/1024)
        print "kBytes/s: %f" % ((nData-nData1)/1024/(time1-time2))
        #print "frame size: %d" %len(data)
       # print "Data %s" %data
        file1.write(rawData)
        rawData = ""
        nData1=nData
    
def main():

    global file1
    file1 = open("dump.txt",'w')
    ser_open()
    while 1:
        ser_receive()
  
if __name__ == '__main__':
    ser = 0
    nData = 0
    nData1 = 0
    rawData = 0
    i = 0
    time1 = 0
    time2 = 0
    rawData = ""
    main()

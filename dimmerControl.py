import time
import serial
import sys
import threading

class bcolors:
    YELLOW  = '\033[93m'
    RED     = '\033[1;31m'
    BLACK   = ''
    BLUE    = '\033[1;34m'



try:
    ser = serial.Serial('/dev/ttyUSB0', 9600)
except:
    print 'No serial. I quit.'
    sys.exit(0)

while True:

    command = None
    while not command:
        try:
            command = int(raw_input(bcolors.BLUE + 'What is your command? '))
            ser.write(chr(command))
        except ValueError:
            if command == 104:
                print 'Help'
            else:
                print bcolors.RED + 'Invalid Number'

    message = ord(ser.read())
#    print(ord(message))
    if message == 0:
        print bcolors.RED + 'Error, wrong command issued.'
    elif message == 1:
        print bcolors.RED + '1 of 2 commands acknowled.'
    elif message == 2:
        print bcolors.RED + '2 of 2 commands acknowled.'
    else:
        print bcolors.RED + '???'

    time.sleep(0.5)

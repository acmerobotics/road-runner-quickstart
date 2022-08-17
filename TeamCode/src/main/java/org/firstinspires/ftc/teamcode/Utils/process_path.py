import json
import os
import argparse

parser = argparse.ArgumentParser(description='Parse wpilib json path and upload it to the connected adb device (adb must be on your path and you must be connected to the control hub).\nRun this script in the output directory of path planner.')
parser.add_argument('pathname', type=str, metavar='Path', help='The name (not direct file name) of the path to parse and upload.')

fname = parser.parse_args().pathname

dataFile = open(f'{fname}.wpilib.json')
data = json.load(dataFile)

csvOut = open(f'{fname}.csv', 'w')
csvOut.write('time,x,y\n')

for datum in data:
    time = datum['time']
    x = datum['pose']['translation']['x'] * 10
    y = datum['pose']['translation']['y'] * 10
    
    csvOut.write(f'{time},{x},{y}\n')

csvOut.close()

os.system(f'adb -s 192.168.43.1:5555 push {fname}.csv /sdcard/FIRST')

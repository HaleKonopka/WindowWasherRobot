import serial
import time
import cbor
from struct import *
import binascii
import sys
import pandas as pd
import time
import argparse
import os

# Parse Arguments
parser = argparse.ArgumentParser()
parser.add_argument('csv', type=str, help='CSV file which data will be logged to')
args = parser.parse_args()
csvName = args.csv
print("Logging to: " + csvName)

# Created python equivalent of millis() function for convenience
millis = lambda : int(round(time.time() * 1000))
timeOfLastCSVWrite__ms = millis()

# Create empty file with name 
if os.path.isfile(csvName):
    print("File: " + csvName + " already exists. Please choose different name")
    sys.exit(-1)
else:
    with open(csvName,'w') as f:
        pass # write nothing just to create the file

# Create empty pandas dataframe
log = pd.DataFrame()
index = 0

# Establish serial communication
ser = serial.Serial('/dev/ttyACM0',115200)
time.sleep(2) # sleep for 2 seconds for serial port to come up
ser.reset_input_buffer() # Flush out any lingering entries

while True:
    # Hangs until message is received over serial
    # Payload length sent first (2 bytes) followed by the payload itself
    # Messages are unidirectional from Arduino to Rasberry Pi, formatted as:
    #   [ payload length LSB | payload length MSB | CBOR payload ] 
    payload_len_bytes = ser.read(2)
    (payload_len, ) = unpack('h',payload_len_bytes) # interpreting bitfield as short (2 bytes)
    # print("Payload Lengh: " + str(payload_len) + " bytes") # Useful debug if messages fail
    payload_bytes=ser.read(payload_len)
    print("Log Received")
    # print("Payload: " + str(binascii.hexlify(payload_bytes))) # Useful debug if messages fail
    payload_dict = cbor.loads(payload_bytes) # convert CBOR vector to dictionary
    # print("CBOR Parsed Dict: " + str(payload_dict)) # Useful debug if messages fail

    # Put data into pandas frame and store in CSV
    # Write out pandas dataframe to csv every 30 sec (overwriting existing dataframe)
    if (millis() - timeOfLastCSVWrite__ms > 3000): 
        # Move prior csv and give temp name (in case of interruption while logging)
        os.rename(csvName, csvName + "_backup")
        with open(csvName, 'w') as f:
            log.to_csv(f, header=True, index=False)
        os.remove(csvName + "_backup")
    
    # Append new DataFrame
    new_log = pd.DataFrame(payload_dict, index=[index])
    index = index + 1;
    log = log.append(new_log, ignore_index=True, sort=False) 
    # Fine to append any new dictionary entries

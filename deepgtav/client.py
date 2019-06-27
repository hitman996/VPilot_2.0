#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import socket, struct
import pickle
import gzip
import os
import numpy as np
from numpy.lib.stride_tricks import as_strided

class Targets:
    def __init__(self, datasetPath, compressionLevel, divideByTrip):
        self.pickleFile = None
        self.datasetPath = datasetPath
        self.compressionLevel = compressionLevel
        self.divideByTrip = divideByTrip
        self.carNum = 0

        if not self.divideByTrip and self.datasetPath != None:
            fullpath = os.path.dirname(self.datasetPath) + '\\' + 'dataset.pz'
            self.pickleFile = gzip.open(fullpath, mode='ab', compresslevel=self.compressionLevel)
        elif self.datasetPath != None:
            while True:
                if os.path.exists(os.path.dirname(self.datasetPath) + '\\' + 'trip' + str(self.carNum) + '.pz'): self.carNum += 1
                else: break

    # def parse(self, frame, lidar, jsonstr):
    #     try:
    #         dct = json.loads(jsonstr)
    #     except ValueError:
    #         return None
    #
    #     # if self.divideByTrip and self.datasetPath != None:
    #     #     # assert('drivingMode' in dct)
    #     #     if dct['drivingMode'][0] == 0 :
    #     #         fullpath = os.path.dirname(self.datasetPath) + '\\' + 'trip' + str(self.carNum) + '.pz'
    #     #         if self.pickleFile != None: self.pickleFile.close()
    #     #         self.pickleFile = gzip.open(fullpath, mode='ab', compresslevel=self.compressionLevel)
    #     #         self.carNum += 1
    #
    #
    #     if frame != None: dct['frame'] = frame
    #     # if lidar != None: dct['lidar'] = np.frombuffer(lidar, dtype=np.float32)
    #     if self.pickleFile != None:
    #         pickle.dump(dct, self.pickleFile)
    #     return dct['frame']
    def parse(self, frame, jsonstr):
        try:
            dct = json.loads(jsonstr)
        except ValueError:
            return None

        dct['frame'] = frame
        if self.pickleFile != None:
            pickle.dump(dct, self.pickleFile)
        return dct

    def close(self):
        if self.pickleFile != None:
            self.pickleFile.close()  

class Client:
    def __init__(self, ip='localhost', port=8000, datasetPath=None, compressionLevel=0, divideByTrip=False):
        print('Trying to connect to DeepGTAV')
        
        self.targets = Targets(datasetPath, compressionLevel, divideByTrip)
        self.frame, self.lidar = False, False
        self.check = np.array([0, 0])

        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((ip, int(port)))
        except:
            print('ERROR: Failed to connect to DeepGTAV')
        else:
            print('Successfully connected to DeepGTAV')

    def sendMessage(self, message):
        if str(message.__class__.__name__) == 'Start' or str(message.__class__.__name__) == 'Config':
            self.frame, self.lidar = message.activate_bytes_frame()

        jsonstr = message.to_json().encode('utf-8')
        try:
            self.s.sendall(len(jsonstr).to_bytes(4, byteorder='little'))
            self.s.sendall(jsonstr)
        except Exception as e:
            print('ERROR: Failed to send message. Reason:', e)
            return False
        return True

    # def recvMessage(self):
    #
    #     if self.frame:
    #         frame = self._recvall()
    #         if not frame:
    #             print('ERROR: Failed to receive frame')
    #             return None
    #     else: frame = None
    #
    #     if self.lidar:
    #         lidar = self._recvall()
    #         if not lidar:
    #             print('ERROR: Failed to receive lidar')
    #             return None
    #     else: lidar = None
    #
    #     data = self._recvall()
    #     if not data:
    #         print('ERROR: Failed to receive message')
    #         return None
    #
    #     return self.targets.parse(frame, lidar, data.decode('utf-8'))
    def frame2numpy(frame, frameSize):
        buff = np.fromstring(frame, dtype='uint8')
        # Scanlines are aligned to 4 bytes in Windows bitmaps
        strideWidth = int((frameSize[0] * 3 + 3) / 4) * 4
        # Return a copy because custom strides are not supported by OpenCV.
        return as_strided(buff, strides=(strideWidth, 3, 1), shape=(frameSize[1], frameSize[0], 3)).copy()

    def recvMessage(self):
        frame = self._recvall()
        if not frame:
            print('ERROR: Failed to receive frame')
            return None
        data = self._recvall()
        if not data:
            print('ERROR: Failed to receive message')
            return None
        return self.targets.parse(frame, data.decode('utf-8'))


    def _recvall(self):
        #Receive first size of message in bytes, make up for 4 bytes then read
        data = b""
        while len(data) < 4:
            packet = self.s.recv(4 - len(data))
            if not packet: return None
            data += packet
        size = struct.unpack('I', data)[0]

        #We now proceed to receive the full message
        data = b""
        while len(data) < size:
            packet = self.s.recv(size - len(data))
            if not packet: return None
            data += packet
        return data

    def close(self):
        self.s.close()
        self.targets.close()


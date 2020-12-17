from time import sleep

import serial
import utils
from threading import Thread
from multiprocessing import Queue, Process
import numpy as np
from radar_depthmap import RadarPlotter
from const import *
from config import parse_config


class MMWave(Thread):
    def __init__(self, queue):
        Thread.__init__(self)
        self.__ports = {
            "data": "COM7",
            "control": "COM8"
        }
        self.__data_port = None
        self.__control_port = None
        self.__config = parse_config("configs/profile_3d.cfg")
        self.__queue = queue
        self.__alive = True

    def connect(self):
        try:
            self.__control_port = serial.Serial(self.__ports["control"], 115200, timeout=0.01)
            self.__data_port = serial.Serial(self.__ports["data"], 921600, timeout=0.01)
            return self.is_connected()
        except serial.serialutil.SerialException:
            print("Cannot connect to the ports.")
            return False

    def disconnect(self):
        if self.is_connected():
            self.__control_port.close()
            self.__data_port.close()


    def is_connected(self):
        if self.__control_port is not None and self.__data_port is not None:
            return self.__control_port.isOpen() and self.__data_port.isOpen()
        return False

    def get_config(self):
        return self.__config

    def send_config(self):
        utils.send_config(self.__control_port, utils.load_config("configs/profile_3d.cfg"))

    def to_int(self, byte_vector):
        return np.sum(np.frombuffer(byte_vector, dtype=np.uint8) * [1, 256, 65536, 16777216])

    def to_float(self, byte_vector):
        return np.frombuffer(byte_vector, dtype=np.uint8).view('<f4')[0]

    def process_buffer(self, buffer):
        index = 8
        version = buffer[index: index + 4]
        index += 4
        totalPacketLen = self.to_int(buffer[index: index + 4])
        index += 4
        tlv_platform = self.to_int(buffer[index: index + 4])
        index += 4
        frameNumber = self.to_int(buffer[index: index + 4])
        index += 4
        timeCpuCycles = self.to_int(buffer[index: index + 4])
        index += 4
        numDetectedObj = self.to_int(buffer[index: index + 4])
        index += 4
        numTLVs = self.to_int(buffer[index: index + 4])
        index += 4
        subFrameNo = self.to_int(buffer[index: index + 4])
        index += 4

        if totalPacketLen == len(buffer):
            print(frameNumber, subFrameNo, numTLVs, numDetectedObj)
            detectedPoints_byteVecIdx = -1
            for tlvidx in np.arange(numTLVs):
                tlvtype = self.to_int(buffer[index: index + 4])
                index += 4
                tlvlength = self.to_int(buffer[index: index + 4])
                index += 4
                if tlvtype == TLV_TYPE["MMWDEMO_OUTPUT_MSG_DETECTED_POINTS"]:
                    detectedPoints_byteVecIdx = index

                index += tlvlength

            if detectedPoints_byteVecIdx != -1:
                x_coord = []
                y_coord = []
                z_coord = []
                doppler = []
                for i in np.arange(numDetectedObj):
                    startIdx = detectedPoints_byteVecIdx + i * 16
                    x_coord.append(self.to_float(buffer[startIdx:startIdx + 4]))
                    y_coord.append(self.to_float(buffer[startIdx + 4:startIdx + 8]))
                    z_coord.append(self.to_float(buffer[startIdx + 8:startIdx + 12]))
                    doppler.append(self.to_float(buffer[startIdx + 12:startIdx + 16]))

                self.__queue.put({"frame_no": frameNumber, "x": x_coord, "y": y_coord, "z": z_coord})

    def terminate(self):
        self.__alive = False

    def run(self):
        if self.__control_port is None or self.__data_port is None:
            print("Please connect to the device.")
            return

        Thread(target=utils.cli_listener, args=(self.__control_port,)).start()

        self.send_config()

        buffer = b""
        while self.__alive:
            try:
                res = self.__data_port.read(32)
                if res[:len(MAGIC_WORD)] == MAGIC_WORD:
                    if len(buffer) > 0:
                        self.process_buffer(buffer)
                    buffer = b"" + res
                    continue
                buffer += res
            except KeyboardInterrupt:
                self.disconnect()
                break


if __name__ == '__main__':
    queue = Queue()
    MMWave(queue).start()

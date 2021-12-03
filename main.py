import serial
import utils
from threading import Thread
from multiprocessing import Queue
import numpy as np
from radar_depthmap import RadarPlotter
import time
import json

from config import parse_config

control_port = "COM4"
data_port = "COM3"

magic_word = b'\x02\x01\x04\x03\x06\x05\x08\x07'

TLV_type = {
    "MMWDEMO_OUTPUT_MSG_DETECTED_POINTS": 1,
    "MMWDEMO_OUTPUT_MSG_RANGE_PROFILE": 2,
    "MMWDEMO_OUTPUT_MSG_NOISE_PROFILE": 3,
    "MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP": 4,
    "MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP": 5,
    "MMWDEMO_OUTPUT_MSG_STATS": 6,
    "MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO": 7,
    "MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP": 8,
    "MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS": 9,
    "MMWDEMO_OUTPUT_MSG_MAX": 10
}


def to_int(byte_vector):
    return np.sum(np.frombuffer(byte_vector, dtype=np.uint8) * [1, 256, 65536, 16777216])


def to_float(byte_vector):
    return np.frombuffer(byte_vector, dtype=np.uint8).view('<f4')[0]


try:
    control_serial = serial.Serial(control_port, 115200, timeout=0.01)
    data_serial = serial.Serial(data_port, 921600, timeout=0.01)

    if control_serial is not None and data_serial is not None:
        Thread(target=utils.cli_listener, args=(control_serial,)).start()
        # Sending configuration
        utils.send_config(control_serial, utils.load_config("configs/profile_3d.cfg"))
        config = parse_config("configs/profile_3d.cfg")
        queue = Queue()
        RadarPlotter(queue, config).start()
        # Listening to the data port
        buffer = b""
        index = 8
        while True:
            try:
                res = data_serial.read(32)
                if res[:len(magic_word)] == magic_word:
                    if len(buffer) > 0:
                        version = buffer[index: index + 4]
                        index += 4
                        totalPacketLen = to_int(buffer[index: index + 4])
                        index += 4
                        tlv_platform = to_int(buffer[index: index + 4])
                        index += 4
                        frameNumber = to_int(buffer[index: index + 4])
                        index += 4
                        timeCpuCycles = to_int(buffer[index: index + 4])
                        index += 4
                        numDetectedObj = to_int(buffer[index: index + 4])
                        index += 4
                        numTLVs = to_int(buffer[index: index + 4])
                        index += 4
                        subFrameNo = to_int(buffer[index: index + 4])
                        index += 4

                        if totalPacketLen == len(buffer):
                            # print(frameNumber, subFrameNo, numTLVs, numDetectedObj)
                            detectedPoints_byteVecIdx = -1
                            for tlvidx in np.arange(numTLVs):
                                tlvtype = to_int(buffer[index: index + 4])
                                index += 4
                                tlvlength = to_int(buffer[index: index + 4])
                                index += 4
                                if tlvtype == TLV_type["MMWDEMO_OUTPUT_MSG_DETECTED_POINTS"]:
                                    detectedPoints_byteVecIdx = index

                                index += tlvlength

                            if detectedPoints_byteVecIdx != -1:
                                x_coord = []
                                y_coord = []
                                z_coord = []
                                doppler = []
                                for i in np.arange(numDetectedObj):
                                    startIdx = detectedPoints_byteVecIdx + i * 16
                                    x_coord.append(to_float(buffer[startIdx:startIdx + 4]))
                                    y_coord.append(to_float(buffer[startIdx + 4:startIdx + 8]))
                                    z_coord.append(to_float(buffer[startIdx + 8:startIdx + 12]))
                                    doppler.append(to_float(buffer[startIdx + 12:startIdx + 16]))

                                range = np.sqrt(np.square(x_coord) + np.square(y_coord) + np.square(z_coord))
                                rangeId = [np.round(r / config["dataPath"][subFrameNo]["rangeIdxToMeters"]) for r in
                                           range]
                                dopplerId = [np.round(d / config["dataPath"][subFrameNo]["dopplerResolutionMps"]) for d
                                             in
                                             doppler]
                                print("Sending frame no: ", frameNumber)
                                queue.put({"frame_no": frameNumber, "x": x_coord, "y": y_coord, "z": z_coord})

                    buffer = b"" + res
                    index = 8
                    continue
                buffer += res
            except KeyboardInterrupt:
                if control_serial.isOpen():
                    control_serial.close()
                if data_serial.isOpen():
                    data_serial.close()
                break
except serial.serialutil.SerialException:
    print("Cannot connect to the ports")

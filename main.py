import serial
import utils
from threading import Thread
import numpy as np

control_port = "COM8"
data_port = "COM7"

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
    return np.frombuffer(byte_vector, dtype=np.uint8).view('<f4')


# def intify(value, base=16, size=2):
#     if type(value) not in (list, tuple, bytes,):
#         value = (value,)
#     if (type(value) in (bytes,) and base == 16) or (type(value) in (list, tuple,)):
#         return sum([item * ((base ** size) ** i) for i, item in enumerate(value)])
#     else:
#         return sum([((item // 16) * base + (item % 16)) * ((base ** size) ** i) for i, item in enumerate(value)])


try:
    control_serial = serial.Serial(control_port, 115200, timeout=0.01)
    data_serial = serial.Serial(data_port, 921600, timeout=0.01)

    if control_serial is not None and data_serial is not None:
        Thread(target=utils.cli_listener, args=(control_serial,)).start()
        # Sending configuration
        config = utils.load_config("configs/profile_3d.cfg")
        utils.send_config(control_serial, config)
        # Listening to the data port
        buffer = b""
        index = 8
        while True:
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
                    currentSubFrameNumber = to_int(buffer[index: index + 4])
                    index += 4

                    if totalPacketLen == len(buffer):
                        print(frameNumber, currentSubFrameNumber, numTLVs, numDetectedObj)
                        detectedPoints_byteVecIdx = -1
                        for tlvidx in range(numTLVs):
                            tlvtype = to_int(buffer[index: index + 4])
                            index += 4
                            tlvlength = to_int(buffer[index: index + 4])
                            index += 4
                            if tlvtype == TLV_type["MMWDEMO_OUTPUT_MSG_RANGE_PROFILE"]:
                                detectedPoints_byteVecIdx = index

                            index += tlvlength

                        if detectedPoints_byteVecIdx != -1:
                            x_coord = []
                            y_coord = []
                            z_coord = []
                            doppler = []
                            for i in range(numDetectedObj):
                                startIdx = detectedPoints_byteVecIdx + i * 16
                                x_coord.append(to_float(buffer[startIdx:startIdx + 4]))
                                y_coord.append(to_float(buffer[startIdx + 4:startIdx + 8]))
                                z_coord.append(to_float(buffer[startIdx + 8:startIdx + 12]))
                                doppler.append(to_float(buffer[startIdx + 12:startIdx + 16]))

                buffer = b"" + res
                index = 8
                continue
            buffer += res




except serial.serialutil.SerialException:
    print("Cannot connect to the ports")

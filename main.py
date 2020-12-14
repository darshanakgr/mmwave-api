import serial
import utils
from threading import Thread

control_port = "COM8"
data_port = "COM7"

try:
    control_serial = serial.Serial(control_port, 115200, timeout=0.01)
    data_serial = serial.Serial(data_port, 921600, timeout=0.01)

    if control_serial is not None and data_serial is not None:
        Thread(target=utils.cli_listener, args=(control_serial,)).start()
        # Sending configuration
        config = utils.load_config("configs/profile_3d.cfg")
        utils.send_config(control_serial, config)
        # Listening to the data port



except serial.serialutil.SerialException:
    print("Cannot connect to the ports")

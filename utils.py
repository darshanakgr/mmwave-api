from serial import Serial
from time import sleep


def load_config(path: str):
    config = open(path, "r")
    lines = [bytes(line, 'latin-1') for line in config.readlines() if not line.startswith("%") and len(line) > 0]
    return lines


def send_command(port: Serial, cmd: str):
    port.write(cmd)
    sleep(0.01)


def send_config(port: Serial, lines: list):
    for line in lines:
        port.write(line)
        sleep(0.01)


def cli_listener(port: Serial):
    while True:
        try:
            if port is not None and port.isOpen():
                line = port.readline().decode("latin-1")
                if len(line) > 0:
                    print(line.rstrip())
            else:
                break
            sleep(0.01)
        except KeyboardInterrupt:
            # port.close()
            break

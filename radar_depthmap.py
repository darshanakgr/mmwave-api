import matplotlib.pyplot as plt
from multiprocessing import Queue, Process
from time import sleep
import numpy as np
import cv2

from config import parse_config


class RadarPlotter(Process):
    def __init__(self, queue: Queue, config):
        Process.__init__(self)
        self.__config = config
        range_max = self.__config["dataPath"][0]["rangeMeters"]
        # Defining the limits of x, y, and z axes.
        self.__range_x = (-range_max / 2 + 2, range_max / 2 + 2)
        self.__range_y = (0, range_max+2)
        self.__range_z = (-range_max / 2 + 2, range_max / 2 + 2)
        self.__queue = queue

    def normalize(self, x, range, bins=15):
        l, u = range
        r = u - l
        if bins == 1:
            return (np.array(x) - l) / r * bins
        return np.ceil((np.array(x) - l) / r * bins).astype(int)

    def run(self):
        while True:
            if not self.__queue.empty():
                data = self.__queue.get()
                frame = np.zeros((16, 16))
                x = self.normalize(data["x"], self.__range_x)
                y = self.normalize(data["y"], self.__range_y)
                z = self.normalize(data["z"], self.__range_z, bins=1)
                frame[x, y] = z
                cv2.imshow("Radar", cv2.resize(frame.T, (600, 600)))
                print("Plotted frame no: ", data["frame_no"])
            # Press 'q' to quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            # Clean up
        cv2.destroyAllWindows()


def generate_data(queue: Queue):
    while True:
        queue.put({"x": np.random.random(10), "y": np.random.random(10), "z": np.random.random(10)})
        sleep(0.1)


if __name__ == '__main__':
    q = Queue()
    config = parse_config("configs/profile_3d.cfg")
    RadarPlotter(q, config).start()
    Process(target=generate_data, args=(q,)).start()

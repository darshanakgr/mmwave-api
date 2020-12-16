import matplotlib.pyplot as plt
from multiprocessing import Queue, Process
from time import sleep
import numpy as np


class Plotter(Process):
    def __init__(self, queue: Queue, config):
        Process.__init__(self)
        self.__figure, self.__axes = plt.subplots(ncols=1, nrows=2)
        self.__config = config
        self.__queue = queue
        self.__figure.tight_layout(pad=2)
        range_max = self.__config["dataPath"][0]["rangeMeters"]
        # Defining the limits of x, y, and z axes.
        self.__range_x = (-range_max / 2, range_max / 2)
        self.__range_y = (0, range_max)
        self.__range_z = (-range_max / 2, range_max / 2)

    def run(self):
        while True:
            try:
                if not self.__queue.empty():
                    data = self.__queue.get()
                    self.__update_plot(data)
                    print("Plotted frame no: ", data["frame_no"])
                sleep(0.001)
            except KeyboardInterrupt:
                break

    def __update_plot(self, data):
        # XY Plot
        self.__axes[0].clear()
        self.__axes[0].scatter(data["x"], data["y"])
        self.__axes[0].set_xlim(self.__range_x)
        self.__axes[0].set_ylim(self.__range_y)
        # XZ Plot
        self.__axes[1].clear()
        self.__axes[1].scatter(data["x"], data["z"])
        self.__axes[1].set_xlim(self.__range_x)
        self.__axes[1].set_ylim(self.__range_z)

        self.__figure.canvas.draw()
        self.__figure.canvas.flush_events()
        plt.show(block=False)


def generate_data(queue: Queue):
    while True:
        queue.put({"x": np.random.random(10), "y": np.random.random(10)})
        sleep(0.1)


if __name__ == '__main__':
    q = Queue()
    Plotter(q).start()
    Process(target=generate_data, args=(q,)).start()

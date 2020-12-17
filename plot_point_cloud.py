import matplotlib.pyplot as plt
from multiprocessing import Queue, Process
from time import sleep
import numpy as np


class Plotter(Process):
    def __init__(self, queue: Queue, config):
        Process.__init__(self)
        self.__figure, self.__axes = plt.subplots(ncols=2, nrows=1)
        self.__config = config
        range_max = self.__config["dataPath"][0]["rangeMeters"]
        # Defining the limits of x, y, and z axes.
        self.__range_x = (-range_max / 2, range_max / 2)
        self.__range_y = (0, range_max)
        self.__range_z = (-range_max / 2, range_max / 2)
        self.__queue = queue
        self.__figure.show()
        self.__figure.canvas.draw()
        self.__figure.tight_layout(pad=2)
        self.__xy_plot = self.__axes[0].plot([], [], animated=True)[0]
        self.__xz_plot = self.__axes[1].plot([], [], animated=True)[0]

        # self.__xy_plot.set_xlim(self.__range_x)
        # self.__xy_plot.set_ylim(self.__range_y)
        # self.__xz_plot.set_xlim(self.__range_x)
        # self.__xz_plot.set_ylim(self.__range_z)

        self.__backgrounds = [self.__figure.canvas.copy_from_bbox(ax.bbox) for ax in self.__axes]



    def run(self):
        while True:
            try:
                data = None
                while not self.__queue.empty():
                    data = self.__queue.get()
                self.__update_plot(data)
                print("Plotted frame no: ", data["frame_no"])
                sleep(0.001)
            except KeyboardInterrupt:
                break

    def __update_plot(self, data):
        # XY Plot
        self.__figure.canvas.restore_region(self.__backgrounds[0])
        self.__xy_plot.set_xdata(data["x"])
        self.__xy_plot.set_ydata(data["y"])
        self.__axes[0].draw_artist(self.__xy_plot)
        self.__figure.canvas.blit(self.__axes[0].bbox)
        # XZ Plot
        self.__figure.canvas.restore_region(self.__backgrounds[1])
        self.__xz_plot.set_xdata(data["x"])
        self.__xz_plot.set_ydata(data["y"])
        self.__axes[1].draw_artist(self.__xz_plot)
        self.__figure.canvas.blit(self.__axes[1].bbox)

        # self.__figure.canvas.draw()
        # self.__figure.canvas.flush_events()
        # plt.show(block=False)


def generate_data(queue: Queue):
    while True:
        queue.put({"x": np.random.random(10), "y": np.random.random(10)})
        sleep(0.1)


if __name__ == '__main__':
    q = Queue()
    Plotter(q).start()
    Process(target=generate_data, args=(q,)).start()

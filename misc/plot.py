from os import lseek
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from numpy.core.defchararray import split


def init(data):
    fig, ax = plt.subplots(nrows=2, ncols=1, figsize=(12, 8))
    sz = len(data)

    plots = []
    texts = []
    for s, series_name in enumerate(("Accel", "Gyro")):
        ax[s].clear()

        ax[s].axis([0, sz, data.max(), data.min()])
        ax[s].xaxis.set_ticks(np.arange(0, sz + 1, 20))
        ax[s].grid(alpha=0.3)

        plots.append((
            *ax[s].plot(np.arange(0, sz), data[:, s*3 + 0],
                        lw=1, label="X", c="#C00"),
            *ax[s].plot(np.arange(0, sz), data[:, s*3 + 1],
                        lw=1, label="Y", c="#0C0"),
            *ax[s].plot(np.arange(0, sz), data[:, s*3 + 2],
                        lw=1, label="Z", c="#00C")))

        ax[s].legend(loc='lower left')

        texts.append((
            ax[s].text(sz + 2, data[-1, s*3 + 0], "X: " +
                       str(data[-1, s*3 + 0]), c="#C00"),
            ax[s].text(sz + 2, data[-1, s*3 + 1], "Y: " +
                       str(data[-1, s*3 + 1]), c="#0C0"),
            ax[s].text(sz + 2, data[-1, s*3 + 2], "Z: " +
                       str(data[-1, s*3 + 2]), c="#00C")))

        ax[0].set_title("Accelerometer")
        ax[1].set_title("Gyroscope")

    return fig, ax, plots, texts


def animate(data, ax, plots, texts):
    sz = len(data)

    for s, series_name in enumerate(("Accel", "Gyro")):
        ax[s].axis([0, sz, -300, 300])

        plots[s][0].set_data(np.arange(0, sz), data[:, s*3 + 0])
        plots[s][1].set_data(np.arange(0, sz), data[:, s*3 + 1])
        plots[s][2].set_data(np.arange(0, sz), data[:, s*3 + 2])

        texts[s][0].set_position((sz * 1.01, data[-1, s*3 + 0]))
        texts[s][1].set_position((sz * 1.01, data[-1, s*3 + 1]))
        texts[s][2].set_position((sz * 1.01, data[-1, s*3 + 2]))

        texts[s][0].set_text(str(data[-1, s*3 + 0]))
        texts[s][1].set_text(str(data[-1, s*3 + 1]))
        texts[s][2].set_text(str(data[-1, s*3 + 2]))


def get_data():
    data = np.zeros((300, 6))
    yield data
    while True:
        for _ in range(50):
            w = input()
            v = [[float(i) for i in w.split()]]
            if len(data) >= 50:
                data = np.append(data[1:], v, 0)
            else:
                data = np.append(data, v, 0)
            print(w, flush=True)
        yield data


if __name__ == "__main__":
    fig, ax, plots, texts = init(next(get_data()))
    ani = animation.FuncAnimation(
        fig, lambda frame: animate(frame, ax, plots, texts), interval=200, frames=get_data)
    plt.tight_layout()
    plt.show()

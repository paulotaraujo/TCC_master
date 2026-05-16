#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import struct
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# desativa atalhos padrão do matplotlib
mpl.rcParams['keymap.back'] = []
mpl.rcParams['keymap.forward'] = []
mpl.rcParams['keymap.home'] = []


# -------------------------------------------------
# leitura do BDAT
# -------------------------------------------------

def read_bdat(path, channels=2, ts64=False):

    raw = path.read_bytes()

    header_size = 12 if ts64 else 8
    rec_size = header_size + channels * 2

    n_records = len(raw) // rec_size

    samples = np.zeros(n_records, dtype=np.uint32)
    timestamps = np.zeros(n_records, dtype=np.uint64 if ts64 else np.uint32)
    analog = np.zeros((channels, n_records), dtype=np.int16)

    offset = 0

    header_fmt = "<IQ" if ts64 else "<II"
    data_fmt = "<" + ("h"*channels)

    for i in range(n_records):

        sample, ts = struct.unpack_from(header_fmt, raw, offset)
        offset += header_size

        vals = struct.unpack_from(data_fmt, raw, offset)
        offset += channels*2

        samples[i] = sample
        timestamps[i] = ts

        for ch in range(channels):
            analog[ch, i] = vals[ch]

    return samples, timestamps, analog


# -------------------------------------------------
# eixo de tempo
# -------------------------------------------------

def build_time(samples, timestamps, fs, use_ts):

    if use_ts:
        t0 = timestamps[0]
        t = (timestamps - t0) / 1_000_000
    else:
        t = samples / fs

    return t


# -------------------------------------------------
# plot
# -------------------------------------------------

def plot_bdat(samples, timestamps, analog, fs, channel, use_ts):

    t = build_time(samples, timestamps, fs, use_ts)

    fig, ax = plt.subplots(figsize=(14,6))
    fig.subplots_adjust(right=0.78)

    lines = []

    if channel == "all":

        for i in range(analog.shape[0]):
            line, = ax.plot(t, analog[i], label=f"Canal {i+1}")
            lines.append((i,line))

    else:

        ch = int(channel)-1
        line, = ax.plot(t, analog[ch], label=f"Canal {channel}")
        lines.append((ch,line))

    ax.set_title("Desenvolvimento das amostras - BDAT")
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Amplitude")
    ax.grid(True)
    ax.legend()

    # linha vertical do cursor
    vline = ax.axvline(t[0], color="k", linestyle="--", alpha=0.6)

    # caixa de informação (fora do gráfico)
    info_box = fig.text(
        0.81, 0.9,
        "",
        ha="left",
        va="top",
        fontsize=10,
        family="monospace",
        bbox=dict(boxstyle="round", fc="white", ec="gray")
    )

    idx = [0]

    # -------------------------------------
    # atualização do cursor
    # -------------------------------------

    def update(i):

        i = max(0, min(i, len(t)-1))
        idx[0] = i

        x = t[i]

        vline.set_xdata([x,x])

        text = (
            f"Amostra: {samples[i]}\n"
            f"Tempo:   {x:.6f} s\n\n"
        )

        for ch,line in lines:

            y = line.get_ydata()[i]
            text += f"Canal {ch+1}: {y}\n"

        info_box.set_text(text)

        fig.canvas.draw_idle()


    # -------------------------------------
    # mouse
    # -------------------------------------

    def on_move(event):

        if event.inaxes != ax or event.xdata is None:
            return

        i = np.argmin(np.abs(t-event.xdata))
        update(i)

    fig.canvas.mpl_connect("motion_notify_event", on_move)


    # -------------------------------------
    # teclado
    # -------------------------------------

    def on_key(event):

        i = idx[0]

        if event.key == "right":
            i += 1

        elif event.key == "left":
            i -= 1

        elif event.key == "up":
            i += 10

        elif event.key == "down":
            i -= 10

        else:
            return

        update(i)

    fig.canvas.mpl_connect("key_press_event", on_key)

    update(0)

    plt.show()


# -------------------------------------------------
# main
# -------------------------------------------------

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument("file", type=Path)
    parser.add_argument("--channels", type=int, default=2)
    parser.add_argument("--sample-rate", type=float, default=2000)
    parser.add_argument("--ts64", action="store_true")
    parser.add_argument("--channel", default="all")
    parser.add_argument("--use-timestamps", action="store_true")

    args = parser.parse_args()

    samples, timestamps, analog = read_bdat(
        args.file,
        args.channels,
        args.ts64
    )

    print("Arquivo carregado:")
    print("Amostras:", len(samples))

    plot_bdat(
        samples,
        timestamps,
        analog,
        args.sample_rate,
        args.channel,
        args.use_timestamps
    )


if __name__ == "__main__":
    main()

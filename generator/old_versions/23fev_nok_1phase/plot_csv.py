#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import matplotlib.pyplot as plt
from pathlib import Path

CSV_PATH = Path("rx.csv")


def read_csv(path):
    t = []
    v = []
    i = []
    sample = []

    with open(path, "r", errors="ignore") as f:
        reader = csv.DictReader(f)

        for row in reader:
            try:
                sample.append(int(row.get("sample_idx", 0)))

                # usa tempo se existir
                if "time_pc_s" in row and row["time_pc_s"]:
                    t.append(float(row["time_pc_s"]))
                else:
                    t.append(len(t))

                # usa tensão em volts se existir
                if "adcV_V" in row and row["adcV_V"]:
                    v.append(float(row["adcV_V"]))
                else:
                    v.append(float(row["adcV_mV"]) / 1000.0)

                if "adcI_V" in row and row["adcI_V"]:
                    i.append(float(row["adcI_V"]))
                else:
                    i.append(float(row["adcI_mV"]) / 1000.0)

            except Exception:
                continue

    return t, v, i, sample


def plot_signals(t, v, i):
    plt.figure()

    plt.plot(t, v, label="Tensão (V)")
    plt.plot(t, i, label="Corrente (V)")

    plt.xlabel("Tempo (s)")
    plt.ylabel("Tensão (V)")
    plt.title("Sinal ADC capturado (ESP32)")
    plt.legend()
    plt.grid()

    plt.show()


def main():
    if not CSV_PATH.exists():
        print("Arquivo rx.csv não encontrado")
        return

    t, v, i, sample = read_csv(CSV_PATH)

    if len(t) == 0:
        print("CSV vazio")
        return

    print(f"Amostras carregadas: {len(t)}")

    plot_signals(t, v, i)


if __name__ == "__main__":
    main()

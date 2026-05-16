#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# =========================
# Config padrão
# =========================
VREF = 3.3     # tensão de referência
ADC_MAX = 4095 # 12 bits

# =========================
def load_csv(path):
    t = []
    adc = []

    with open(path, "r", errors="ignore") as f:
        reader = csv.reader(f)
        header = next(reader, None)

        for row in reader:
            if len(row) < 2:
                continue
            try:
                t.append(int(row[0]))
                adc.append(int(row[1]))
            except:
                continue

    return np.array(t), np.array(adc)

# =========================
def compute_fs(t):
    if len(t) < 2:
        return 0, 0

    dt = np.diff(t)
    dt = dt[dt > 0]

    if len(dt) == 0:
        return 0, 0

    dt_mean = np.mean(dt)
    fs = 1e6 / dt_mean

    return fs, dt_mean

# =========================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="rx.csv")
    ap.add_argument("--volts", action="store_true", help="converter para volts")
    ap.add_argument("--limit", type=int, default=0, help="limitar nº de amostras")
    args = ap.parse_args()

    path = Path(args.file)

    print(f"[INFO] Lendo {path}")
    t, adc = load_csv(path)

    if len(t) == 0:
        print("Arquivo vazio ou inválido.")
        return

    # limita amostras (pra não travar gráfico)
    if args.limit > 0:
        t = t[:args.limit]
        adc = adc[:args.limit]

    # normaliza tempo para começar em 0
    t0 = t[0]
    t = (t - t0) / 1e6  # segundos

    # calcula frequência de amostragem
    fs, dt_mean = compute_fs(t * 1e6)

    print(f"[INFO] Amostras: {len(t)}")
    print(f"[INFO] Fs estimada: {fs:.2f} Hz")
    print(f"[INFO] dt médio: {dt_mean:.1f} us")

    # converte para volts (opcional)
    if args.volts:
        y = (adc / ADC_MAX) * VREF
        ylabel = "Tensão (V)"
    else:
        y = adc
        ylabel = "ADC (0-4095)"

    # =========================
    # Plot
    # =========================
    plt.figure(figsize=(12, 6))
    plt.plot(t, y)

    plt.title("Sinal Capturado (ESP32)")
    plt.xlabel("Tempo (s)")
    plt.ylabel(ylabel)
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# =========================
if __name__ == "__main__":
    main()

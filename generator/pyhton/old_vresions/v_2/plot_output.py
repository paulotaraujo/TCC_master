#!/usr/bin/env python3
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv_flexible(path: Path):
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = set(reader.fieldnames or [])

        old_format = {"seq", "t_us", "dt_us", "dacV_code", "dacI_code"}
        new_format = {"tx_index", "dt_us", "dacV_code", "dacI_code"}

        rows = list(reader)

    if not rows:
        raise RuntimeError("Nenhuma linha encontrada no CSV.")

    # ---------- Formato novo ----------
    if new_format.issubset(fieldnames):
        tx_index = []
        t_cont_us = []
        dac_v = []
        dac_i = []

        t_accum = 0

        for row in rows:
            dt_us = int(row["dt_us"])
            t_accum += dt_us

            tx_index.append(int(row["tx_index"]))
            t_cont_us.append(t_accum)
            dac_v.append(int(row["dacV_code"]))
            dac_i.append(int(row["dacI_code"]))

        return tx_index, t_cont_us, dac_v, dac_i, "new"

    # ---------- Formato antigo ----------
    if old_format.issubset(fieldnames):
        tx_index = []
        t_cont_us = []
        dac_v = []
        dac_i = []

        t_accum = 0
        last_seq = None

        for i, row in enumerate(rows):
            seq = int(row["seq"])
            dt_us = int(row["dt_us"])

            # Se o seq reiniciar, significa novo ciclo.
            # Para manter o gráfico contínuo, só seguimos acumulando.
            if last_seq is not None and seq < last_seq:
                pass

            t_accum += dt_us

            tx_index.append(i)
            t_cont_us.append(t_accum)
            dac_v.append(int(row["dacV_code"]))
            dac_i.append(int(row["dacI_code"]))

            last_seq = seq

        return tx_index, t_cont_us, dac_v, dac_i, "old"

    raise RuntimeError(
        "CSV inválido. Formatos aceitos:\n"
        "  antigo: seq,t_us,dt_us,dacV_code,dacI_code\n"
        "  novo:   tx_index,dt_us,dacV_code,dacI_code"
    )


def plot_continuous(t_cont_us, dac_v, dac_i, label_suffix=""):
    t_s = [x / 1_000_000.0 for x in t_cont_us]

    plt.figure(figsize=(14, 6))
    plt.plot(t_s, dac_v, label=f"DAC V (dacV_code){label_suffix}")
    plt.plot(t_s, dac_i, label=f"DAC I (dacI_code){label_suffix}")
    plt.xlabel("Tempo contínuo (s)")
    plt.ylabel("Código DAC (0 a 4095)")
    plt.title("Amostras DAC - gráfico contínuo")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def main():
    if len(sys.argv) < 2:
        print("Uso:")
        print("  python plot_output.py generator_output.csv")
        sys.exit(1)

    path = Path(sys.argv[1])
    if not path.exists():
        raise FileNotFoundError(f"Arquivo não encontrado: {path}")

    tx_index, t_cont_us, dac_v, dac_i, fmt = read_csv_flexible(path)

    print(f"Formato detectado: {fmt}")
    print(f"Linhas lidas: {len(tx_index)}")
    print(f"Tempo contínuo total: {t_cont_us[-1] / 1_000_000.0:.6f} s")
    print(f"dacV_code: min={min(dac_v)} max={max(dac_v)}")
    print(f"dacI_code: min={min(dac_i)} max={max(dac_i)}")

    plot_continuous(t_cont_us, dac_v, dac_i)


if __name__ == "__main__":
    main()
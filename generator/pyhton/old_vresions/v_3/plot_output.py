#!/usr/bin/env python3
import csv
import sys
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt

# Desativa atalhos padrão do Matplotlib que podem conflitar com as setas
mpl.rcParams["keymap.back"] = []
mpl.rcParams["keymap.forward"] = []
mpl.rcParams["keymap.home"] = []


def read_csv_flexible(path: Path):
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = set(reader.fieldnames or [])
        rows = list(reader)

    if not rows:
        raise RuntimeError("Nenhuma linha encontrada no CSV.")

    # Formato usado no logger do PC
    old_format = {"seq", "t_us", "dt_us", "dacV_code", "dacI_code"}
    new_format = {"tx_index", "dt_us", "dacV_code", "dacI_code"}

    if new_format.issubset(fieldnames):
        sample_idx = []
        t_cont_us = []
        dac_v = []
        dac_i = []

        t_accum = 0
        for row in rows:
            dt_us = int(row["dt_us"])
            t_accum += dt_us

            sample_idx.append(int(row["tx_index"]))
            t_cont_us.append(t_accum)
            dac_v.append(int(row["dacV_code"]))
            dac_i.append(int(row["dacI_code"]))

        return sample_idx, t_cont_us, dac_v, dac_i, "new"

    if old_format.issubset(fieldnames):
        sample_idx = []
        t_cont_us = []
        dac_v = []
        dac_i = []

        t_accum = 0
        last_seq = None

        for i, row in enumerate(rows):
            seq = int(row["seq"])
            dt_us = int(row["dt_us"])

            if last_seq is not None and seq < last_seq:
                pass

            t_accum += dt_us

            sample_idx.append(i)
            t_cont_us.append(t_accum)
            dac_v.append(int(row["dacV_code"]))
            dac_i.append(int(row["dacI_code"]))

            last_seq = seq

        return sample_idx, t_cont_us, dac_v, dac_i, "old"

    raise RuntimeError(
        "CSV inválido. Formatos aceitos:\n"
        "  antigo: seq,t_us,dt_us,dacV_code,dacI_code\n"
        "  novo:   tx_index,dt_us,dacV_code,dacI_code"
    )


def plot_continuous(sample_idx, t_cont_us, dac_v, dac_i, label_suffix=""):
    t_s = [x / 1_000_000.0 for x in t_cont_us]

    fig, ax = plt.subplots(figsize=(15, 6))
    fig.subplots_adjust(right=0.78)  # reserva espaço à direita para o painel

    ax.plot(t_s, dac_v, label=f"DAC V (dacV_code){label_suffix}")
    ax.plot(t_s, dac_i, label=f"DAC I (dacI_code){label_suffix}")

    ax.set_xlabel("Tempo contínuo (s)")
    ax.set_ylabel("Código DAC (0 a 4095)")
    ax.set_title("Amostras DAC - gráfico contínuo")
    ax.grid(True)
    ax.legend()

    # Linhas do cursor
    vline = ax.axvline(t_s[0], linestyle="--", linewidth=1)
    hline = ax.axhline(dac_v[0], linestyle="--", linewidth=1)

    # Marcadores na amostra atual
    point_v, = ax.plot([t_s[0]], [dac_v[0]], marker="o", linestyle="None")
    point_i, = ax.plot([t_s[0]], [dac_i[0]], marker="o", linestyle="None")

    # Painel de informações fora da área do gráfico
    info_text = fig.text(
        0.80,
        0.80,
        "",
        va="top",
        ha="left",
        fontsize=10,
        family="monospace",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.9),
    )

    current_idx = 0

    def update_cursor(idx: int):
        nonlocal current_idx

        idx = max(0, min(idx, len(t_s) - 1))
        current_idx = idx

        x = t_s[idx]
        yv = dac_v[idx]
        yi = dac_i[idx]

        vline.set_xdata([x, x])
        hline.set_ydata([yv, yv])

        point_v.set_data([x], [yv])
        point_i.set_data([x], [yi])

        info_text.set_text(
            f"Amostra : {sample_idx[idx]}\n"
            f"Índice  : {idx}\n"
            f"Tempo   : {x:.6f} s\n"
            f"DAC V   : {yv}\n"
            f"DAC I   : {yi}"
        )

        fig.canvas.draw_idle()

    def nearest_index_from_event(event):
        if event.xdata is None:
            return None

        x = event.xdata
        best_idx = min(range(len(t_s)), key=lambda i: abs(t_s[i] - x))
        return best_idx

    def on_move(event):
        if event.inaxes != ax:
            return

        idx = nearest_index_from_event(event)
        if idx is None:
            return

        update_cursor(idx)

    def on_key(event):
        nonlocal current_idx

        if event.key == "right":
            update_cursor(current_idx + 1)
        elif event.key == "left":
            update_cursor(current_idx - 1)
        elif event.key == "home":
            update_cursor(0)
        elif event.key == "end":
            update_cursor(len(t_s) - 1)

    fig.canvas.mpl_connect("motion_notify_event", on_move)
    fig.canvas.mpl_connect("key_press_event", on_key)

    update_cursor(0)
    plt.show()


def main():
    if len(sys.argv) < 2:
        print("Uso:")
        print("  python plot_output.py generator_output.csv")
        sys.exit(1)

    path = Path(sys.argv[1])
    if not path.exists():
        raise FileNotFoundError(f"Arquivo não encontrado: {path}")

    sample_idx, t_cont_us, dac_v, dac_i, fmt = read_csv_flexible(path)

    print(f"Formato detectado: {fmt}")
    print(f"Linhas lidas: {len(sample_idx)}")
    print(f"Tempo contínuo total: {t_cont_us[-1] / 1_000_000.0:.6f} s")
    print(f"dacV_code: min={min(dac_v)} max={max(dac_v)}")
    print(f"dacI_code: min={min(dac_i)} max={max(dac_i)}")

    plot_continuous(sample_idx, t_cont_us, dac_v, dac_i)


if __name__ == "__main__":
    main()
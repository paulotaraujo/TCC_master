#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Plota o CSV gerado por generator_universal_with_tx_csv.py.

O script:
- lê o CSV de transmissão
- reconstrói o eixo de tempo a partir de dt_us
- plota os códigos DAC enviados
- opcionalmente plota também os valores reais de origem
- permite mostrar só alguns DACs

Uso:
    python3 plot_tx_csv.py tx_saida.csv
    python3 plot_tx_csv.py tx_saida.csv --codes-only
    python3 plot_tx_csv.py tx_saida.csv --dacs 0,1
    python3 plot_tx_csv.py tx_saida.csv --start 0.1 --end 0.3
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Sequence

import matplotlib.pyplot as plt


DAC_MID = 2048
DAC_MAX = 4095


def parse_dacs(text: str) -> List[int]:
    out = []
    for tok in text.split(","):
        tok = tok.strip()
        if not tok:
            continue
        idx = int(tok)
        if idx < 0 or idx > 3:
            raise ValueError("Os DACs devem estar entre 0 e 3.")
        out.append(idx)
    if not out:
        raise ValueError("Nenhum DAC válido informado.")
    return sorted(set(out))


def read_tx_csv(path: str) -> Dict[str, List[float]]:
    data: Dict[str, List[float]] = {
        "sample_number": [],
        "timestamp_ticks": [],
        "dt_us": [],
        "flags": [],
        "time_s": [],
    }

    for i in range(4):
        data[f"dac{i}_code"] = []
        data[f"dac{i}_real_value"] = []

    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)

        t = 0.0
        for row_idx, row in enumerate(reader):
            dt_us = float(row.get("dt_us", 0) or 0)

            if row_idx == 0:
                t = 0.0
            else:
                t += dt_us / 1_000_000.0

            data["sample_number"].append(float(row.get("sample_number", 0) or 0))
            data["timestamp_ticks"].append(float(row.get("timestamp_ticks", 0) or 0))
            data["dt_us"].append(dt_us)
            data["flags"].append(float(row.get("flags", 0) or 0))
            data["time_s"].append(t)

            for i in range(4):
                code_key = f"dac{i}_code"
                real_key = f"dac{i}_real_value"

                code_val = row.get(code_key, "")
                real_val = row.get(real_key, "")

                data[code_key].append(float(code_val) if code_val != "" else float("nan"))
                data[real_key].append(float(real_val) if real_val != "" else float("nan"))

    if not data["time_s"]:
        raise RuntimeError("CSV vazio ou sem linhas válidas.")

    return data


def apply_time_window(data: Dict[str, List[float]], start: float | None, end: float | None) -> Dict[str, List[float]]:
    times = data["time_s"]
    mask = []
    for t in times:
        keep = True
        if start is not None and t < start:
            keep = False
        if end is not None and t > end:
            keep = False
        mask.append(keep)

    out: Dict[str, List[float]] = {}
    for key, values in data.items():
        out[key] = [v for v, keep in zip(values, mask) if keep]

    if not out["time_s"]:
        raise RuntimeError("Nenhuma amostra dentro da janela de tempo selecionada.")

    return out


def plot_data(
    data: Dict[str, List[float]],
    dacs: Sequence[int],
    codes_only: bool,
    title: str,
) -> None:
    t = data["time_s"]

    if codes_only:
        fig, ax = plt.subplots(figsize=(12, 6))
        for i in dacs:
            ax.plot(t, data[f"dac{i}_code"], label=f"DAC{i} code")
        ax.axhline(DAC_MID, linestyle="--", linewidth=1, label="DAC mid")
        ax.set_title(title)
        ax.set_xlabel("Tempo (s)")
        ax.set_ylabel("Código DAC")
        ax.set_ylim(-50, DAC_MAX + 50)
        ax.grid(True, alpha=0.3)
        ax.legend()
        plt.tight_layout()
        plt.show()
        return

    fig1, ax1 = plt.subplots(figsize=(12, 6))
    for i in dacs:
        ax1.plot(t, data[f"dac{i}_code"], label=f"DAC{i} code")
    ax1.axhline(DAC_MID, linestyle="--", linewidth=1, label="DAC mid")
    ax1.set_title(title + " - códigos DAC")
    ax1.set_xlabel("Tempo (s)")
    ax1.set_ylabel("Código DAC")
    ax1.set_ylim(-50, DAC_MAX + 50)
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    plt.tight_layout()

    fig2, ax2 = plt.subplots(figsize=(12, 6))
    plotted_any = False
    for i in dacs:
        y = data[f"dac{i}_real_value"]
        if all(str(v) == "nan" for v in y):
            continue
        ax2.plot(t, y, label=f"DAC{i} valor real")
        plotted_any = True

    ax2.set_title(title + " - valores reais de origem")
    ax2.set_xlabel("Tempo (s)")
    ax2.set_ylabel("Grandeza original")
    ax2.grid(True, alpha=0.3)
    if plotted_any:
        ax2.legend()

    plt.tight_layout()
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plota o CSV de transmissão enviado à ESP32.")
    parser.add_argument("csv_path", help="arquivo CSV gerado com --tx-csv")
    parser.add_argument("--dacs", default="0,1,2,3", help="DACs para plotar, ex: 0,1")
    parser.add_argument("--codes-only", action="store_true", help="plota apenas os códigos DAC")
    parser.add_argument("--start", type=float, help="tempo inicial em segundos")
    parser.add_argument("--end", type=float, help="tempo final em segundos")
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        raise RuntimeError(f"Arquivo não encontrado: {csv_path}")

    dacs = parse_dacs(args.dacs)
    data = read_tx_csv(str(csv_path))
    data = apply_time_window(data, args.start, args.end)

    plot_data(
        data=data,
        dacs=dacs,
        codes_only=args.codes_only,
        title=f"CSV de transmissão: {csv_path.name}",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
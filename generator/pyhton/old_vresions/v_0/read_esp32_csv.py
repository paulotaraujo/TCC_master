#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt


def read_esp32_csv(path: str) -> Dict[str, List[float]]:
    data: Dict[str, List[float]] = {
        "sample_idx": [],
        "applied_us": [],
        "dt_us": [],
        "flags": [],
        "time_s": [],
        "dac0": [],
        "dac1": [],
        "dac2": [],
        "dac3": [],
        "adc34": [],
        "adc35": [],
    }

    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)

        prev_applied_us = None
        t_acc = 0.0

        for row in reader:
            sample_idx = float(row.get("sample_idx", 0) or 0)
            applied_us = float(row.get("applied_us", 0) or 0)
            dt_us = float(row.get("dt_us", 0) or 0)
            flags = float(row.get("flags", 0) or 0)
            dac0 = float(row.get("dac0", 0) or 0)
            dac1 = float(row.get("dac1", 0) or 0)
            dac2 = float(row.get("dac2", 0) or 0)
            dac3 = float(row.get("dac3", 0) or 0)
            adc34 = float(row.get("adc34", 0) or 0)
            adc35 = float(row.get("adc35", 0) or 0)

            if prev_applied_us is None:
                t_acc = 0.0
            else:
                delta_applied = applied_us - prev_applied_us
                if delta_applied > 0:
                    t_acc += delta_applied / 1_000_000.0
                else:
                    t_acc += dt_us / 1_000_000.0

            prev_applied_us = applied_us

            data["sample_idx"].append(sample_idx)
            data["applied_us"].append(applied_us)
            data["dt_us"].append(dt_us)
            data["flags"].append(flags)
            data["time_s"].append(t_acc)
            data["dac0"].append(dac0)
            data["dac1"].append(dac1)
            data["dac2"].append(dac2)
            data["dac3"].append(dac3)
            data["adc34"].append(adc34)
            data["adc35"].append(adc35)

    if not data["sample_idx"]:
        raise RuntimeError("CSV vazio ou sem amostras válidas.")

    return data


def apply_time_window(data: Dict[str, List[float]], start: float | None, end: float | None) -> Dict[str, List[float]]:
    out: Dict[str, List[float]] = {k: [] for k in data.keys()}

    for i, t in enumerate(data["time_s"]):
        if start is not None and t < start:
            continue
        if end is not None and t > end:
            continue
        for key in data.keys():
            out[key].append(data[key][i])

    if not out["sample_idx"]:
        raise RuntimeError("Nenhuma amostra dentro da janela selecionada.")

    return out


def print_summary(data: Dict[str, List[float]]) -> None:
    n = len(data["sample_idx"])
    duration = data["time_s"][-1] - data["time_s"][0] if n > 1 else 0.0
    mean_dt_us = sum(data["dt_us"][1:]) / max(1, len(data["dt_us"]) - 1) if n > 1 else 0.0

    print("=== CSV ESP32 ===")
    print(f"Amostras: {n}")
    print(f"Duração estimada: {duration:.6f} s")
    print(f"dt médio: {mean_dt_us:.3f} us")
    print(f"ADC34: min={min(data['adc34']):.1f} max={max(data['adc34']):.1f}")
    print(f"ADC35: min={min(data['adc35']):.1f} max={max(data['adc35']):.1f}")


def plot_data(data: Dict[str, List[float]], with_dac: bool, title: str) -> None:
    t = data["time_s"]

    fig1, ax1 = plt.subplots(figsize=(12, 6))
    ax1.plot(t, data["adc34"], label="ADC34")
    ax1.plot(t, data["adc35"], label="ADC35")
    ax1.set_title(title + " - ADCs")
    ax1.set_xlabel("Tempo (s)")
    ax1.set_ylabel("Código ADC")
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    plt.tight_layout()

    if with_dac:
        fig2, ax2 = plt.subplots(figsize=(12, 6))
        ax2.plot(t, data["dac0"], label="DAC0")
        ax2.plot(t, data["dac1"], label="DAC1")
        ax2.plot(t, data["dac2"], label="DAC2")
        ax2.plot(t, data["dac3"], label="DAC3")
        ax2.set_title(title + " - códigos DAC aplicados")
        ax2.set_xlabel("Tempo (s)")
        ax2.set_ylabel("Código DAC")
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        plt.tight_layout()

    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Lê e plota o CSV de output salvo pela ESP32.")
    parser.add_argument("csv_path", help="arquivo CSV salvo no SD pela ESP32")
    parser.add_argument("--with-dac", action="store_true", help="plota também os DACs aplicados")
    parser.add_argument("--start", type=float, help="tempo inicial em segundos")
    parser.add_argument("--end", type=float, help="tempo final em segundos")
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        raise RuntimeError(f"Arquivo não encontrado: {csv_path}")

    data = read_esp32_csv(str(csv_path))
    data = apply_time_window(data, args.start, args.end)
    print_summary(data)
    plot_data(data, with_dac=args.with_dac, title=f"Output ESP32: {csv_path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
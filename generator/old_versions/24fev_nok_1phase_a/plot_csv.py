#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib.pyplot as plt


def sniff_dialect(path: Path) -> csv.Dialect:
    sample = path.read_text(errors="ignore")[:4096]
    sniffer = csv.Sniffer()
    try:
        return sniffer.sniff(sample, delimiters=[",", ";", "\t"])
    except Exception:
        d = csv.excel
        d.delimiter = ","
        return d


def to_int(s: Optional[str], default: int = 0) -> int:
    if s is None:
        return default
    s = str(s).strip()
    if not s:
        return default
    # aceita "123", "123.0", "123,0"
    s = s.replace(",", ".")
    return int(float(s))


def to_float(s: Optional[str], default: float = float("nan")) -> float:
    if s is None:
        return default
    s = str(s).strip()
    if not s:
        return default
    s = s.replace(",", ".")
    return float(s)


def dac12_to_volts(code: int, vref: float = 3.3) -> float:
    # 12-bit: 0..4095
    if code < 0:
        code = 0
    if code > 4095:
        code = 4095
    return (code / 4095.0) * vref


def load_csv(path: Path) -> Dict[str, List[float]]:
    d = sniff_dialect(path)
    cols: Dict[str, List[float]] = {
        "sample": [],
        "t_us": [],
        "dt_us": [],
        "dacV_code": [],
        "dacI_code": [],
        "adcV_raw": [],
        "adcI_raw": [],
        "adcV_V": [],
        "adcI_V": [],
        "clipV": [],
        "clipI": [],
    }

    with path.open("r", newline="", errors="ignore") as f:
        reader = csv.DictReader(f, dialect=d)
        if reader.fieldnames is None:
            raise ValueError("CSV sem cabeçalho (fieldnames).")

        # normaliza nomes (caso tenha espaços)
        fieldmap = {name.strip(): name for name in reader.fieldnames}

        def get(row, key):
            # tenta key exata, senão procura variante com strip
            if key in row:
                return row.get(key)
            if key in fieldmap:
                return row.get(fieldmap[key])
            return None

        for row in reader:
            cols["sample"].append(float(to_int(get(row, "sample"))))
            cols["t_us"].append(float(to_int(get(row, "t_us"))))
            cols["dt_us"].append(float(to_int(get(row, "dt_us"))))
            cols["dacV_code"].append(float(to_int(get(row, "dacV_code"))))
            cols["dacI_code"].append(float(to_int(get(row, "dacI_code"))))
            cols["adcV_raw"].append(float(to_int(get(row, "adcV_raw"))))
            cols["adcI_raw"].append(float(to_int(get(row, "adcI_raw"))))
            cols["adcV_V"].append(to_float(get(row, "adcV_V")))
            cols["adcI_V"].append(to_float(get(row, "adcI_V")))
            cols["clipV"].append(float(to_int(get(row, "clipV"))))
            cols["clipI"].append(float(to_int(get(row, "clipI"))))

    return cols


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_path", type=Path, help="CSV gerado pela ESP32 (output.csv)")
    ap.add_argument("--vref", type=float, default=3.3, help="Vref do DAC (default 3.3V)")
    ap.add_argument("--skip", type=int, default=0, help="pular N amostras iniciais (debug)")
    ap.add_argument("--max", type=int, default=0, help="plotar no máximo N amostras (0 = todas)")
    ap.add_argument("--no-dac", action="store_true", help="não plotar curvas do DAC (somente ADC)")
    ap.add_argument("--show-clips", action="store_true", help="marcar pontos de clipping")
    args = ap.parse_args()

    data = load_csv(args.csv_path)

    n = len(data["t_us"])
    if n == 0:
        raise SystemExit("CSV vazio.")

    start = max(0, args.skip)
    end = n if args.max <= 0 else min(n, start + args.max)

    t_us = data["t_us"][start:end]
    t0 = t_us[0]
    t_s = [(x - t0) / 1_000_000.0 for x in t_us]

    adcV = data["adcV_V"][start:end]
    adcI = data["adcI_V"][start:end]

    dacV_code = [int(x) for x in data["dacV_code"][start:end]]
    dacI_code = [int(x) for x in data["dacI_code"][start:end]]
    dacV = [dac12_to_volts(c, args.vref) for c in dacV_code]
    dacI = [dac12_to_volts(c, args.vref) for c in dacI_code]

    clipV = [int(x) for x in data["clipV"][start:end]]
    clipI = [int(x) for x in data["clipI"][start:end]]

    # ===== Plot 1: Tensão =====
    plt.figure()
    plt.title("Canal V (Tensão) — DAC vs ADC")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Volts (V)")
    if not args.no_dac:
        plt.plot(t_s, dacV, label="DAC V (code->V)")
    plt.plot(t_s, adcV, label="ADC V (medido)")
    if args.show_clips:
        idx = [i for i, c in enumerate(clipV) if c == 1]
        if idx:
            plt.scatter([t_s[i] for i in idx], [adcV[i] for i in idx], label="clipV=1", marker="x")
    plt.grid(True)
    plt.legend()

    # ===== Plot 2: Corrente (canal I) =====
    plt.figure()
    plt.title("Canal I (Corrente) — DAC vs ADC")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Volts (V)")
    if not args.no_dac:
        plt.plot(t_s, dacI, label="DAC I (code->V)")
    plt.plot(t_s, adcI, label="ADC I (medido)")
    if args.show_clips:
        idx = [i for i, c in enumerate(clipI) if c == 1]
        if idx:
            plt.scatter([t_s[i] for i in idx], [adcI[i] for i in idx], label="clipI=1", marker="x")
    plt.grid(True)
    plt.legend()

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
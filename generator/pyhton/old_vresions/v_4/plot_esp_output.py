#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


DAC_BITS = 12
DAC_MAX_CODE = (1 << DAC_BITS) - 1
DAC_VREF = 3.3


def code_to_volts(code: int, vref: float = DAC_VREF) -> float:
    return (float(code) / DAC_MAX_CODE) * vref


def read_esp_csv(path: Path):
    rows = []

    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)

        required = {
            "seq",
            "planned_t_us",
            "planned_dt_us",
            "actual_t_us",
            "late_us",
            "dacV_code",
            "dacI_code",
            "adcV_raw",
            "adcI_raw",
            "adcV_V",
            "adcI_V",
            "flags",
            "buffer_level",
        }

        missing = required - set(reader.fieldnames or [])
        if missing:
            raise RuntimeError(f"CSV inválido. Colunas ausentes: {sorted(missing)}")

        for row in reader:
            rows.append(
                {
                    "seq": int(row["seq"]),
                    "planned_t_us": int(row["planned_t_us"]),
                    "planned_dt_us": int(row["planned_dt_us"]),
                    "actual_t_us": int(row["actual_t_us"]),
                    "late_us": int(row["late_us"]),
                    "dacV_code": int(row["dacV_code"]),
                    "dacI_code": int(row["dacI_code"]),
                    "adcV_raw": int(row["adcV_raw"]),
                    "adcI_raw": int(row["adcI_raw"]),
                    "adcV_V": float(row["adcV_V"]),
                    "adcI_V": float(row["adcI_V"]),
                    "flags": int(row["flags"]),
                    "buffer_level": int(row["buffer_level"]),
                }
            )

    if not rows:
        raise RuntimeError("Nenhuma linha de dados foi lida do CSV.")

    return rows


def build_time_axis(rows, mode: str):
    if mode == "planned":
        t0 = rows[0]["planned_t_us"]
        return [(r["planned_t_us"] - t0) / 1e6 for r in rows]

    if mode == "actual":
        t0 = rows[0]["actual_t_us"]
        return [(r["actual_t_us"] - t0) / 1e6 for r in rows]

    raise ValueError(f"Modo de tempo inválido: {mode}")


def plot_series(x, y, title, xlabel, ylabel):
    plt.figure(figsize=(12, 5))
    plt.plot(x, y)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser(
        description="Lê o CSV de saída da ESP32 e plota tempo x amplitude dos DACs/ADCs."
    )
    parser.add_argument("csv_file", help="Caminho do arquivo esp32_output.csv")
    parser.add_argument(
        "--time-mode",
        choices=["planned", "actual"],
        default="planned",
        help="Usa planned_t_us ou actual_t_us como eixo do tempo",
    )
    parser.add_argument(
        "--show-dac-codes",
        action="store_true",
        help="Plota também os códigos brutos dos DACs",
    )
    parser.add_argument(
        "--show-adc",
        action="store_true",
        help="Plota também os sinais ADC medidos pela ESP32",
    )
    parser.add_argument(
        "--show-late",
        action="store_true",
        help="Plota também o atraso late_us",
    )
    args = parser.parse_args()

    path = Path(args.csv_file)
    if not path.exists():
        raise SystemExit(f"Arquivo não encontrado: {path}")

    rows = read_esp_csv(path)
    t = build_time_axis(rows, args.time_mode)

    dac_v_code = [r["dacV_code"] for r in rows]
    dac_i_code = [r["dacI_code"] for r in rows]

    dac_v_volts = [code_to_volts(v) for v in dac_v_code]
    dac_i_volts = [code_to_volts(v) for v in dac_i_code]

    adc_v_volts = [r["adcV_V"] for r in rows]
    adc_i_volts = [r["adcI_V"] for r in rows]

    late_us = [r["late_us"] for r in rows]
    buffer_level = [r["buffer_level"] for r in rows]

    xlabel = f"Tempo ({args.time_mode}) [s]"

    if args.show_dac_codes:
        plot_series(
            t,
            dac_v_code,
            "DAC V - Código bruto",
            xlabel,
            "Código DAC",
        )
        plot_series(
            t,
            dac_i_code,
            "DAC I - Código bruto",
            xlabel,
            "Código DAC",
        )

    plot_series(
        t,
        dac_v_volts,
        "DAC V - Tensão de saída estimada",
        xlabel,
        "Tensão [V]",
    )
    plot_series(
        t,
        dac_i_volts,
        "DAC I - Tensão de saída estimada",
        xlabel,
        "Tensão [V]",
    )

    if args.show_adc:
        plot_series(
            t,
            adc_v_volts,
            "ADC V - Tensão medida",
            xlabel,
            "Tensão [V]",
        )
        plot_series(
            t,
            adc_i_volts,
            "ADC I - Tensão medida",
            xlabel,
            "Tensão [V]",
        )

    if args.show_late:
        plot_series(
            t,
            late_us,
            "Atraso de reprodução (late_us)",
            xlabel,
            "late_us [us]",
        )

    plot_series(
        t,
        buffer_level,
        "Nível do buffer durante a reprodução",
        xlabel,
        "buffer_level",
    )

    plt.show()


if __name__ == "__main__":
    main()
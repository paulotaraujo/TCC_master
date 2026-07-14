#!/usr/bin/env python3
"""Dump the ESP32 embedded receiver RAM log to CSV.

The firmware keeps logging out of the protection critical path. This helper asks
for a serial dump only when the user wants to validate/plot the captured data.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import serial


CSV_PREFIX = "dev_t_s,"


def read_line(ser: serial.Serial, timeout: float) -> str | None:
    old_timeout = ser.timeout
    ser.timeout = timeout
    try:
        raw = ser.readline()
    finally:
        ser.timeout = old_timeout
    if not raw:
        return None
    text = raw.decode(errors="ignore").strip()
    return text or None


def main() -> None:
    parser = argparse.ArgumentParser(description="Exporta o log RAM da ESP32 receptora embarcada.")
    parser.add_argument("--port", required=True, help="Porta serial, ex.: /dev/ttyUSB1")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--out", default="embedded_capture.csv", help="CSV de saída")
    parser.add_argument("--timeout", type=float, default=30.0, help="Timeout do dump em segundos")
    parser.add_argument("--stop-first", action="store_true", help="Envia stop antes do dump")
    parser.add_argument("--clear-after", action="store_true", help="Limpa o log na ESP32 após dump concluído")
    args = parser.parse_args()

    out_path = Path(args.out)
    csv_lines: list[str] = []

    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=2.0) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        if args.stop_first:
            ser.write(b"stop\n")
            ser.flush()
            time.sleep(0.1)

        ser.write(b"dump\n")
        ser.flush()

        deadline = time.monotonic() + args.timeout
        done = False
        while time.monotonic() < deadline:
            line = read_line(ser, timeout=0.5)
            if not line:
                continue
            if line.startswith(CSV_PREFIX) or (csv_lines and line[0].isdigit()):
                csv_lines.append(line)
                continue
            print(f"[ESP32] {line}")
            if line.startswith("DUMP_DONE"):
                done = True
                break

        if not done:
            raise SystemExit("Timeout esperando DUMP_DONE da ESP32.")

        if args.clear_after:
            ser.write(b"clear\n")
            ser.flush()

    if not csv_lines:
        raise SystemExit("A ESP32 não retornou linhas CSV.")

    out_path.write_text("\n".join(csv_lines) + "\n", encoding="utf-8")
    print(f"CSV salvo em: {out_path} ({len(csv_lines) - 1} linhas de dados)")


if __name__ == "__main__":
    main()

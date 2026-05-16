#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
import re
import struct
import time
from pathlib import Path

import serial


def split_csv(line: str) -> list[str]:
    return [x.strip() for x in line.strip().split(",")]


def parse_cfg_2ch(cfg_path: Path) -> tuple[float, float, float, float, float, float, float]:
    """
    Retorna: fs, freq, timeMult, aV, bV, aI, bI
    Pegando os 2 primeiros canais analógicos.
    """
    lines = [ln.strip() for ln in cfg_path.read_text(errors="ignore").splitlines() if ln.strip()]
    if len(lines) < 10:
        raise ValueError("CFG muito curto.")

    # Linha 2: TT,NA,ND (ex: 2,2A,0D)
    parts = split_csv(lines[1])
    if len(parts) < 3:
        raise ValueError("Linha 2 inválida no CFG.")

    # NA está em parts[1] (pode vir "2A")
    m = re.match(r"^\s*(\d+)", parts[1])
    if not m:
        raise ValueError("NA inválido no CFG.")
    nA = int(m.group(1))
    if nA < 2:
        raise ValueError("Preciso de pelo menos 2 canais analógicos (V e I).")

    # Linhas analógicas começam em lines[2]
    # Formato: idx,name,phase,ccbm,unit,a,b,...
    def get_ab(i: int) -> tuple[float, float]:
        p = split_csv(lines[2 + i])
        if len(p) < 7:
            raise ValueError(f"Linha analógica {i+1} incompleta.")
        a = float(p[5].replace(",", "."))
        b = float(p[6].replace(",", "."))
        return a, b

    aV, bV = get_ab(0)
    aI, bI = get_ab(1)

    # freq está após analógicos + digitais:
    # vamos achar a primeira linha "freq" logo depois desses blocos
    # ND em parts[2] (pode vir "0D")
    m2 = re.match(r"^\s*(\d+)", parts[2])
    nD = int(m2.group(1)) if m2 else 0
    freq_line_idx = 2 + nA + nD
    freq = float(lines[freq_line_idx].split(",")[0].replace(",", "."))

    # nrates e primeira taxa
    nrates = int(lines[freq_line_idx + 1].split(",")[0])
    if nrates < 1:
        raise ValueError("nrates inválido.")
    fs = float(split_csv(lines[freq_line_idx + 2])[0].replace(",", "."))

    # timeMult geralmente é a última linha numérica do arquivo
    timeMult = 1.0
    for ln in reversed(lines):
        up = ln.upper()
        if up in ("ASCII", "BINARY", "BINARY32", "FLOAT32"):
            continue
        try:
            timeMult = float(split_csv(ln)[0].replace(",", "."))
            break
        except Exception:
            pass

    return fs, freq, timeMult, aV, bV, aI, bI


def iter_bdat_binary_2ch(bdat_path: Path, nA: int = 2):
    """
    Lê COMTRADE BINARY (int16) assumindo:
      sample(int32) + time(int32) + nA*int16 + (sem digitais)
    Retorna (sample, rawV, rawI)
    """
    rec_size = 4 + 4 + (2 * nA)
    head = struct.Struct("<ii")
    analog = struct.Struct("<" + "h" * nA)

    with bdat_path.open("rb") as f:
        while True:
            chunk = f.read(rec_size)
            if not chunk:
                break
            if len(chunk) != rec_size:
                raise ValueError("Registro truncado no BDAT.")
            sample, _t = head.unpack_from(chunk, 0)
            a = analog.unpack_from(chunk, 8)
            yield sample, int(a[0]), int(a[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", type=Path, required=True)
    ap.add_argument("--bdat", type=Path, required=True)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--throttle", type=float, default=0.0002, help="sleep por linha (s)")
    ap.add_argument("--progress-every", type=int, default=200)
    args = ap.parse_args()

    fs, freq, timeMult, aV, bV, aI, bI = parse_cfg_2ch(args.cfg)

    cfg_line = f"CFG,{fs:.6f},{freq:.6f},{timeMult:.6f},{aV:.12g},{bV:.12g},{aI:.12g},{bI:.12g}"

    print(f"Conectando em {args.port} @ {args.baud} ...")
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(1.2)

        ser.write((cfg_line + "\n").encode("ascii"))
        time.sleep(args.throttle)

        sent = 0
        for sample, rv, ri in iter_bdat_binary_2ch(args.bdat, nA=2):
            ser.write(f"S,{sample},{rv},{ri}\n".encode("ascii"))
            sent += 1
            if args.throttle > 0:
                time.sleep(args.throttle)
            if args.progress_every and (sent % args.progress_every == 0):
                print(f"... {sent} (out_waiting={ser.out_waiting})")

        ser.write(b"END\n")
        ser.flush()

    print("✅ Envio finalizado. A ESP32 vai salvar /comtrade_binary/output.csv no SD.")


if __name__ == "__main__":
    main()
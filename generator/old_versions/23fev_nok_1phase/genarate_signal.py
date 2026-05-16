#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import struct
import time
from pathlib import Path
import serial

PORT = "/dev/ttyUSB0"
BAUD = 921600

CFG_PATH  = Path("export.cfg")
BDAT_PATH = Path("export.bdat")
OUT_CSV   = Path("rx.csv")

V_MID = 1.65
V_AMP = 1.55
NOM_CYCLES = 5
V_FAULT_LIMIT_MULT = 2.0
I_FAULT_LIMIT_MULT = 3.0

SYNC1 = 0xA5
SYNC2 = 0x5A
STOP_BIN = b"\x55\xAA\xFF\x00"


def clamp12(x: int) -> int:
    return 0 if x < 0 else (4095 if x > 4095 else x)

def volts_to_dac12(v: float) -> int:
    v = max(0.0, min(3.3, v))
    return clamp12(int(round((v / 3.3) * 4095.0)))

def map_eng_to_volts_clip(eng: float, clip_peak: float) -> float:
    clip_peak = max(1e-9, clip_peak)
    eng = max(-clip_peak, min(clip_peak, eng))
    x = eng / clip_peak
    return V_MID + x * V_AMP

def parse_channel_count(token: str) -> int:
    s = str(token).strip().upper()
    num = ""
    for c in s:
        if c.isdigit():
            num += c
        else:
            break
    if not num:
        raise ValueError(f"Formato inválido de contagem de canais: {token!r}")
    return int(num)

def read_cfg_minimal(cfg_path: Path):
    lines = cfg_path.read_text(errors="ignore").splitlines()
    it = iter(lines)

    next(it, None)
    tt = next(it, "")
    parts = [p.strip() for p in tt.split(",")]
    if len(parts) < 3:
        raise RuntimeError("Linha 2 do CFG inválida (contagem de canais).")

    nAnalog = parse_channel_count(parts[1])
    nDigital = parse_channel_count(parts[2])

    ch = []
    for _ in range(nAnalog):
        row = next(it, "")
        p = [x.strip() for x in row.split(",")]
        a = float(p[5].replace(",", "."))
        b = float(p[6].replace(",", "."))
        ch.append((a, b))

    for _ in range(nDigital):
        next(it, None)

    freq = float(next(it, "60").strip().replace(",", ".") or "60")

    nrates = int(next(it, "1").strip() or "1")
    fs = 0.0
    if nrates > 0:
        r0 = next(it, "0").strip()
        fs = float(r0.split(",")[0].strip().replace(",", "."))
        for _ in range(nrates - 1):
            next(it, None)

    next(it, None)
    next(it, None)

    data_format = next(it, "BINARY").strip()
    time_mult = float(next(it, "1").strip().replace(",", ".") or "1")

    (aV, bV) = ch[0] if len(ch) > 0 else (1.0, 0.0)
    (aI, bI) = ch[1] if len(ch) > 1 else (1.0, 0.0)

    return {
        "nAnalog": nAnalog,
        "nDigital": nDigital,
        "freqHz": freq,
        "fsHz": fs,
        "format": data_format,
        "timeMult": time_mult,
        "aV": aV, "bV": bV,
        "aI": aI, "bI": bI,
    }

def detect_layout(file_size: int, nAnalog: int, nDigital: int):
    digital_words = (nDigital + 15) // 16
    digital_bytes = digital_words * 2

    rec32 = 4 + 4 + (nAnalog * 2) + digital_bytes
    if rec32 > 0 and file_size % rec32 == 0:
        return {"ts64": False, "recSize": rec32, "digitalBytes": digital_bytes, "total": file_size // rec32}

    rec64 = 4 + 8 + (nAnalog * 2) + digital_bytes
    if rec64 > 0 and file_size % rec64 == 0:
        return {"ts64": True, "recSize": rec64, "digitalBytes": digital_bytes, "total": file_size // rec64}

    raise RuntimeError("Não consegui detectar BINARY 32/64 pelo tamanho do BDAT.")

def compute_nominal_peaks(cfg, layout):
    fs = cfg["fsHz"]
    f = cfg["freqHz"]
    if fs <= 1.0 or f <= 1.0:
        raise RuntimeError("fs/f inválidos no CFG.")

    samples_per_cycle = max(8, int(round(fs / f)))
    N = min(layout["total"], samples_per_cycle * NOM_CYCLES)

    recSize = layout["recSize"]
    ts64 = layout["ts64"]
    digB = layout["digitalBytes"]
    nAnalog = cfg["nAnalog"]

    aV, bV = cfg["aV"], cfg["bV"]
    aI, bI = cfg["aI"], cfg["bI"]

    sumPeakV = 0.0
    sumPeakI = 0.0
    cycles = 0
    peakV = 0.0
    peakI = 0.0
    idx = 0

    with BDAT_PATH.open("rb") as fbin:
        for _ in range(N):
            rec = fbin.read(recSize)
            if len(rec) != recSize:
                break

            off = 4 + (8 if ts64 else 4)

            engV = 0.0
            engI = 0.0
            for chn in range(nAnalog):
                raw = struct.unpack_from("<h", rec, off)[0]
                off += 2
                if chn == 0:
                    engV = aV * raw + bV
                elif chn == 1:
                    engI = aI * raw + bI
            off += digB

            peakV = max(peakV, abs(engV))
            peakI = max(peakI, abs(engI))

            idx += 1
            if idx >= samples_per_cycle:
                sumPeakV += peakV
                sumPeakI += peakI
                cycles += 1
                peakV = 0.0
                peakI = 0.0
                idx = 0

    if cycles == 0:
        raise RuntimeError("Falha ao estimar nominal (ciclos=0).")

    v_nom = max(1.0, sumPeakV / cycles)
    i_nom = max(1.0, sumPeakI / cycles)

    v_clip = v_nom * V_FAULT_LIMIT_MULT
    i_clip = i_nom * I_FAULT_LIMIT_MULT
    return v_nom, i_nom, v_clip, i_clip

def read_exact(ser: serial.Serial, n: int, buf: bytearray) -> bytes:
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    out = bytes(buf[:n])
    del buf[:n]
    return out

def wait_for_ok_stop(ser: serial.Serial, timeout_s: float = 10.0) -> bytes:
    buf = bytearray()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
            if b"OK_STOP" in buf:
                return bytes(buf)
    return bytes(buf)

def wait_for_dump_and_save(ser: serial.Serial, out_csv: Path, timeout_s: float = 40.0):
    buf = bytearray()
    deadline = time.time() + timeout_s

    while True:
        if time.time() > deadline:
            raise TimeoutError("Não apareceu 'DUMP'.")

        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)

        i = buf.find(b"DUMP")
        if i >= 0:
            del buf[:i]
            break

    hdr = read_exact(ser, 12, buf)
    total = struct.unpack_from("<I", hdr, 4)[0]
    print(f"✅ DUMP total_samples = {total}")

    data_bytes = total * 4
    data = bytearray()
    while len(data) < data_bytes:
        chunk = ser.read(min(65536, data_bytes - len(data)))
        if chunk:
            data.extend(chunk)

    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sample_idx", "adcV_mV", "adcI_mV", "adcV_V", "adcI_V"])
        for k in range(total):
            off = k * 4
            v_mv = data[off + 0] | (data[off + 1] << 8)
            i_mv = data[off + 2] | (data[off + 3] << 8)
            w.writerow([k, v_mv, i_mv, f"{v_mv/1000.0:.6f}", f"{i_mv/1000.0:.6f}"])

    print(f"✅ CSV salvo: {out_csv.resolve()}")

def stream_comtrade_to_esp(ser: serial.Serial, cfg, layout, v_clip, i_clip) -> int:
    recSize = layout["recSize"]
    ts64 = layout["ts64"]
    digB = layout["digitalBytes"]
    nAnalog = cfg["nAnalog"]
    timeMult = cfg["timeMult"]
    fs = cfg["fsHz"]

    aV, bV = cfg["aV"], cfg["bV"]
    aI, bI = cfg["aI"], cfg["bI"]

    prev_ts = None
    total = layout["total"]

    MAX_OUT_WAITING = 2048
    dt_us_total = 0

    print(f"▶️ Enviando {total} amostras para ESP32 (com throttle)...")

    with BDAT_PATH.open("rb") as fbin:
        for idx in range(total):
            rec = fbin.read(recSize)
            if len(rec) != recSize:
                break

            if not ts64:
                ts = struct.unpack_from("<i", rec, 4)[0]
                off = 8
            else:
                ts = struct.unpack_from("<q", rec, 4)[0]
                off = 12

            if prev_ts is None:
                dt_us = 0
            else:
                dts = ts - prev_ts
                if dts < 0:
                    dts = 0
                dt_us = int(round(float(dts) * timeMult))
                if dt_us == 0 and fs > 0.1:
                    dt_us = int(round(1_000_000.0 / fs))
                dt_us = min(65535, max(0, dt_us))
            prev_ts = ts
            dt_us_total += dt_us

            engV = 0.0
            engI = 0.0
            for chn in range(nAnalog):
                raw = struct.unpack_from("<h", rec, off)[0]
                off += 2
                if chn == 0:
                    engV = aV * raw + bV
                elif chn == 1:
                    engI = aI * raw + bI
            off += digB

            vOut = map_eng_to_volts_clip(engV, v_clip)
            iOut = map_eng_to_volts_clip(engI, i_clip)
            dacV = volts_to_dac12(vOut)
            dacI = volts_to_dac12(iOut)

            frame = struct.pack("<BBHHH", SYNC1, SYNC2, dt_us, dacV, dacI)
            ser.write(frame)

            while ser.out_waiting > MAX_OUT_WAITING:
                time.sleep(0.001)

            if total > 0 and (idx % max(1, total // 20)) == 0:
                print(f"  ... {idx}/{total}  (out_waiting={ser.out_waiting})")

    ser.flush()
    print("✅ Stream finalizado.")
    return dt_us_total

def main():
    cfg = read_cfg_minimal(CFG_PATH)
    if cfg["format"].strip().upper() != "BINARY":
        raise RuntimeError("Este script está preparado para COMTRADE BINARY (export.bdat).")

    size = BDAT_PATH.stat().st_size
    layout = detect_layout(size, cfg["nAnalog"], cfg["nDigital"])
    v_nom, i_nom, v_clip, i_clip = compute_nominal_peaks(cfg, layout)

    print(f"CFG: nA={cfg['nAnalog']} nD={cfg['nDigital']} fs={cfg['fsHz']:.2f}Hz f={cfg['freqHz']:.2f}Hz timeMult={cfg['timeMult']}")
    print(f"Nominal({NOM_CYCLES} ciclos): V_nom_peak={v_nom:.6f} I_nom_peak={i_nom:.6f}")
    print(f"Limites: V_clip={v_clip:.6f} ({V_FAULT_LIMIT_MULT}x)  I_clip={i_clip:.6f} ({I_FAULT_LIMIT_MULT}x)")
    print(f"Layout: ts64={layout['ts64']} recSize={layout['recSize']} total={layout['total']}")

    with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
        time.sleep(0.7)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        dt_us_total = stream_comtrade_to_esp(ser, cfg, layout, v_clip, i_clip)

        playback_s_real = dt_us_total / 1_000_000.0
        margin = 2.0
        wait_s = playback_s_real + margin
        print(f"Playback real (soma dt_us): {playback_s_real:.3f}s | Aguardando ~{wait_s:.2f}s antes do STOP...")
        time.sleep(max(0.2, wait_s))

        print("Enviando STOP binário (20x)...")
        for _ in range(20):
            ser.write(STOP_BIN)
            ser.flush()
            time.sleep(0.02)

        print("Aguardando OK_STOP...")
        rx = wait_for_ok_stop(ser, timeout_s=10.0)
        txt = rx.decode(errors="ignore").strip()
        print("RX após STOP (texto):", txt)

        if b"OK_STOP" not in rx:
            raise RuntimeError("Ainda não recebi OK_STOP. A ESP32 não detectou STOP (provável STOP ainda na fila ou firmware diferente).")

        wait_for_dump_and_save(ser, OUT_CSV, timeout_s=40.0)

if __name__ == "__main__":
    main()
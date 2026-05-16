#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COMTRADE -> ESP32/MCP4728 streamer

Arquitetura:
- Python faz TODO o trabalho de alto nível:
  * parse do .cfg
  * leitura do .dat/.bdat
  * conversão de amostras analógicas para grandeza real (a*x+b)
  * cálculo de tempo entre amostras
  * escolha de até 4 canais para o MCP4728
  * mapeamento de amplitude para 0..4095
  * envio binário para a ESP32
  * gravação opcional em CSV do que foi enviado à ESP32

- ESP32 faz SOMENTE:
  * receber frames
  * esperar dt_us
  * escrever no MCP4728
"""

from __future__ import annotations

import argparse
import csv
import os
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Iterator, List, Optional, Sequence, Tuple

try:
    import serial
except ImportError:
    print("ERRO: pyserial não está instalado. Rode: pip install pyserial", file=sys.stderr)
    raise

FRAME_SYNC = 0xA5
FRAME_END = 0x5A
FLAG_IDLE_AFTER = 0x01
FLAG_RESET_CLOCK = 0x02
DAC_MAX = 4095
DAC_MID = 2048
DEFAULT_BAUD = 921600


@dataclass
class AnalogChannel:
    index_1based: int
    name: str
    phase: str
    ccbm: str
    unit: str
    a: float
    b: float
    skew: float
    min_raw: float
    max_raw: float
    primary: float
    secondary: float
    ps: str

    def apply(self, raw_value: float) -> float:
        return self.a * raw_value + self.b


@dataclass
class DigitalChannel:
    index_1based: int
    name: str
    phase: str
    ccbm: str
    y: int


@dataclass
class SampleRateSegment:
    samples_per_second: float
    end_sample: int


@dataclass
class ComtradeCfg:
    station_name: str
    rec_dev_id: str
    rev_year: str
    analog_channels: List[AnalogChannel] = field(default_factory=list)
    digital_channels: List[DigitalChannel] = field(default_factory=list)
    system_frequency_hz: float = 60.0
    sample_rates: List[SampleRateSegment] = field(default_factory=list)
    start_timestamp_str: str = ""
    trigger_timestamp_str: str = ""
    data_format: str = "BINARY"
    timemult: float = 1.0
    time_code: Optional[str] = None
    local_code: Optional[str] = None
    tmq_code: Optional[str] = None
    leap_second: Optional[str] = None

    @property
    def total_samples(self) -> int:
        return self.sample_rates[-1].end_sample if self.sample_rates else 0


@dataclass
class SampleRecord:
    sample_number: int
    timestamp_ticks: int
    analog_real: List[float]
    digital_bits: List[int]


def _read_cfg_lines(path: str) -> List[str]:
    with open(path, "r", encoding="utf-8-sig", errors="ignore") as f:
        lines = [ln.strip() for ln in f if ln.strip()]
    if not lines:
        raise RuntimeError(f"CFG vazio: {path}")
    return lines


def parse_cfg(cfg_path: str) -> ComtradeCfg:
    lines = _read_cfg_lines(cfg_path)
    pos = 0

    hdr = [x.strip() for x in lines[pos].split(",")]
    pos += 1
    if len(hdr) < 2:
        raise RuntimeError("Primeira linha do CFG inválida.")
    station_name = hdr[0]
    rec_dev_id = hdr[1]
    rev_year = hdr[2] if len(hdr) >= 3 else "1991"

    counts = [x.strip() for x in lines[pos].split(",")]
    pos += 1
    if len(counts) < 2:
        raise RuntimeError("Linha de contagem de canais inválida.")
    total_channels = int(counts[0])

    def _parse_count_token(tok: str) -> int:
        tok = tok.strip().upper()
        num = "".join(ch for ch in tok if ch.isdigit())
        return int(num) if num else 0

    n_analog = _parse_count_token(counts[1]) if len(counts) >= 2 else 0
    n_digital = _parse_count_token(counts[2]) if len(counts) >= 3 else max(total_channels - n_analog, 0)

    analog_channels: List[AnalogChannel] = []
    digital_channels: List[DigitalChannel] = []

    for _ in range(n_analog):
        parts = [x.strip() for x in lines[pos].split(",")]
        pos += 1
        if len(parts) < 13:
            raise RuntimeError(f"Linha de canal analógico inválida: {parts}")
        analog_channels.append(
            AnalogChannel(
                index_1based=int(parts[0]),
                name=parts[1],
                phase=parts[2],
                ccbm=parts[3],
                unit=parts[4],
                a=float(parts[5]),
                b=float(parts[6]),
                skew=float(parts[7]),
                min_raw=float(parts[8]),
                max_raw=float(parts[9]),
                primary=float(parts[10]),
                secondary=float(parts[11]),
                ps=parts[12],
            )
        )

    for _ in range(n_digital):
        parts = [x.strip() for x in lines[pos].split(",")]
        pos += 1
        if len(parts) < 5:
            raise RuntimeError(f"Linha de canal digital inválida: {parts}")
        digital_channels.append(
            DigitalChannel(
                index_1based=int(parts[0]),
                name=parts[1],
                phase=parts[2],
                ccbm=parts[3],
                y=int(parts[4]),
            )
        )

    system_frequency_hz = float(lines[pos].replace(",", "."))
    pos += 1

    nrates = int(lines[pos].split(",")[0].strip())
    pos += 1
    sample_rates: List[SampleRateSegment] = []
    for _ in range(nrates):
        parts = [x.strip() for x in lines[pos].split(",")]
        pos += 1
        if len(parts) < 2:
            raise RuntimeError(f"Linha de taxa de amostragem inválida: {parts}")
        sample_rates.append(
            SampleRateSegment(
                samples_per_second=float(parts[0].replace(",", ".")),
                end_sample=int(float(parts[1].replace(",", "."))),
            )
        )

    start_timestamp_str = lines[pos]
    pos += 1
    trigger_timestamp_str = lines[pos]
    pos += 1

    data_format = lines[pos].split(",")[0].strip().upper()
    pos += 1

    timemult = 1.0
    time_code = local_code = tmq_code = leap_second = None
    if pos < len(lines):
        try:
            timemult = float(lines[pos].replace(",", "."))
            pos += 1
        except ValueError:
            pass

    if pos < len(lines):
        time_code = lines[pos]
        pos += 1
    if pos < len(lines):
        local_code = lines[pos]
        pos += 1
    if pos < len(lines):
        tmq_code = lines[pos]
        pos += 1
    if pos < len(lines):
        leap_second = lines[pos]
        pos += 1

    return ComtradeCfg(
        station_name=station_name,
        rec_dev_id=rec_dev_id,
        rev_year=rev_year,
        analog_channels=analog_channels,
        digital_channels=digital_channels,
        system_frequency_hz=system_frequency_hz,
        sample_rates=sample_rates,
        start_timestamp_str=start_timestamp_str,
        trigger_timestamp_str=trigger_timestamp_str,
        data_format=data_format,
        timemult=timemult,
        time_code=time_code,
        local_code=local_code,
        tmq_code=tmq_code,
        leap_second=leap_second,
    )


def resolve_data_file(cfg_path: str, data_arg: Optional[str]) -> str:
    if data_arg:
        if not os.path.exists(data_arg):
            raise RuntimeError(f"Arquivo de dados não encontrado: {data_arg}")
        return data_arg

    base, _ = os.path.splitext(cfg_path)
    candidates = [base + ".dat", base + ".DAT", base + ".bdat", base + ".BDAT"]
    for cand in candidates:
        if os.path.exists(cand):
            return cand
    raise RuntimeError("Não encontrei arquivo .dat/.bdat. Use --data para informar explicitamente.")


def digital_word_count(n_digital: int) -> int:
    return (n_digital + 15) // 16


def record_size_bytes(cfg: ComtradeCfg, ts_bytes: int) -> int:
    fmt = cfg.data_format.upper()
    nA = len(cfg.analog_channels)
    nD_words = digital_word_count(len(cfg.digital_channels))

    if fmt == "BINARY":
        analog_bytes = 2 * nA
    elif fmt in ("BINARY32", "FLOAT32"):
        analog_bytes = 4 * nA
    else:
        raise RuntimeError(f"record_size_bytes não se aplica ao formato {fmt}")

    return 4 + ts_bytes + analog_bytes + 2 * nD_words


def infer_timestamp_bytes(cfg: ComtradeCfg, data_path: str) -> int:
    if cfg.data_format.upper() == "ASCII":
        return 0

    fsize = os.path.getsize(data_path)
    total = cfg.total_samples
    if total <= 0:
        raise RuntimeError("CFG sem total de amostras válido.")

    candidates = []
    for ts_bytes in (4, 8):
        rec = record_size_bytes(cfg, ts_bytes)
        if rec > 0 and (fsize % rec == 0):
            candidates.append((ts_bytes, fsize // rec))

    for ts_bytes, inferred_total in candidates:
        if inferred_total == total:
            return ts_bytes

    if len(candidates) == 1:
        return candidates[0][0]

    return 4


def estimate_duration_seconds(cfg: ComtradeCfg) -> float:
    if not cfg.sample_rates:
        return 0.0
    prev_end = 0
    total = 0.0
    for seg in cfg.sample_rates:
        seg_count = seg.end_sample - prev_end
        prev_end = seg.end_sample
        if seg.samples_per_second > 0:
            total += seg_count / seg.samples_per_second
    return total


def timestamp_ticks_to_seconds(cfg: ComtradeCfg, timestamp_ticks: int) -> float:
    return (timestamp_ticks * cfg.timemult) / 1_000_000.0


def get_segment_sample_rate(cfg: ComtradeCfg, sample_number: int) -> float:
    for seg in cfg.sample_rates:
        if sample_number <= seg.end_sample:
            return seg.samples_per_second
    return cfg.sample_rates[-1].samples_per_second if cfg.sample_rates else 0.0


def iter_ascii_records(cfg: ComtradeCfg, data_path: str) -> Iterator[SampleRecord]:
    with open(data_path, "r", encoding="utf-8-sig", errors="ignore") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            parts = [x.strip() for x in row if x is not None]
            if len(parts) < 2 + len(cfg.analog_channels):
                continue

            sample_number = int(float(parts[0]))
            timestamp_ticks = int(float(parts[1]))

            analog_real: List[float] = []
            base = 2
            for ch, raw_text in zip(cfg.analog_channels, parts[base:base + len(cfg.analog_channels)]):
                analog_real.append(ch.apply(float(raw_text)))

            digital_bits: List[int] = []
            dig_start = base + len(cfg.analog_channels)
            for tok in parts[dig_start:]:
                try:
                    word = int(float(tok))
                except ValueError:
                    word = 0
                for bit in range(16):
                    digital_bits.append((word >> bit) & 1)

            yield SampleRecord(sample_number, timestamp_ticks, analog_real, digital_bits[:len(cfg.digital_channels)])


def _unpack_analog_values(fmt: str, blob: bytes, count: int) -> List[float]:
    if count == 0:
        return []
    if fmt == "BINARY":
        return list(struct.unpack("<" + "h" * count, blob))
    if fmt == "BINARY32":
        return list(struct.unpack("<" + "i" * count, blob))
    if fmt == "FLOAT32":
        return list(struct.unpack("<" + "f" * count, blob))
    raise RuntimeError(f"Formato binário não suportado: {fmt}")


def iter_binary_records(cfg: ComtradeCfg, data_path: str) -> Iterator[SampleRecord]:
    fmt = cfg.data_format.upper()
    ts_bytes = infer_timestamp_bytes(cfg, data_path)
    nA = len(cfg.analog_channels)
    nD_words = digital_word_count(len(cfg.digital_channels))
    rec_size = record_size_bytes(cfg, ts_bytes)

    with open(data_path, "rb") as f:
        while True:
            buf = f.read(rec_size)
            if not buf:
                break
            if len(buf) != rec_size:
                raise RuntimeError("Arquivo de dados truncado ou tamanho de registro inconsistente.")

            offset = 0
            sample_number = struct.unpack_from("<I", buf, offset)[0]
            offset += 4

            if ts_bytes == 4:
                timestamp_ticks = struct.unpack_from("<I", buf, offset)[0]
            else:
                timestamp_ticks = struct.unpack_from("<Q", buf, offset)[0]
            offset += ts_bytes

            analog_width = 2 if fmt == "BINARY" else 4
            analog_blob = buf[offset: offset + analog_width * nA]
            offset += analog_width * nA
            analog_raw = _unpack_analog_values(fmt, analog_blob, nA)
            analog_real = [ch.apply(raw) for ch, raw in zip(cfg.analog_channels, analog_raw)]

            digital_bits: List[int] = []
            for _ in range(nD_words):
                word = struct.unpack_from("<H", buf, offset)[0]
                offset += 2
                for bit in range(16):
                    digital_bits.append((word >> bit) & 1)

            yield SampleRecord(sample_number, timestamp_ticks, analog_real, digital_bits[:len(cfg.digital_channels)])


def iter_records(cfg: ComtradeCfg, data_path: str) -> Iterator[SampleRecord]:
    if cfg.data_format.upper() == "ASCII":
        yield from iter_ascii_records(cfg, data_path)
    elif cfg.data_format.upper() in ("BINARY", "BINARY32", "FLOAT32"):
        yield from iter_binary_records(cfg, data_path)
    else:
        raise RuntimeError(f"Formato COMTRADE não suportado: {cfg.data_format}")


def parse_channels_arg(channels_arg: Optional[str], available_count: int) -> List[int]:
    if not channels_arg:
        return list(range(1, min(available_count, 4) + 1))
    out: List[int] = []
    for tok in channels_arg.split(","):
        tok = tok.strip()
        if not tok:
            continue
        idx = int(tok)
        if idx < 1 or idx > available_count:
            raise RuntimeError(f"Canal analógico inválido: {idx}. Faixa: 1..{available_count}")
        out.append(idx)
    if len(out) > 4:
        raise RuntimeError("O MCP4728 suporta no máximo 4 canais por vez.")
    return out


def parse_optional_float_list(text: Optional[str]) -> List[Optional[float]]:
    if not text:
        return []
    out: List[Optional[float]] = []
    for tok in text.split(","):
        tok = tok.strip()
        if not tok or tok.lower() in ("auto", "none", "null"):
            out.append(None)
        else:
            out.append(float(tok))
    return out


def collect_selected_channel_stats(cfg: ComtradeCfg, data_path: str, selected_1based: Sequence[int]) -> Tuple[List[float], int, Optional[int], Optional[int]]:
    max_abs = [0.0 for _ in selected_1based]
    count = 0
    first_ts = None
    last_ts = None
    for rec in iter_records(cfg, data_path):
        count += 1
        if first_ts is None:
            first_ts = rec.timestamp_ticks
        last_ts = rec.timestamp_ticks
        for i, ch1 in enumerate(selected_1based):
            value = rec.analog_real[ch1 - 1]
            av = abs(value)
            if av > max_abs[i]:
                max_abs[i] = av
    return max_abs, count, first_ts, last_ts


def build_channel_fullscales(stats_max_abs: Sequence[float], fullscales_arg: Sequence[Optional[float]]) -> List[float]:
    out: List[float] = []
    for i, auto_max in enumerate(stats_max_abs):
        fs = fullscales_arg[i] if i < len(fullscales_arg) else None
        if fs is None:
            fs = auto_max if auto_max > 0 else 1.0
        if fs <= 0:
            raise RuntimeError("Fullscale deve ser > 0.")
        out.append(fs)
    return out


def real_to_dac_code(value: float, fullscale: float) -> int:
    norm = 0.0 if fullscale == 0 else (value / fullscale)
    if norm > 1.0:
        norm = 1.0
    elif norm < -1.0:
        norm = -1.0
    code = int(round(DAC_MID + norm * DAC_MID))
    if code < 0:
        return 0
    if code > DAC_MAX:
        return DAC_MAX
    return code


def build_frame(dt_us: int, dac_codes: Sequence[int], flags: int) -> bytes:
    vals = list(dac_codes[:4]) + [DAC_MID] * max(0, 4 - len(dac_codes))
    vals = vals[:4]
    dt_us = max(0, min(int(dt_us), 0xFFFFFFFF))
    flags &= 0xFF

    raw14 = struct.pack(
        "<BI4HB",
        FRAME_SYNC,
        dt_us,
        int(vals[0]) & 0xFFFF,
        int(vals[1]) & 0xFFFF,
        int(vals[2]) & 0xFFFF,
        int(vals[3]) & 0xFFFF,
        flags,
    )
    checksum = sum(raw14) & 0xFF
    return raw14 + bytes([checksum, FRAME_END])


def wait_esp32_ready(port: str, baud: int, timeout_s: float = 8.0) -> serial.Serial:
    ser = serial.Serial(port, baudrate=baud, timeout=0.2, write_timeout=2.0)
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass
    time.sleep(2.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    t0 = time.time()
    seen: List[str] = []
    while time.time() - t0 < timeout_s:
        line = ser.readline()
        if not line:
            continue
        text = line.decode(errors="ignore").strip()
        if not text:
            continue
        seen.append(text)
        print(f"[ESP32] {text}")
        if text == "READY":
            return ser

    ser.close()
    raise RuntimeError(f"Não recebi READY da ESP32. Recebido: {seen}")


def print_summary(cfg: ComtradeCfg, data_path: str, selected_1based: Sequence[int], fullscales: Sequence[float], total_records: int, duration_s: float) -> None:
    print("=== COMTRADE ===")
    print(f"Estação: {cfg.station_name}")
    print(f"Dispositivo: {cfg.rec_dev_id}")
    print(f"Revisão: {cfg.rev_year}")
    print(f"Formato: {cfg.data_format}")
    print(f"Arquivo de dados: {data_path}")
    print(f"Canais analógicos: {len(cfg.analog_channels)} | digitais: {len(cfg.digital_channels)}")
    print(f"Amostras: {total_records} | duração: {duration_s:.6f} s")
    print(f"Taxas de amostragem declaradas: {len(cfg.sample_rates)}")
    for i, seg in enumerate(cfg.sample_rates, start=1):
        print(f"  taxa[{i}]: {seg.samples_per_second} Hz até amostra {seg.end_sample}")
    for i, ch1 in enumerate(selected_1based, start=1):
        ch = cfg.analog_channels[ch1 - 1]
        print(f"DAC {i}: canal analógico {ch1} ({ch.name}) [{ch.unit}] | fullscale={fullscales[i-1]:.6f}")


def save_tx_row(
    writer: csv.DictWriter,
    cfg: ComtradeCfg,
    rec: SampleRecord,
    dt_us: int,
    codes: Sequence[int],
    flags: int,
    selected_1based: Sequence[int],
    fullscales: Sequence[float],
) -> None:
    padded_codes = list(codes[:4]) + [DAC_MID] * max(0, 4 - len(codes))
    padded_codes = padded_codes[:4]

    row = {
        "sample_number": rec.sample_number,
        "timestamp_ticks": rec.timestamp_ticks,
        "dt_us": dt_us,
        "flags": flags,
        "dac0_code": int(padded_codes[0]),
        "dac1_code": int(padded_codes[1]),
        "dac2_code": int(padded_codes[2]),
        "dac3_code": int(padded_codes[3]),
    }

    for i in range(4):
        if i < len(selected_1based):
            ch1 = selected_1based[i]
            ch = cfg.analog_channels[ch1 - 1]
            row[f"dac{i}_src_channel"] = ch1
            row[f"dac{i}_src_name"] = ch.name
            row[f"dac{i}_unit"] = ch.unit
            row[f"dac{i}_real_value"] = rec.analog_real[ch1 - 1]
            row[f"dac{i}_fullscale"] = fullscales[i]
        else:
            row[f"dac{i}_src_channel"] = ""
            row[f"dac{i}_src_name"] = ""
            row[f"dac{i}_unit"] = ""
            row[f"dac{i}_real_value"] = ""
            row[f"dac{i}_fullscale"] = ""

    writer.writerow(row)


def stream_records(
    ser: serial.Serial,
    cfg: ComtradeCfg,
    data_path: str,
    selected_1based: Sequence[int],
    fullscales: Sequence[float],
    speed: float,
    idle_after_end: bool,
    tx_csv_path: Optional[str] = None,
) -> None:
    prev_ts: Optional[int] = None
    sent = 0
    start_wall = time.time()

    csv_file = None
    csv_writer = None

    try:
        if tx_csv_path:
            csv_file = open(tx_csv_path, "w", newline="", encoding="utf-8")
            fieldnames = [
                "sample_number", "timestamp_ticks", "dt_us", "flags",
                "dac0_code", "dac1_code", "dac2_code", "dac3_code",
                "dac0_src_channel", "dac0_src_name", "dac0_unit", "dac0_real_value", "dac0_fullscale",
                "dac1_src_channel", "dac1_src_name", "dac1_unit", "dac1_real_value", "dac1_fullscale",
                "dac2_src_channel", "dac2_src_name", "dac2_unit", "dac2_real_value", "dac2_fullscale",
                "dac3_src_channel", "dac3_src_name", "dac3_unit", "dac3_real_value", "dac3_fullscale",
            ]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            csv_writer.writeheader()

        for rec in iter_records(cfg, data_path):
            if prev_ts is None:
                dt_s = 0.0
                flags = FLAG_RESET_CLOCK
            else:
                dt_s = timestamp_ticks_to_seconds(cfg, rec.timestamp_ticks - prev_ts)
                if dt_s <= 0:
                    fs = get_segment_sample_rate(cfg, rec.sample_number)
                    dt_s = (1.0 / fs) if fs > 0 else 0.0
                flags = 0

            prev_ts = rec.timestamp_ticks
            dt_s = dt_s / speed if speed > 0 else dt_s
            dt_us = int(round(dt_s * 1_000_000.0))
            if dt_us < 0:
                dt_us = 0

            codes = []
            for fs, ch1 in zip(fullscales, selected_1based):
                value = rec.analog_real[ch1 - 1]
                codes.append(real_to_dac_code(value, fs))

            ser.write(build_frame(dt_us, codes, flags))

            if csv_writer is not None:
                save_tx_row(csv_writer, cfg, rec, dt_us, codes, flags, selected_1based, fullscales)

            sent += 1
            if sent % 1000 == 0:
                elapsed = time.time() - start_wall
                print(f"Enviadas {sent} amostras... ({elapsed:.1f}s)")

        if idle_after_end:
            ser.write(build_frame(0, [DAC_MID] * 4, FLAG_IDLE_AFTER | FLAG_RESET_CLOCK))
            if csv_writer is not None:
                idle_rec = SampleRecord(
                    sample_number=sent + 1,
                    timestamp_ticks=prev_ts if prev_ts is not None else 0,
                    analog_real=[],
                    digital_bits=[],
                )
                save_tx_row(csv_writer, cfg, idle_rec, 0, [DAC_MID] * 4, FLAG_IDLE_AFTER | FLAG_RESET_CLOCK, [], [])

        ser.flush()
        print(f"Fim do streaming: {sent} amostras enviadas.")
        if tx_csv_path:
            print(f"CSV de transmissão salvo em: {tx_csv_path}")

    finally:
        if csv_file is not None:
            csv_file.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Streamer universal COMTRADE -> ESP32/MCP4728")
    parser.add_argument("--port", required=True, help="porta serial da ESP32, ex: /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"baudrate serial (default: {DEFAULT_BAUD})")
    parser.add_argument("--cfg", required=True, help="arquivo .cfg do COMTRADE")
    parser.add_argument("--data", help="arquivo .dat/.bdat do COMTRADE (opcional, mas recomendado quando o nome muda)")
    parser.add_argument("--channels", help="canais analógicos 1-based para DACs, ex: 1,2,3,4")
    parser.add_argument("--fullscales", help="fullscale físico por canal, ex: 188.5,1604.5,auto,auto")
    parser.add_argument("--speed", type=float, default=1.0, help="fator de velocidade; 1.0 = tempo real")
    parser.add_argument("--no-idle-after-end", action="store_true", help="não força retorno ao nível médio ao final")
    parser.add_argument("--tx-csv", help="salva em CSV os dados enviados para a ESP32")
    args = parser.parse_args()

    cfg_path = args.cfg
    data_path = resolve_data_file(cfg_path, args.data)

    cfg = parse_cfg(cfg_path)
    if not cfg.analog_channels:
        raise RuntimeError("Este COMTRADE não possui canais analógicos para reproduzir no MCP4728.")

    selected_1based = parse_channels_arg(args.channels, len(cfg.analog_channels))
    stats_max_abs, total_records, first_ts, last_ts = collect_selected_channel_stats(cfg, data_path, selected_1based)
    fullscales = build_channel_fullscales(stats_max_abs, parse_optional_float_list(args.fullscales))

    if first_ts is not None and last_ts is not None and last_ts >= first_ts:
        duration_s = timestamp_ticks_to_seconds(cfg, last_ts - first_ts)
        if duration_s <= 0:
            duration_s = estimate_duration_seconds(cfg)
    else:
        duration_s = estimate_duration_seconds(cfg)

    print_summary(cfg, data_path, selected_1based, fullscales, total_records, duration_s)
    ser = wait_esp32_ready(args.port, args.baud)
    try:
        print("Iniciando streaming...")
        stream_records(
            ser=ser,
            cfg=cfg,
            data_path=data_path,
            selected_1based=selected_1based,
            fullscales=fullscales,
            speed=args.speed,
            idle_after_end=not args.no_idle_after_end,
            tx_csv_path=args.tx_csv,
        )
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")
        raise SystemExit(130)
    except Exception as exc:
        print(f"ERRO: {exc}", file=sys.stderr)
        raise SystemExit(1)
#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import struct
import time
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple, Set

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial não está instalado. Instale com: pip install pyserial") from exc

# ============================================================
# Protocolo legado (compatível com generator.py / v_0.ino)
# [0]   0xA5
# [1:4] dt_us (uint32 LE)
# [5:6] dac_v
# [7:8] dac_i
# [9]   flags
# [10]  checksum = sum(buf[0:10]) & 0xFF
# [11]  0x5A
# ============================================================

MAGIC0 = 0xA5
MAGIC1 = 0x5A
FLAG_IDLE_AFTER = 0x01
FLAG_RESET_CLOCK = 0x02

V_MID = 1.65
V_AMP = 1.55
DAC_MAX = 4095
DAC_VREF = 3.3
DAC_MID = 2048

NOM_CYCLES = 5
V_FAULT_LIMIT_MULT = 2.0
I_FAULT_LIMIT_MULT = 3.0

DEFAULT_BAUD = 921600

STATE_IDLE = "idle"
STATE_PREF_LOOP = "pref_loop"
STATE_PLAY_ONCE = "play_once"
STATE_PLAY_REPEAT = "play_repeat"
STATE_EXIT = "exit"


@dataclass
class AnalogCh:
    a: float = 1.0
    b: float = 0.0

    def raw_to_eng(self, raw: int) -> float:
        return self.a * float(raw) + self.b


@dataclass
class ComtradeCfg:
    n_analog: int = 0
    n_digital: int = 0
    freq_hz: float = 60.0
    sample_rate_hz: float = 0.0
    time_mult: float = 1.0
    data_format: str = ""
    ch_v: AnalogCh = field(default_factory=AnalogCh)
    ch_i: AnalogCh = field(default_factory=AnalogCh)
    ts64: bool = False
    rec_size: int = 0
    total_records: int = 0
    digital_bytes: int = 0


@dataclass
class SamplePacket:
    seq: int
    dt_us: int
    dac_v: int
    dac_i: int
    flags: int
    t_us: int
    eng_v: float
    eng_i: float
    clip_v: int
    clip_i: int


@dataclass
class TxLogEntry:
    tx_index: int
    host_t_s: float
    mode: str
    run_id: int
    event: str
    seq: int
    t_us: int
    dt_us: int
    dac_v: int
    dac_i: int
    eng_v: float
    eng_i: float
    clip_v: int
    clip_i: int
    flags: int


class TxLogger:
    def __init__(self) -> None:
        self.rows: List[TxLogEntry] = []
        self.tx_index = 0

    def append(
        self,
        *,
        mode: str,
        run_id: int,
        event: str,
        seq: int,
        t_us: int,
        dt_us: int,
        dac_v: int,
        dac_i: int,
        eng_v: float,
        eng_i: float,
        clip_v: int,
        clip_i: int,
        flags: int,
    ) -> None:
        self.tx_index += 1
        self.rows.append(
            TxLogEntry(
                tx_index=self.tx_index,
                host_t_s=time.time(),
                mode=mode,
                run_id=run_id,
                event=event,
                seq=seq,
                t_us=t_us,
                dt_us=dt_us,
                dac_v=dac_v,
                dac_i=dac_i,
                eng_v=eng_v,
                eng_i=eng_i,
                clip_v=clip_v,
                clip_i=clip_i,
                flags=flags,
            )
        )

    def save_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f)
            wr.writerow([
                "tx_index",
                "host_t_s",
                "mode",
                "run_id",
                "event",
                "seq",
                "t_us",
                "dt_us",
                "dacV_code",
                "dacI_code",
                "engV",
                "engI",
                "clipV",
                "clipI",
                "flags",
            ])
            for r in self.rows:
                wr.writerow([
                    r.tx_index,
                    f"{r.host_t_s:.6f}",
                    r.mode,
                    r.run_id,
                    r.event,
                    r.seq,
                    r.t_us,
                    r.dt_us,
                    r.dac_v,
                    r.dac_i,
                    f"{r.eng_v:.6f}",
                    f"{r.eng_i:.6f}",
                    r.clip_v,
                    r.clip_i,
                    r.flags,
                ])


class IdleSpanTracker:
    def __init__(self, logger: TxLogger) -> None:
        self.logger = logger
        self.active = False
        self.start_monotonic = 0.0
        self.mode = "idle"
        self.run_id = 0
        self.event = "idle_span"

    def start(self, *, mode: str, run_id: int, event: str = "idle_span") -> None:
        if self.active:
            return
        self.active = True
        self.start_monotonic = time.monotonic()
        self.mode = mode
        self.run_id = run_id
        self.event = event

    def stop(self) -> None:
        if not self.active:
            return
        elapsed_s = time.monotonic() - self.start_monotonic
        elapsed_us = max(0, int(round(elapsed_s * 1_000_000.0)))
        self.logger.append(
            mode=self.mode,
            run_id=self.run_id,
            event=self.event,
            seq=-1,
            t_us=0,
            dt_us=elapsed_us,
            dac_v=DAC_MID,
            dac_i=DAC_MID,
            eng_v=0.0,
            eng_i=0.0,
            clip_v=0,
            clip_i=0,
            flags=FLAG_IDLE_AFTER | FLAG_RESET_CLOCK,
        )
        self.active = False

    def restart(self, *, mode: str, run_id: int, event: str = "idle_span") -> None:
        self.stop()
        self.start(mode=mode, run_id=run_id, event=event)


class CommandController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._command: Optional[str] = None

    def set(self, cmd: str) -> None:
        with self._lock:
            self._command = cmd

    def get(self) -> Optional[str]:
        with self._lock:
            return self._command

    def clear(self) -> None:
        with self._lock:
            self._command = None


# ---------------- CFG / BDAT ----------------

def parse_cfg(cfg_path: Path) -> ComtradeCfg:
    lines = [ln.strip() for ln in cfg_path.read_text(encoding="utf-8", errors="ignore").splitlines() if ln.strip()]
    if len(lines) < 6:
        raise RuntimeError("CFG inválido: poucas linhas")

    cfg = ComtradeCfg()

    parts = [p.strip() for p in lines[1].split(",")]
    if len(parts) < 3:
        raise RuntimeError("CFG inválido na linha de TT,NA,ND")

    def parse_count_token(tok: str, suffix: str) -> int:
        tok = tok.strip().upper()
        if tok.endswith(suffix):
            tok = tok[:-len(suffix)]
        return int(tok)

    cfg.n_analog = parse_count_token(parts[1], "A")
    cfg.n_digital = parse_count_token(parts[2], "D")

    idx = 2
    for ch in range(cfg.n_analog):
        row = [p.strip() for p in lines[idx].split(",")]
        idx += 1
        if len(row) < 7:
            raise RuntimeError(f"Linha analógica inválida no canal {ch}")
        a = float(row[5])
        b = float(row[6])
        if ch == 0:
            cfg.ch_v = AnalogCh(a, b)
        elif ch == 1:
            cfg.ch_i = AnalogCh(a, b)

    idx += cfg.n_digital

    if idx < len(lines):
        try:
            fr = float(lines[idx])
            if 1.0 < fr < 500.0:
                cfg.freq_hz = fr
        except ValueError:
            pass
        idx += 1

    if idx >= len(lines):
        raise RuntimeError("CFG inválido: faltou linha de número de taxas")
    nrates = int(lines[idx])
    idx += 1

    if nrates > 0:
        if idx >= len(lines):
            raise RuntimeError("CFG inválido: faltou linha da taxa")
        rate_parts = [p.strip() for p in lines[idx].split(",")]
        cfg.sample_rate_hz = float(rate_parts[0])
        idx += nrates

    idx += 2

    if idx < len(lines):
        cfg.data_format = lines[idx].strip()
        idx += 1
    else:
        raise RuntimeError("CFG inválido: faltou formato de dados")

    if idx < len(lines):
        try:
            tm = float(lines[idx])
            if tm > 0.0:
                cfg.time_mult = tm
        except ValueError:
            pass

    return cfg


def compute_bdat_layout(cfg: ComtradeCfg, bdat_path: Path) -> None:
    if cfg.data_format.upper() != "BINARY":
        raise RuntimeError(f"Formato não suportado neste script: {cfg.data_format!r}. Esperado: BINARY")

    file_size = bdat_path.stat().st_size
    digital_words = (cfg.n_digital + 15) // 16
    cfg.digital_bytes = digital_words * 2

    rec_size32 = 4 + 4 + (cfg.n_analog * 2) + cfg.digital_bytes
    if rec_size32 > 0 and file_size % rec_size32 == 0:
        cfg.ts64 = False
        cfg.rec_size = rec_size32
    else:
        rec_size64 = 4 + 8 + (cfg.n_analog * 2) + cfg.digital_bytes
        if rec_size64 > 0 and file_size % rec_size64 == 0:
            cfg.ts64 = True
            cfg.rec_size = rec_size64
        else:
            raise RuntimeError("Não foi possível inferir o layout do BDAT (timestamp 32/64 bits)")

    cfg.total_records = file_size // cfg.rec_size
    if cfg.total_records <= 0:
        raise RuntimeError("BDAT sem registros válidos")


def iter_bdat_records(cfg: ComtradeCfg, bdat_path: Path):
    with bdat_path.open("rb") as f:
        for _ in range(cfg.total_records):
            rec = f.read(cfg.rec_size)
            if len(rec) != cfg.rec_size:
                raise RuntimeError("Leitura incompleta do BDAT")

            off = 0
            _sample_num = struct.unpack_from("<I", rec, off)[0]
            off += 4

            if cfg.ts64:
                ts = struct.unpack_from("<q", rec, off)[0]
                off += 8
            else:
                ts = struct.unpack_from("<i", rec, off)[0]
                off += 4

            eng_v = 0.0
            eng_i = 0.0
            for ch in range(cfg.n_analog):
                raw = struct.unpack_from("<h", rec, off)[0]
                off += 2
                if ch == 0:
                    eng_v = cfg.ch_v.raw_to_eng(raw)
                elif ch == 1:
                    eng_i = cfg.ch_i.raw_to_eng(raw)

            yield ts, eng_v, eng_i


def compute_nominal_peaks(cfg: ComtradeCfg, bdat_path: Path) -> Tuple[float, float, float, float]:
    if cfg.sample_rate_hz < 1.0 or cfg.freq_hz < 1.0:
        raise RuntimeError("sample_rate_hz ou freq_hz inválidos para calcular pico nominal")

    samples_per_cycle = round(cfg.sample_rate_hz / cfg.freq_hz)
    samples_per_cycle = max(samples_per_cycle, 8)
    limit_n = min(cfg.total_records, samples_per_cycle * NOM_CYCLES)
    if limit_n < samples_per_cycle:
        raise RuntimeError("Amostras insuficientes para calcular pico nominal")

    cycle_peak_v = 0.0
    cycle_peak_i = 0.0
    idx_in_cycle = 0
    peaks_v: List[float] = []
    peaks_i: List[float] = []

    for idx, (_, eng_v, eng_i) in enumerate(iter_bdat_records(cfg, bdat_path)):
        if idx >= limit_n:
            break
        cycle_peak_v = max(cycle_peak_v, abs(eng_v))
        cycle_peak_i = max(cycle_peak_i, abs(eng_i))
        idx_in_cycle += 1
        if idx_in_cycle >= samples_per_cycle:
            peaks_v.append(cycle_peak_v)
            peaks_i.append(cycle_peak_i)
            cycle_peak_v = 0.0
            cycle_peak_i = 0.0
            idx_in_cycle = 0

    if not peaks_v or not peaks_i:
        raise RuntimeError("Falha ao calcular os picos nominais")

    v_nom = max(sum(peaks_v) / len(peaks_v), 1.0)
    i_nom = max(sum(peaks_i) / len(peaks_i), 1.0)
    return v_nom, i_nom, v_nom * V_FAULT_LIMIT_MULT, i_nom * I_FAULT_LIMIT_MULT


def volts_to_dac12(v: float) -> int:
    v = max(0.0, min(DAC_VREF, v))
    return max(0, min(DAC_MAX, int(round((v / DAC_VREF) * DAC_MAX))))


def map_eng_to_volts(eng: float, clip_peak: float) -> float:
    if clip_peak < 1e-9:
        clip_peak = 1.0
    eng = max(-clip_peak, min(clip_peak, eng))
    x = eng / clip_peak
    x = max(-1.0, min(1.0, x))
    return V_MID + x * V_AMP


def build_samples(cfg: ComtradeCfg, bdat_path: Path) -> Tuple[List[SamplePacket], float, float]:
    v_nom, i_nom, v_clip, i_clip = compute_nominal_peaks(cfg, bdat_path)
    print(f"CFG: nA={cfg.n_analog} nD={cfg.n_digital} fs={cfg.sample_rate_hz:.2f}Hz f={cfg.freq_hz:.2f}Hz timeMult={cfg.time_mult}")
    print(f"Nominal(5 ciclos): V_nom_peak={v_nom:.6f} I_nom_peak={i_nom:.6f}")
    print(f"Limites: V_clip={v_clip:.6f} ({V_FAULT_LIMIT_MULT:.1f}x) I_clip={i_clip:.6f} ({I_FAULT_LIMIT_MULT:.1f}x)")
    print(f"Layout: ts64={cfg.ts64} recSize={cfg.rec_size} total={cfg.total_records}")

    samples: List[SamplePacket] = []
    prev_ts: Optional[int] = None
    t_accum = 0

    for seq, (ts, eng_v, eng_i) in enumerate(iter_bdat_records(cfg, bdat_path)):
        if prev_ts is None:
            dt_us = 0
        else:
            dts = max(0, ts - prev_ts)
            dt_us = int(round(float(dts) * cfg.time_mult * 1_000_000))
            if dt_us == 0 and cfg.sample_rate_hz > 0.1:
                dt_us = int(round(1_000_000.0 / cfg.sample_rate_hz))
        prev_ts = ts
        t_accum += dt_us

        clip_v = 1 if abs(eng_v) > v_clip else 0
        clip_i = 1 if abs(eng_i) > i_clip else 0
        flags = (clip_v & 0x01) | ((clip_i & 0x01) << 1)

        dac_v = volts_to_dac12(map_eng_to_volts(eng_v, v_clip))
        dac_i = volts_to_dac12(map_eng_to_volts(eng_i, i_clip))

        samples.append(
            SamplePacket(
                seq=seq,
                dt_us=dt_us,
                dac_v=dac_v,
                dac_i=dac_i,
                flags=flags,
                t_us=t_accum,
                eng_v=eng_v,
                eng_i=eng_i,
                clip_v=clip_v,
                clip_i=clip_i,
            )
        )

    return samples, v_clip, i_clip


# ---------------- CICLO PRÉ-FALTA COM INTERPOLAÇÃO ----------------

def _lerp(a: float, b: float, alpha: float) -> float:
    return a + alpha * (b - a)

import math

def build_test_sine(
    freq_hz: float = 60.0,
    amplitude_v: float = 3.0,
    sample_rate: float = 2000.0,
    cycles: int = 10,
) -> List[SamplePacket]:

    samples: List[SamplePacket] = []

    dt_us = int(round(1_000_000 / sample_rate))
    total_samples = int(sample_rate * cycles / freq_hz)

    t_accum = 0

    for i in range(total_samples):

        t = i / sample_rate
        angle = 2 * math.pi * freq_hz * t

        v = V_MID + (amplitude_v / 2.0) * math.sin(angle)
        i_fake = V_MID + (amplitude_v / 2.0) * math.sin(angle)

        dac_v = volts_to_dac12(v)
        dac_i = volts_to_dac12(i_fake)

        samples.append(
            SamplePacket(
                seq=i,
                dt_us=dt_us,
                dac_v=dac_v,
                dac_i=dac_i,
                flags=0,
                t_us=t_accum,
                eng_v=v,
                eng_i=i_fake,
                clip_v=0,
                clip_i=0,
            )
        )

        t_accum += dt_us

    print(f"Modo teste: senoide {freq_hz} Hz | {amplitude_v} Vpp | {total_samples} amostras")

    return samples

def build_prefault_cycle_buffer(
    samples: List[SamplePacket],
    cfg: ComtradeCfg,
    v_clip: float,
    i_clip: float,
) -> List[SamplePacket]:
    """
    Regras:
    - o ciclo começa exatamente na primeira amostra real do arquivo
    - o final é construído por interpolação para coincidir com a amplitude inicial
    - o período total fica próximo de 1/f
    """
    if len(samples) < 6:
        raise RuntimeError("Amostras insuficientes para construir o buffer pré-falta")

    start = samples[0]
    target_v = start.eng_v
    target_i = start.eng_i
    target_period_us = 1_000_000.0 / cfg.freq_hz
    approx_samples = max(8, int(round(cfg.sample_rate_hz / cfg.freq_hz)))

    # janela de busca ao redor de 1 período
    search_left = max(1, approx_samples - max(6, int(0.35 * approx_samples)))
    search_right = min(len(samples) - 2, approx_samples + max(6, int(0.35 * approx_samples)))

    best = None
    best_score = None

    # Procura um trecho [i, i+1] onde a tensão cruza a tensão inicial
    for i in range(search_left, search_right + 1):
        s0 = samples[i]
        s1 = samples[i + 1]

        dv0 = s0.eng_v - target_v
        dv1 = s1.eng_v - target_v

        # cruza o nível da primeira amostra
        crosses = (dv0 == 0.0) or (dv1 == 0.0) or (dv0 < 0.0 < dv1) or (dv1 < 0.0 < dv0)
        if not crosses:
            continue

        denom = (s1.eng_v - s0.eng_v)
        if abs(denom) < 1e-12:
            alpha = 0.5
        else:
            alpha = (target_v - s0.eng_v) / denom
        alpha = max(0.0, min(1.0, alpha))

        cross_t_us = _lerp(float(s0.t_us), float(s1.t_us), alpha)
        cross_i = _lerp(s0.eng_i, s1.eng_i, alpha)

        period_err = abs(cross_t_us - target_period_us) / target_period_us
        current_err = abs(cross_i - target_i) / max(1.0, abs(target_i), abs(cross_i))
        slope_err = abs((s1.eng_v - s0.eng_v) - (samples[1].eng_v - samples[0].eng_v)) / max(1.0, abs(samples[1].eng_v - samples[0].eng_v))

        score = period_err + 0.35 * current_err + 0.15 * slope_err

        if best_score is None or score < best_score:
            best_score = score
            best = (i, alpha, cross_t_us, cross_i)

    if best is None:
        # fallback: escolhe o melhor índice bruto próximo de 1 período
        best_i = None
        best_fallback = None
        for i in range(search_left, search_right + 1):
            s = samples[i]
            period_err = abs(float(s.t_us) - target_period_us) / target_period_us
            amp_err = (
                abs(s.eng_v - target_v) / max(1.0, abs(target_v), abs(s.eng_v))
                + abs(s.eng_i - target_i) / max(1.0, abs(target_i), abs(s.eng_i))
            )
            score = period_err + 0.5 * amp_err
            if best_fallback is None or score < best_fallback:
                best_fallback = score
                best_i = i

        assert best_i is not None
        end_index = best_i
        end_t_us = float(samples[best_i].t_us)
        end_v = target_v
        end_i = target_i
    else:
        end_index, _alpha, end_t_us, end_i = best
        end_v = target_v

    cycle: List[SamplePacket] = []

    def make_packet(seq: int, rel_t_us: int, prev_rel_t_us: int, eng_v: float, eng_i: float) -> SamplePacket:
        dt_us = max(0, rel_t_us - prev_rel_t_us)
        clip_v = 1 if abs(eng_v) > v_clip else 0
        clip_i = 1 if abs(eng_i) > i_clip else 0
        flags = (clip_v & 0x01) | ((clip_i & 0x01) << 1)
        dac_v = volts_to_dac12(map_eng_to_volts(eng_v, v_clip))
        dac_i = volts_to_dac12(map_eng_to_volts(eng_i, i_clip))
        return SamplePacket(
            seq=seq,
            dt_us=dt_us,
            dac_v=dac_v,
            dac_i=dac_i,
            flags=flags,
            t_us=rel_t_us,
            eng_v=eng_v,
            eng_i=eng_i,
            clip_v=clip_v,
            clip_i=clip_i,
        )

    # início = primeira amostra real do arquivo, preservado
    prev_rel = 0
    cycle.append(make_packet(
        0,
        0,
        0,
        start.eng_v,
        start.eng_i,
    ))

    seq_counter = 1

    # amostras internas reais
    for idx in range(1, end_index + 1):
        s = samples[idx]
        rel_t = int(round(float(s.t_us) - float(start.t_us)))
        rel_t = max(prev_rel, rel_t)
        cycle.append(make_packet(seq_counter, rel_t, prev_rel, s.eng_v, s.eng_i))
        prev_rel = rel_t
        seq_counter += 1

    # ponto final interpolado/fechado com mesma amplitude do início
    final_rel_t = max(prev_rel, int(round(end_t_us - float(start.t_us))))
    cycle.append(make_packet(
        seq_counter,
        final_rel_t,
        prev_rel,
        end_v,
        end_i,
    ))

    print(
        f"Pré-falta: ciclo detectado com interpolação, "
        f"periodo={final_rel_t/1_000_000.0:.8f}s, "
        f"amostras_no_buffer={len(cycle)}"
    )

    return cycle


# ---------------- SERIAL ----------------

def checksum8(buf: bytes) -> int:
    return sum(buf) & 0xFF


def build_legacy_frame(dt_us: int, dac_v: int, dac_i: int, flags: int) -> bytes:
    dt_us = max(0, min(int(dt_us), 0xFFFFFFFF))
    dac_v = max(0, min(DAC_MAX, int(dac_v)))
    dac_i = max(0, min(DAC_MAX, int(dac_i)))
    flags &= 0xFF

    raw10 = struct.pack(
        "<BIHHB",
        MAGIC0,
        dt_us,
        dac_v,
        dac_i,
        flags,
    )
    chk = checksum8(raw10)
    return raw10 + bytes([chk, MAGIC1])


def wait_esp32_ready(port: str, baud: int, timeout_s: float = 12.0) -> serial.Serial:
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
    seen = []

    while time.time() - t0 < timeout_s:
        raw = ser.readline()
        if not raw:
            continue

        text = raw.decode(errors="ignore").strip()
        if not text:
            continue

        seen.append(text)
        print(f"[ESP] {text}")

        if "READY" in text:
            return ser

    ser.close()
    raise RuntimeError(f"Não recebi READY da ESP32. Recebido: {seen}")


def send_idle_frame(
    ser: serial.Serial,
    logger: TxLogger,
    *,
    mode: str,
    run_id: int,
    event: str,
    reset_clock: bool = True,
) -> None:
    flags = FLAG_IDLE_AFTER
    if reset_clock:
        flags |= FLAG_RESET_CLOCK

    ser.write(build_legacy_frame(0, DAC_MID, DAC_MID, flags))
    ser.flush()

    logger.append(
        mode=mode,
        run_id=run_id,
        event=event,
        seq=-1,
        t_us=0,
        dt_us=0,
        dac_v=DAC_MID,
        dac_i=DAC_MID,
        eng_v=0.0,
        eng_i=0.0,
        clip_v=0,
        clip_i=0,
        flags=flags,
    )


def command_input_thread(ctrl: CommandController) -> None:
    valid = {"s", "f", "fr", "p", "r", "q", "x", "t"}
    while True:
        try:
            raw = input("> ").strip().lower()
        except EOFError:
            ctrl.set("x")
            return
        except KeyboardInterrupt:
            ctrl.set("x")
            return

        if raw not in valid:
            print("Comando inválido. Use: s, f, fr, p, r, q, x")
            continue

        ctrl.set(raw)
        if raw == "x":
            return

def send_sequence(
    ser: serial.Serial,
    packets: List[SamplePacket],
    ctrl: CommandController,
    logger: TxLogger,
    *,
    mode_name: str,
    run_id: int,
    interrupt_commands: Set[str],
    verbose: bool = True,
) -> str:
    if verbose:
        print(f"▶️ Enviando {len(packets)} amostras para ESP32 ({mode_name})...")

    last_print = 0.0

    for idx, s in enumerate(packets, start=1):
        cmd = ctrl.get()
        if cmd in interrupt_commands:
            if cmd == "q":
                send_idle_frame(
                    ser,
                    logger,
                    mode="idle",
                    run_id=run_id,
                    event="idle_after_interrupt",
                    reset_clock=True,
                )
                return "q"
            if cmd == "x":
                send_idle_frame(
                    ser,
                    logger,
                    mode="idle",
                    run_id=run_id,
                    event="idle_before_exit",
                    reset_clock=True,
                )
                return "x"

        ser.write(build_legacy_frame(s.dt_us, s.dac_v, s.dac_i, s.flags))

        logger.append(
            mode=mode_name,
            run_id=run_id,
            event="sample",
            seq=s.seq,
            t_us=s.t_us,
            dt_us=s.dt_us,
            dac_v=s.dac_v,
            dac_i=s.dac_i,
            eng_v=s.eng_v,
            eng_i=s.eng_i,
            clip_v=s.clip_v,
            clip_i=s.clip_i,
            flags=s.flags,
        )

        if (idx % 32) == 0:
            ser.flush()
            time.sleep(0.001)

        now = time.monotonic()
        if verbose and (now - last_print) > 0.25:
            print(f"  ... {idx}/{len(packets)}  (out_waiting={ser.out_waiting or 0})")
            last_print = now

    ser.flush()
    return "done"


def run_interactive(
    ser: serial.Serial,
    full_samples: List[SamplePacket],
    prefault_cycle: List[SamplePacket],
    test_samples,
    logger: TxLogger,
) -> None:
    ctrl = CommandController()
    th = threading.Thread(target=command_input_thread, args=(ctrl,), daemon=True)
    th.start()

    idle_tracker = IdleSpanTracker(logger)
    run_id = 0
    state = STATE_IDLE
    last_state = None

    send_idle_frame(
        ser,
        logger,
        mode="idle",
        run_id=run_id,
        event="idle_after_connect",
        reset_clock=True,
    )
    idle_tracker.start(mode="idle", run_id=run_id, event="idle_span")

    print("ℹ️ Modo idle ativo: saídas em 1,65V.")
    print("Comandos:")
    print("  t  : gerar senoide de teste (60Hz 3V)")
    print("  s  : reproduzir continuamente o primeiro ciclo ajustado (pré-falta)")
    print("  f  : em modo s, ao fim do ciclo atual toca o COMTRADE completo uma vez")
    print("  fr : em modo s, ao fim do ciclo atual toca o COMTRADE completo e depois entra em loop")
    print("  p  : em idle, reproduz o COMTRADE completo uma vez")
    print("  r  : em idle, reproduz o COMTRADE completo continuamente")
    print("  q  : interrompe e volta para idle (1,65V)")
    print("  x  : encerra o programa")

    while True:
        cmd = ctrl.get()

        if cmd == "t":
            state = "test_mode"
            ctrl.clear()
            continue

        if state != last_state:
            if state == STATE_IDLE:
                send_idle_frame(
                    ser,
                    logger,
                    mode="idle",
                    run_id=run_id,
                    event="idle_command",
                    reset_clock=True,
                )
                idle_tracker.restart(mode="idle", run_id=run_id, event="idle_span")
                print("ℹ️ Idle ativo (1,65V).")

            elif state == STATE_PREF_LOOP:
                idle_tracker.stop()
                print("ℹ️ Modo pré-falta contínuo solicitado.")

            elif state == STATE_PLAY_ONCE:
                idle_tracker.stop()
                print("ℹ️ Reprodução completa única solicitada.")

            elif state == STATE_PLAY_REPEAT:
                idle_tracker.stop()
                print("ℹ️ Reprodução completa contínua solicitada.")

            elif state == STATE_EXIT:
                idle_tracker.stop()
                print("ℹ️ Encerrando programa...")

            last_state = state

        if state == STATE_IDLE:
            if cmd is None:
                time.sleep(0.05)
                continue

            if cmd == "q":
                ctrl.clear()
                time.sleep(0.05)
                continue

            if cmd == "x":
                state = STATE_EXIT
                ctrl.clear()
                continue

            if cmd == "s":
                state = STATE_PREF_LOOP
                ctrl.clear()
                continue

            if cmd == "p":
                state = STATE_PLAY_ONCE
                ctrl.clear()
                continue

            if cmd == "r":
                state = STATE_PLAY_REPEAT
                ctrl.clear()
                continue

            ctrl.clear()
            continue

        if state == STATE_PREF_LOOP:
            run_id += 1
            result = send_sequence(
                ser,
                prefault_cycle,
                ctrl,
                logger,
                mode_name="prefault_cycle",
                run_id=run_id,
                interrupt_commands={"q", "x"},
                verbose=False,
            )

            if result == "q":
                idle_tracker.restart(mode="idle", run_id=run_id, event="idle_span")
                ctrl.clear()
                state = STATE_IDLE
                continue

            if result == "x":
                ctrl.clear()
                state = STATE_EXIT
                continue

            cmd = ctrl.get()

            if cmd == "f":
                ctrl.clear()
                state = STATE_PLAY_ONCE
                continue

            if cmd == "fr":
                ctrl.clear()
                state = STATE_PLAY_REPEAT
                continue

            if cmd == "p":
                ctrl.clear()
                state = STATE_PLAY_ONCE
                continue

            if cmd == "r":
                ctrl.clear()
                state = STATE_PLAY_REPEAT
                continue

            if cmd == "q":
                ctrl.clear()
                idle_tracker.restart(mode="idle", run_id=run_id, event="idle_span")
                state = STATE_IDLE
                continue

            if cmd == "x":
                ctrl.clear()
                state = STATE_EXIT
                continue

            continue

        if state == STATE_PLAY_ONCE:
            run_id += 1
            result = send_sequence(
                ser,
                full_samples,
                ctrl,
                logger,
                mode_name="full_once",
                run_id=run_id,
                interrupt_commands={"q", "x"},
            )

            if result == "q":
                idle_tracker.restart(mode="idle", run_id=run_id, event="idle_span")
                ctrl.clear()
                state = STATE_IDLE
                continue

            if result == "x":
                ctrl.clear()
                state = STATE_EXIT
                continue

            send_idle_frame(
                ser,
                logger,
                mode="idle",
                run_id=run_id,
                event="idle_after_cycle",
                reset_clock=True,
            )
            idle_tracker.restart(mode="idle", run_id=run_id, event="idle_span")
            ctrl.clear()
            state = STATE_IDLE
            continue

        if state == STATE_PLAY_REPEAT:
            run_id += 1
            result = send_sequence(
                ser,
                full_samples,
                ctrl,
                logger,
                mode_name="full_repeat",
                run_id=run_id,
                interrupt_commands={"q", "x"},
            )

            if result == "q":
                idle_tracker.restart(mode="idle", run_id=run_id, event="idle_span")
                ctrl.clear()
                state = STATE_IDLE
                continue

            if result == "x":
                ctrl.clear()
                state = STATE_EXIT
                continue

            cmd = ctrl.get()
            if cmd in {"q", "x"}:
                continue
            if cmd is not None:
                ctrl.clear()
            continue

        if state == STATE_EXIT:
            idle_tracker.stop()
            send_idle_frame(
                ser,
                logger,
                mode="idle",
                run_id=run_id,
                event="idle_before_exit",
                reset_clock=True,
            )
            break

        if state == "test_mode":
            run_id += 1

            result = send_sequence(
                ser,
                test_samples,
                ctrl,
                logger,
                mode_name="test_sine",
                run_id=run_id,
                interrupt_commands={"q", "x"},
            )

            if result == "q":
                state = STATE_IDLE
                ctrl.clear()
                continue

            if result == "x":
                state = STATE_EXIT
                ctrl.clear()
                continue

            continue

    idle_tracker.stop()
    print("✅ Programa encerrado.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Lê COMTRADE no PC, processa e envia amostras prontas para a ESP32.")
    parser.add_argument("--cfg", required=True, help="Caminho do arquivo .cfg")
    parser.add_argument("--bdat", required=True, help="Caminho do arquivo .bdat")
    parser.add_argument("--port", required=True, help="Porta serial da ESP32, ex.: /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baudrate serial (default: {DEFAULT_BAUD})")
    parser.add_argument("--out", help="CSV opcional de saída do lado do PC, gravado ao encerrar o programa")
    parser.add_argument("--ready-timeout", type=float, default=12.0, help="Timeout para READY")
    args = parser.parse_args()

    cfg_path = Path(args.cfg)
    bdat_path = Path(args.bdat)

    if not cfg_path.exists():
        raise SystemExit(f"CFG não encontrado: {cfg_path}")
    if not bdat_path.exists():
        raise SystemExit(f"BDAT não encontrado: {bdat_path}")

    cfg = parse_cfg(cfg_path)
    compute_bdat_layout(cfg, bdat_path)

    full_samples, v_clip, i_clip = build_samples(cfg, bdat_path)
    prefault_cycle = build_prefault_cycle_buffer(full_samples, cfg, v_clip, i_clip)

    logger = TxLogger()
    test_samples = build_test_sine()

    ser = wait_esp32_ready(args.port, args.baud, timeout_s=args.ready_timeout)
    try:
        run_interactive(ser, full_samples, prefault_cycle, test_samples, logger)
    finally:
        try:
            send_idle_frame(
                ser,
                logger,
                mode="idle",
                run_id=0,
                event="idle_in_finally",
                reset_clock=True,
            )
        except Exception:
            pass

        try:
            ser.close()
        except Exception:
            pass

        if args.out:
            out_path = Path(args.out)
            logger.save_csv(out_path)
            print(f"✅ CSV salvo ao encerrar o programa em: {out_path}")
        else:
            print("ℹ️ CSV de saída não solicitado.")


if __name__ == "__main__":
    main()
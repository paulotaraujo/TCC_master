#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import struct
import time
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial não está instalado. Instale com: pip install pyserial") from exc

# ============================================================
# Protocolo antigo (compatível com generator.py / v_0.ino)
# [0]   0xA5
# [1:4] dt_us (uint32 LE)
# [5:6] dac0
# [7:8] dac1
# [9:10] dac2
# [11:12] dac3
# [13]  flags
# [14]  checksum = sum(buf[0:14]) & 0xFF
# [15]  0x5A
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
REPEAT_GAP_S = 0.0


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
    """
    Rastreia intervalos de idle apenas no log do Python.
    Não envia nada extra para a ESP32. Só mede o tempo real
    em que o sistema ficou em 1.65V entre transmissões.
    """
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

    idx += 2  # start / trigger

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
            sample_num = struct.unpack_from("<I", rec, off)[0]
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

            yield sample_num, ts, eng_v, eng_i


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

    for idx, (_, _, eng_v, eng_i) in enumerate(iter_bdat_records(cfg, bdat_path)):
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

    v_nom = sum(peaks_v) / len(peaks_v)
    i_nom = sum(peaks_i) / len(peaks_i)
    v_nom = max(v_nom, 1.0)
    i_nom = max(i_nom, 1.0)
    v_clip = v_nom * V_FAULT_LIMIT_MULT
    i_clip = i_nom * I_FAULT_LIMIT_MULT
    return v_nom, i_nom, v_clip, i_clip


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


def build_samples(cfg: ComtradeCfg, bdat_path: Path) -> List[SamplePacket]:
    v_nom, i_nom, v_clip, i_clip = compute_nominal_peaks(cfg, bdat_path)
    print(f"CFG: nA={cfg.n_analog} nD={cfg.n_digital} fs={cfg.sample_rate_hz:.2f}Hz f={cfg.freq_hz:.2f}Hz timeMult={cfg.time_mult}")
    print(f"Nominal(5 ciclos): V_nom_peak={v_nom:.6f} I_nom_peak={i_nom:.6f}")
    print(f"Limites: V_clip={v_clip:.6f} ({V_FAULT_LIMIT_MULT:.1f}x) I_clip={i_clip:.6f} ({I_FAULT_LIMIT_MULT:.1f}x)")
    print(f"Layout: ts64={cfg.ts64} recSize={cfg.rec_size} total={cfg.total_records}")

    samples: List[SamplePacket] = []
    prev_ts: Optional[int] = None
    t_accum = 0

    for seq, (_, ts, eng_v, eng_i) in enumerate(iter_bdat_records(cfg, bdat_path)):
        if prev_ts is None:
            dt_us = 0
        else:
            dts = max(0, ts - prev_ts)
            dt_us = int(round(float(dts) * cfg.time_mult))
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
    return samples


# ---------------- SERIAL ----------------

def checksum8(buf: bytes) -> int:
    return sum(buf) & 0xFF


def build_legacy_frame(dt_us: int, dac_v: int, dac_i: int, flags: int) -> bytes:
    dt_us = max(0, min(int(dt_us), 0xFFFFFFFF))
    dac_v = max(0, min(DAC_MAX, int(dac_v)))
    dac_i = max(0, min(DAC_MAX, int(dac_i)))
    flags &= 0xFF

    raw14 = struct.pack(
        "<BI4HB",
        MAGIC0,
        dt_us,
        dac_v,
        dac_i,
        DAC_MID,
        DAC_MID,
        flags,
    )
    chk = checksum8(raw14)
    return raw14 + bytes([chk, MAGIC1])


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

    # Registra somente a transição/comando de idle.
    # A duração real do idle entra pelo IdleSpanTracker.
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


class CommandController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._command: str = "idle"

    def set(self, cmd: str) -> None:
        with self._lock:
            self._command = cmd

    def get(self) -> str:
        with self._lock:
            return self._command

    def is_stop_requested(self) -> bool:
        with self._lock:
            return self._command in ("idle", "exit")


def command_input_thread(ctrl: CommandController) -> None:
    while True:
        try:
            raw = input("> ").strip().lower()
        except EOFError:
            ctrl.set("exit")
            return
        except KeyboardInterrupt:
            ctrl.set("exit")
            return

        if raw not in {"s", "r", "q", "x"}:
            print("Comando inválido. Use: s, r, q, x")
            continue

        if raw == "s":
            ctrl.set("single")
        elif raw == "r":
            ctrl.set("repeat")
        elif raw == "q":
            ctrl.set("idle")
        elif raw == "x":
            ctrl.set("exit")
            return


def send_one_cycle(
    ser: serial.Serial,
    samples: List[SamplePacket],
    ctrl: CommandController,
    logger: TxLogger,
    *,
    mode_name: str,
    run_id: int,
) -> str:
    print(f"▶️ Enviando {len(samples)} amostras para ESP32 (protocolo legado 16 bytes/frame)...")
    last_print = 0.0

    for idx, s in enumerate(samples, start=1):
        if ctrl.is_stop_requested():
            send_idle_frame(
                ser,
                logger,
                mode="idle",
                run_id=run_id,
                event="idle_after_interrupt",
                reset_clock=True,
            )
            return ctrl.get()

        frame = build_legacy_frame(s.dt_us, s.dac_v, s.dac_i, s.flags)
        ser.write(frame)

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
        if (now - last_print) > 0.25:
            print(f"  ... {idx}/{len(samples)}  (out_waiting={ser.out_waiting or 0})")
            last_print = now

    ser.flush()
    send_idle_frame(
        ser,
        logger,
        mode="idle",
        run_id=run_id,
        event="idle_after_cycle",
        reset_clock=True,
    )
    print("✅ Ciclo concluído. Retornando ao modo idle.")
    return ctrl.get()


def wait_gap_with_interrupt(ctrl: CommandController, seconds: float) -> str:
    t0 = time.monotonic()
    while (time.monotonic() - t0) < seconds:
        cmd = ctrl.get()
        if cmd != "repeat":
            return cmd
        time.sleep(0.05)
    return ctrl.get()


def run_interactive(ser: serial.Serial, samples: List[SamplePacket], logger: TxLogger) -> None:
    ctrl = CommandController()
    th = threading.Thread(target=command_input_thread, args=(ctrl,), daemon=True)
    th.start()

    idle_tracker = IdleSpanTracker(logger)
    run_id = 0

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
    print("  s : executar COMTRADE uma vez")
    print("  r : repetir continuamente (1 s entre execuções)")
    print("  q : parar transmissão e voltar para idle")
    print("  x : encerrar programa")

    last_state = "idle"

    while True:
        cmd = ctrl.get()

        if cmd != last_state:
            if cmd == "idle":
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

            elif cmd == "single":
                idle_tracker.stop()
                print("ℹ️ Modo single solicitado.")

            elif cmd == "repeat":
                idle_tracker.stop()
                print("ℹ️ Modo repeat solicitado.")

            elif cmd == "exit":
                idle_tracker.stop()
                print("ℹ️ Encerrando programa...")

            last_state = cmd

        if cmd == "exit":
            send_idle_frame(
                ser,
                logger,
                mode="idle",
                run_id=run_id,
                event="idle_before_exit",
                reset_clock=True,
            )
            break

        if cmd == "idle":
            time.sleep(0.05)
            continue

        if cmd == "single":
            run_id += 1
            final_cmd = send_one_cycle(
                ser,
                samples,
                ctrl,
                logger,
                mode_name="single",
                run_id=run_id,
            )

            # Sempre volta a medir o idle assim que o ciclo termina/interrompe.
            idle_tracker.restart(mode="idle", run_id=run_id, event="idle_span")

            if final_cmd == "exit":
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

            if ctrl.get() == "single":
                ctrl.set("idle")
            continue

        if cmd == "repeat":
            run_id += 1
            final_cmd = send_one_cycle(
                ser,
                samples,
                ctrl,
                logger,
                mode_name="repeat",
                run_id=run_id,
            )

            # Se saiu do ciclo, volta ao estado idle e passa a medir esse trecho.
            idle_tracker.restart(mode="idle", run_id=run_id, event="repeat_gap_idle")

            if final_cmd == "exit":
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

            if ctrl.get() != "repeat":
                continue

            print(f"ℹ️ Aguardando {REPEAT_GAP_S:.1f} s para próxima execução...")
            gap_cmd = wait_gap_with_interrupt(ctrl, REPEAT_GAP_S)

            # Fecha o trecho real de idle do gap.
            idle_tracker.stop()

            if gap_cmd == "exit":
                send_idle_frame(
                    ser,
                    logger,
                    mode="idle",
                    run_id=run_id,
                    event="idle_before_exit",
                    reset_clock=True,
                )
                break

            if gap_cmd != "repeat":
                # Entrou em idle ou single. Se entrou em idle, o bloco de mudança
                # de estado acima reinicia a medição do idle corretamente.
                continue

            # Vai repetir: não precisa mandar idle extra, só segue.
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
    samples = build_samples(cfg, bdat_path)

    logger = TxLogger()

    ser = wait_esp32_ready(args.port, args.baud, timeout_s=args.ready_timeout)
    try:
        run_interactive(ser, samples, logger)
    finally:
        try:
            # Apenas garante a saída em idle ao finalizar.
            # Isso não é usado para medir duração; a duração já foi registrada
            # pelo IdleSpanTracker dentro do loop interativo.
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
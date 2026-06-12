#!/usr/bin/env python3
"""
Uploader/terminal for v_6.ino.

The ESP32 now owns COMTRADE parsing, scaling and frame generation. This script
only uploads the selected .cfg/.bdat files over USB and forwards playback
commands after the firmware reports READY_PLAYBACK.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Optional

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial nao esta instalado. Instale com: pip install pyserial") from exc


DEFAULT_BAUD = 921600
CHUNK_SIZE = 256
DEFAULT_UPLOAD_DELAY_S = 0.0
VALID_COMMANDS = {"s", "f", "p", "q", "x", "t"}
DAC_V_AMP = 1.55


def open_esp32(port: str, baud: int, ready_timeout: float) -> serial.Serial:
    ser = serial.Serial(port, baudrate=baud, timeout=0.2, write_timeout=5.0)
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass

    time.sleep(2.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    deadline = time.monotonic() + ready_timeout
    seen: list[str] = []
    while time.monotonic() < deadline:
        line = read_line(ser, timeout=0.5)
        if not line:
            continue
        seen.append(line)
        print(f"[ESP32] {line}")
        if line.startswith("READY_UPLOAD") or line.startswith("READY_PLAYBACK"):
            return ser

    ser.close()
    raise RuntimeError(f"Nao recebi READY da ESP32. Recebido: {seen}")


def read_line(ser: serial.Serial, timeout: float) -> Optional[str]:
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


def wait_for_prefix(ser: serial.Serial, prefix: str, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    last_lines: list[str] = []
    while time.monotonic() < deadline:
        line = read_line(ser, timeout=0.5)
        if not line:
            continue
        print(f"[ESP32] {line}")
        last_lines.append(line)
        if line.startswith(prefix):
            return line
        if line.startswith("ERR "):
            raise RuntimeError(line)
    raise TimeoutError(f"Timeout esperando {prefix!r}. Ultimas linhas: {last_lines[-8:]}")


def upload_file(ser: serial.Serial, kind: str, path: Path, upload_delay_s: float) -> None:
    size = path.stat().st_size
    print(f"Enviando {kind}: {path} ({size} bytes)")
    ser.write(f"UPLOAD {kind} {size}\n".encode("ascii"))
    ser.flush()

    wait_for_prefix(ser, f"READY_RECEIVE {kind}", timeout=10.0)

    sent = 0
    last_print = 0.0
    with path.open("rb") as file:
        while True:
            chunk = file.read(CHUNK_SIZE)
            if not chunk:
                break
            ser.write(chunk)
            sent += len(chunk)
            wait_for_prefix(ser, f"RX {sent}", timeout=20.0)
            if upload_delay_s > 0.0:
                time.sleep(upload_delay_s)
            now = time.monotonic()
            if now - last_print > 0.25:
                print(f"  ... {sent}/{size} bytes")
                last_print = now
    ser.flush()

    wait_for_prefix(ser, f"OK {kind}", timeout=20.0)


def parse_status_fields(line: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for part in line.split()[2:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value
    return fields


def process_on_esp32(ser: serial.Serial, timeout: float) -> dict[str, dict[str, str]]:
    print("Solicitando processamento local na ESP32...")
    ser.write(b"PROCESS\n")
    ser.flush()
    deadline = time.monotonic() + timeout
    status: dict[str, dict[str, str]] = {}
    last_lines: list[str] = []

    while time.monotonic() < deadline:
        line = read_line(ser, timeout=0.5)
        if not line:
            continue
        print(f"[ESP32] {line}")
        last_lines.append(line)
        if line.startswith("STATUS CFG "):
            status["cfg"] = parse_status_fields(line)
        elif line.startswith("STATUS SCALE "):
            status["scale"] = parse_status_fields(line)
        elif line.startswith("READY_PLAYBACK"):
            status["ready"] = parse_status_fields("STATUS READY " + line[len("READY_PLAYBACK "):])
            return status
        elif line.startswith("ERR "):
            raise RuntimeError(line)

    raise TimeoutError(f"Timeout esperando READY_PLAYBACK. Ultimas linhas: {last_lines[-8:]}")


def write_receiver_config(out_path: Path, cfg_path: Path, bdat_path: Path, status: dict[str, dict[str, str]]) -> None:
    cfg = status.get("cfg", {})
    scale = status.get("scale", {})

    required = {"vNom", "iNom", "vClip", "iClip"}
    missing = sorted(required - set(scale))
    if missing:
        raise RuntimeError(f"ESP32 nao informou campos de escala: {missing}")

    v_nom = float(scale["vNom"])
    i_nom = float(scale["iNom"])
    v_nom_rms = float(scale.get("vNomRms", v_nom / math.sqrt(2.0)))
    i_nom_rms = float(scale.get("iNomRms", i_nom / math.sqrt(2.0)))
    v_clip = float(scale["vClip"])
    i_clip = float(scale["iClip"])

    payload = {
        "schema_version": 2,
        "generator_script": "generate_signal_v6.py",
        "processing_side": "esp32",
        "generated_unix_s": time.time(),
        "comtrade": {
            "cfg_path": str(cfg_path),
            "bdat_path": str(bdat_path),
            "n_analog": int(cfg.get("nA", "0")),
            "n_digital": int(cfg.get("nD", "0")),
            "freq_hz": float(cfg.get("f", "60.0")),
            "sample_rate_hz": float(cfg.get("fs", "0.0")),
            "records": int(cfg.get("records", "0")),
            "rec_size": int(cfg.get("recSize", "0")),
            "ts64": cfg.get("ts64", "0") == "1",
            "data_format": "BINARY",
        },
        "dac_mapping": {
            "v_mid": 1.65,
            "v_amp": DAC_V_AMP,
            "dac_vref": 3.3,
            "dac_max": 4095,
            "dac_mid_code": 2048,
            "protocol_channels": {
                "voltage": "dac_v",
                "current": "dac_i",
            },
        },
        "comtrade_reference": {
            "v_nom_peak": v_nom,
            "i_nom_peak": i_nom,
            "v_nom_rms": v_nom_rms,
            "i_nom_rms": i_nom_rms,
            "v_clip_peak": v_clip,
            "i_clip_peak": i_clip,
            "v_clip_rms": v_clip / math.sqrt(2.0),
            "i_clip_rms": i_clip / math.sqrt(2.0),
        },
        "receiver_recommendation": {
            "channel_map": {
                "gpio34": "voltage",
                "gpio35": "current",
            },
            "adc_vref": 3.3,
            "offset_v": 1.65,
            "offset_i": 1.65,
            "v_scale_eng_per_volt": v_clip / DAC_V_AMP if DAC_V_AMP > 0 else 1.0,
            "i_scale_eng_per_volt": i_clip / DAC_V_AMP if DAC_V_AMP > 0 else 1.0,
            "freq_hz": float(cfg.get("f", "60.0")),
            "sample_rate_hz": float(cfg.get("fs", "0.0")),
        },
    }

    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Receiver config salvo em: {out_path}")


def reader_thread(ser: serial.Serial, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            line = read_line(ser, timeout=0.2)
        except Exception as exc:
            print(f"[serial] leitura encerrada: {exc}")
            stop.set()
            return
        if line:
            print(f"[ESP32] {line}")


def command_loop(ser: serial.Serial) -> None:
    stop = threading.Event()
    thread = threading.Thread(target=reader_thread, args=(ser, stop), daemon=True)
    thread.start()

    print("Comandos liberados:")
    print("  t  : senoide de teste")
    print("  s  : ciclo pre-falta continuo")
    print("  f  : no modo s, toca COMTRADE completo uma vez")
    print("  p  : toca COMTRADE completo uma vez")
    print("  q  : interrompe e volta para idle")
    print("  x  : encerra este terminal")

    try:
        while not stop.is_set():
            try:
                cmd = input("> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                cmd = "x"

            if not cmd:
                continue
            if cmd not in VALID_COMMANDS:
                print("Comando invalido. Use: s, f, p, q, x, t")
                continue

            ser.write(f"{cmd}\n".encode("ascii"))
            ser.flush()
            if cmd == "x":
                stop.set()
                break
    finally:
        stop.set()
        thread.join(timeout=1.0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Envia COMTRADE para a ESP32 v_6 e encaminha comandos de reproducao."
    )
    parser.add_argument("--cfg", required=True, help="Caminho do arquivo .cfg")
    parser.add_argument("--bdat", required=True, help="Caminho do arquivo .bdat/.dat binario")
    parser.add_argument("--port", required=True, help="Porta serial da ESP32, ex.: /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baudrate serial (default: {DEFAULT_BAUD})")
    parser.add_argument("--ready-timeout", type=float, default=12.0, help="Timeout para READY_UPLOAD")
    parser.add_argument("--process-timeout", type=float, default=60.0, help="Timeout para PROCESS/READY_PLAYBACK")
    parser.add_argument(
        "--receiver-config",
        help="Caminho para salvar JSON de escala da recepcao. Default: '<nome_cfg>_receiver_config.json'",
    )
    parser.add_argument(
        "--upload-delay",
        type=float,
        default=DEFAULT_UPLOAD_DELAY_S,
        help="Pausa adicional entre chunks de upload em segundos (default: 0.0)",
    )
    parser.add_argument("--skip-upload", action="store_true", help="Nao envia arquivos; apenas manda PROCESS e abre comandos")
    args = parser.parse_args()

    cfg_path = Path(args.cfg)
    bdat_path = Path(args.bdat)
    if not cfg_path.exists():
        raise SystemExit(f"CFG nao encontrado: {cfg_path}")
    if not bdat_path.exists():
        raise SystemExit(f"BDAT nao encontrado: {bdat_path}")

    ser = open_esp32(args.port, args.baud, args.ready_timeout)
    try:
        if not args.skip_upload:
            upload_file(ser, "CFG", cfg_path, args.upload_delay)
            upload_file(ser, "BDAT", bdat_path, args.upload_delay)
        status = process_on_esp32(ser, args.process_timeout)
        receiver_config_path = Path(args.receiver_config) if args.receiver_config else Path.cwd() / f"{cfg_path.stem}_receiver_config.json"
        write_receiver_config(receiver_config_path, cfg_path, bdat_path, status)
        command_loop(ser)
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        raise SystemExit(1)

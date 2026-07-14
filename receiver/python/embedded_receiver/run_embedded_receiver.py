#!/usr/bin/env python3
"""Run the ESP32 embedded receiver like the old Python receiver.

This script is the terminal-facing pipeline:
  configure ESP32 over serial -> start acquisition/protection -> wait Ctrl+C
  -> stop.
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
import time
from pathlib import Path
from typing import Optional

import serial


DEFAULT_BAUD = 921600
STATUS_KV_RE = re.compile(r"\b([A-Za-z0-9_]+)=([^\s]+)")


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


def drain_serial(ser: serial.Serial, quiet_s: float = 0.4, max_s: float = 3.0) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + max_s
    quiet_deadline = time.monotonic() + quiet_s
    while time.monotonic() < deadline:
        line = read_line(ser, 0.05)
        if line:
            lines.append(line)
            quiet_deadline = time.monotonic() + quiet_s
            continue
        if time.monotonic() >= quiet_deadline:
            break
    return lines


def wait_ready(ser: serial.Serial, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    seen: list[str] = []
    while time.monotonic() < deadline:
        line = read_line(ser, 0.5)
        if not line:
            continue
        seen.append(line)
        print(f"[ESP32] {line}")
        if line.startswith("READY_EMBEDDED_RX"):
            drain_serial(ser, quiet_s=0.5, max_s=3.0)
            return
    if seen:
        drain_serial(ser, quiet_s=0.5, max_s=3.0)
        return
    print("[aviso] não recebi READY; tentando configurar assim mesmo.")


def send_cmd(
    ser: serial.Serial,
    cmd: str,
    expect_prefix: str | None = None,
    timeout: float = 8.0,
    drain_before: bool = True,
) -> None:
    if drain_before:
        drain_serial(ser, quiet_s=0.8, max_s=5.0)
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()
    if expect_prefix is None:
        return

    deadline = time.monotonic() + timeout
    last: list[str] = []
    while time.monotonic() < deadline:
        line = read_line(ser, 0.25)
        if not line:
            continue
        last.append(line)
        if line.startswith(expect_prefix):
            return
        if line.startswith("ERR"):
            raise RuntimeError(line)
    raise TimeoutError(f"Timeout esperando {expect_prefix!r} após {cmd!r}. Últimas linhas: {last[-5:]}")


def send_cfg(
    ser: serial.Serial,
    command: str,
    timeout: float = 8.0,
    drain_before: bool = False,
) -> None:
    parts = command.split(maxsplit=2)
    if len(parts) < 3 or parts[0] != "cfg":
        raise ValueError(f"Comando cfg inválido: {command}")
    key = parts[1]

    if drain_before:
        drain_serial(ser, quiet_s=0.8, max_s=5.0)
    ser.write((command + "\n").encode("ascii"))
    ser.flush()

    deadline = time.monotonic() + timeout
    last: list[str] = []
    expected_prefix = f"ACK cfg {key}="
    while time.monotonic() < deadline:
        line = read_line(ser, 0.25)
        if not line:
            continue
        last.append(line)
        if line.startswith(expected_prefix):
            return
        if line.startswith("ERR"):
            raise RuntimeError(line)
    raise TimeoutError(
        f"Timeout esperando {expected_prefix!r} após {command!r}. Últimas linhas: {last[-8:]}"
    )


def cfg_cmd(key: str, value: object) -> str:
    if isinstance(value, bool):
        value = "1" if value else "0"
    return f"cfg {key} {value}"


def parse_status(line: str) -> dict[str, str]:
    if line.startswith("STATUS "):
        line = line.removeprefix("STATUS ")
    return dict(STATUS_KV_RE.findall(line))


def load_config(path: Path) -> tuple[dict, dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"JSON inválido: {path}")
    rec = payload.get("receiver_recommendation", {})
    cref = payload.get("comtrade_reference", {})
    if not isinstance(rec, dict) or not isinstance(cref, dict):
        raise SystemExit("Config precisa conter receiver_recommendation e comtrade_reference.")
    return rec, cref


def build_config_commands(args: argparse.Namespace) -> list[str]:
    rec, cref = load_config(Path(args.config))

    freq = float(rec.get("freq_hz", 60.0))
    v_scale = float(rec["v_scale_eng_per_volt"]) * float(args.v_scale_correction)
    i_scale = float(rec["i_scale_eng_per_volt"]) * float(args.i_scale_correction)
    commands = [
        cfg_cmd("sample_rate_hz", int(round(float(rec.get("sample_rate_hz", 1000.0))))),
        cfg_cmd("v_scale", v_scale),
        cfg_cmd("i_scale", i_scale),
        cfg_cmd("v_nominal_rms", float(cref["v_nom_rms"])),
        cfg_cmd("i_nominal_rms", float(cref["i_nom_rms"])),
        cfg_cmd("freq_ref_hz", freq),
        cfg_cmd("f_min_hz", min(55.0, 0.9 * freq)),
        cfg_cmd("f_max_hz", max(65.0, 1.1 * freq)),
        cfg_cmd("normalize_to_comtrade", bool(args.normalize_to_comtrade)),
        cfg_cmd("norm_min_pu", float(args.norm_min_pu)),
    ]

    commands.append(cfg_cmd("oc_enabled", args.over_current is not None))
    if args.over_current is not None:
        oc50, oc51, delay = args.over_current
        if oc50 <= oc51 or oc51 < 0.0 or delay < 0.0:
            raise ValueError("--over-current exige INST_RISE_PCT > TIMED_RISE_PCT >= 0 e DELAY_S >= 0")
        commands += [
            cfg_cmd("oc_51_pct", oc51),
            cfg_cmd("oc_50_pct", oc50),
            cfg_cmd("oc_51_delay_s", delay),
        ]

    commands.append(cfg_cmd("dist_enabled", args.distance is not None))
    if args.distance is not None:
        line_z, z1_pct, z2_pct, delay = args.distance
        commands += [
            cfg_cmd("dist_line_z_ohm", line_z),
            cfg_cmd("dist_z1_pct", z1_pct),
            cfg_cmd("dist_z2_pct", z2_pct),
            cfg_cmd("dist_z2_delay_s", delay),
        ]
    commands.append(cfg_cmd("dist_line_angle_deg", float(args.distance_line_angle)))

    commands.append(cfg_cmd("dir67_enabled", args.directional_67 is not None))
    if args.directional_67 is not None:
        commands.append(cfg_cmd("dir67_forward", args.directional_67 == "forward"))
    commands.append(cfg_cmd("dir67_power_min_w", float(args.directional_67_power_min)))

    commands.append(cfg_cmd("uv_enabled", args.under_voltage is not None))
    if args.under_voltage is not None:
        instant_level, timed_drop, delay = args.under_voltage
        if not (0.0 < instant_level < 100.0 - timed_drop < 100.0) or delay < 0.0:
            raise ValueError("--under-voltage exige 0 < INST_LEVEL_PCT < 100-TIMED_DROP_PCT < 100 e DELAY_S >= 0")
        commands += [
            cfg_cmd("uv_inst_level_pct", instant_level),
            cfg_cmd("uv_timed_drop_pct", timed_drop),
            cfg_cmd("uv_delay_s", delay),
        ]

    commands.append(cfg_cmd("ov_enabled", args.over_voltage is not None))
    if args.over_voltage is not None:
        instant_rise, timed_rise, delay = args.over_voltage
        if instant_rise <= timed_rise or timed_rise < 0.0 or delay < 0.0:
            raise ValueError("--over-voltage exige INST_RISE_PCT > TIMED_RISE_PCT >= 0 e DELAY_S >= 0")
        commands += [
            cfg_cmd("ov_inst_rise_pct", instant_rise),
            cfg_cmd("ov_timed_rise_pct", timed_rise),
            cfg_cmd("ov_delay_s", delay),
        ]

    commands.append(cfg_cmd("protection_events", bool(args.protection_events)))
    commands.append(cfg_cmd("protection_event_interval_s", float(args.protection_event_interval)))

    return commands


def build_calibration_commands(args: argparse.Namespace) -> list[str]:
    rec, cref = load_config(Path(args.config))

    freq = float(rec.get("freq_hz", 60.0))
    v_scale = float(rec["v_scale_eng_per_volt"]) * float(args.v_scale_correction)
    i_scale = float(rec["i_scale_eng_per_volt"]) * float(args.i_scale_correction)
    return [
        cfg_cmd("sample_rate_hz", int(round(float(rec.get("sample_rate_hz", 1000.0))))),
        cfg_cmd("v_scale", v_scale),
        cfg_cmd("i_scale", i_scale),
        cfg_cmd("v_nominal_rms", float(cref["v_nom_rms"])),
        cfg_cmd("i_nominal_rms", float(cref["i_nom_rms"])),
        cfg_cmd("freq_ref_hz", freq),
        cfg_cmd("f_min_hz", min(55.0, 0.9 * freq)),
        cfg_cmd("f_max_hz", max(65.0, 1.1 * freq)),
        cfg_cmd("normalize_to_comtrade", False),
        cfg_cmd("norm_min_pu", float(args.norm_min_pu)),
        cfg_cmd("oc_enabled", False),
        cfg_cmd("dist_enabled", False),
        cfg_cmd("dir67_enabled", False),
        cfg_cmd("uv_enabled", False),
        cfg_cmd("ov_enabled", False),
        cfg_cmd("protection_events", False),
    ]


def auto_calibrate_scales(ser: serial.Serial, args: argparse.Namespace) -> None:
    _rec, cref = load_config(Path(args.config))
    target_v = float(cref["v_nom_rms"])
    target_i = float(cref["i_nom_rms"])

    print("Auto-calibração: aplicando configuração sem trips...")
    if not args.legacy_config_drain:
        drain_serial(ser, quiet_s=0.8, max_s=5.0)
    for command in build_calibration_commands(args):
        send_cfg(ser, command, drain_before=args.legacy_config_drain)

    send_cmd(ser, "resettrip", "OK resettrip")
    send_cmd(ser, "start", "OK start")
    print("Auto-calibração: medindo pré-falta. Mantenha a geradora em modo 's'.")

    samples_v: list[float] = []
    samples_i: list[float] = []
    t0 = time.monotonic()
    next_status = t0
    deadline = t0 + float(args.auto_scale_seconds)
    settle_until = t0 + float(args.auto_scale_settle)

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_status:
            send_cmd(ser, "status", expect_prefix=None, drain_before=False)
            next_status = now + 0.15

        line = read_line(ser, 0.05)
        if not line:
            continue
        if line.startswith("STATUS "):
            fields = parse_status(line)
            if now < settle_until or fields.get("valid") != "1":
                continue
            try:
                v1 = float(fields["V1"])
                i1 = float(fields["I1"])
            except (KeyError, ValueError):
                continue
            if v1 > 0.0 and i1 > 0.0:
                samples_v.append(v1)
                samples_i.append(i1)
        elif line.startswith("ERR"):
            print(f"[ESP32] {line}")

    send_cmd(ser, "stop", "OK stop")

    if len(samples_v) < args.auto_scale_min_samples or len(samples_i) < args.auto_scale_min_samples:
        raise RuntimeError(
            "Auto-calibração falhou: poucas amostras válidas. "
            "Confirme que a geradora está em pré-falta contínua e tente aumentar --auto-scale-seconds."
        )

    measured_v = statistics.median(samples_v)
    measured_i = statistics.median(samples_i)
    if measured_v <= 0.0 or measured_i <= 0.0:
        raise RuntimeError("Auto-calibração falhou: medição inválida de V1/I1.")

    corr_v = target_v / measured_v
    corr_i = target_i / measured_i
    args.v_scale_correction *= corr_v
    args.i_scale_correction *= corr_i

    print(
        "Auto-calibração concluída: "
        f"V1_med={measured_v:.3f}V alvo={target_v:.3f}V fator_v={corr_v:.6f}; "
        f"I1_med={measured_i:.6f}A alvo={target_i:.6f}A fator_i={corr_i:.6f}."
    )
    print(
        "Correções efetivas: "
        f"--v-scale-correction {args.v_scale_correction:.6f} "
        f"--i-scale-correction {args.i_scale_correction:.6f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Recepção embarcada ESP32 com configuração e execução em um comando."
    )
    parser.add_argument("--config", required=True, help="config.json do ensaio")
    parser.add_argument("--port", default="/dev/ttyUSB1")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--ready-timeout", type=float, default=5.0)
    parser.add_argument("--normalize-to-comtrade", action="store_true")
    parser.add_argument("--norm-min-pu", type=float, default=0.5)
    parser.add_argument(
        "--v-scale-correction",
        type=float,
        default=1.0,
        help="Multiplica a escala de tensão v_scale do config.json antes de enviá-la à ESP32.",
    )
    parser.add_argument(
        "--i-scale-correction",
        type=float,
        default=1.0,
        help="Multiplica a escala de corrente i_scale do config.json antes de enviá-la à ESP32.",
    )
    parser.add_argument(
        "--auto-scale-calibrate",
        action="store_true",
        help=(
            "Antes de armar as proteções, mede V1/I1 em pré-falta e ajusta automaticamente "
            "v_scale/i_scale para bater com os nominais do config.json. A geradora deve estar em modo 's'."
        ),
    )
    parser.add_argument(
        "--auto-scale-seconds",
        type=float,
        default=2.0,
        help="Duração da medição de pré-falta usada por --auto-scale-calibrate.",
    )
    parser.add_argument(
        "--auto-scale-settle",
        type=float,
        default=0.5,
        help="Tempo inicial ignorado na auto-calibração para estabilização de offset/ciclos.",
    )
    parser.add_argument(
        "--auto-scale-min-samples",
        type=int,
        default=5,
        help="Número mínimo de STATUS válidos necessário para aceitar a auto-calibração.",
    )
    parser.add_argument("--over-current", nargs=3, type=float, metavar=("INST_50_RISE_PCT", "TIMED_51_RISE_PCT", "DELAY_51_S"))
    parser.add_argument("--distance", nargs=4, type=float, metavar=("LINE_Z_OHM", "Z1_PCT", "Z2_PCT", "DELAY_Z2_S"))
    parser.add_argument("--distance-line-angle", type=float, default=0.0)
    parser.add_argument("--directional-67", choices=("forward", "reverse"))
    parser.add_argument("--directional-67-power-min", type=float, default=10000.0)
    parser.add_argument("--under-voltage", nargs=3, type=float, metavar=("INST_LEVEL_PCT", "TIMED_DROP_PCT", "DELAY_S"))
    parser.add_argument("--over-voltage", nargs=3, type=float, metavar=("INST_RISE_PCT", "TIMED_RISE_PCT", "DELAY_S"))
    parser.add_argument(
        "--protection-events",
        action="store_true",
        help="Imprime eventos de proteção recebidos da ESP32: pickup, reset, timing e trip.",
    )
    parser.add_argument(
        "--protection-event-interval",
        type=float,
        default=0.05,
        help="Intervalo mínimo entre mensagens TIMING de proteções temporizadas.",
    )
    parser.add_argument(
        "--legacy-config-drain",
        action="store_true",
        help="Usa a limpeza serial antiga antes de cada parâmetro de configuração.",
    )
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=2.0) as ser:
        try:
            ser.setDTR(False)
            ser.setRTS(False)
        except Exception:
            pass
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        wait_ready(ser, args.ready_timeout)
        send_cmd(ser, "stop", "OK stop")
        send_cmd(ser, "resettrip", "OK resettrip")

        if args.auto_scale_calibrate:
            auto_calibrate_scales(ser, args)
            send_cmd(ser, "resettrip", "OK resettrip")

        print("Aplicando configuração na ESP32...")
        commands = build_config_commands(args)
        if not args.legacy_config_drain:
            drain_serial(ser, quiet_s=0.8, max_s=5.0)
        for command in commands:
            send_cfg(ser, command, drain_before=args.legacy_config_drain)
        print(f"Configuração aplicada ({len(commands)} parâmetros).")

        send_cmd(ser, "start", "OK start")
        print("Leitura/proteção em execução. Pressione Ctrl+C para parar.")

        try:
            while True:
                line = read_line(ser, 0.2)
                if line and line.startswith("EVENT "):
                    if args.protection_events:
                        print(line.removeprefix("EVENT "))
                elif line and (line.startswith("ERR") or line.startswith("STATUS")):
                    print(f"[ESP32] {line}")
        except KeyboardInterrupt:
            print("\nInterrupção recebida. Parando ESP32...")

        send_cmd(ser, "stop", "OK stop")

    print("Recepção encerrada. Trips permanecem retidos até resettrip ou reinício da ESP32.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        raise SystemExit(1)

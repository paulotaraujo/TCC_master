#!/usr/bin/env python3
"""Configure embedded_receiver.ino from config.json and protection CLI arguments."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


SKETCH_PATH = Path(__file__).with_name("embedded_receiver.ino")


def bool_cpp(value: bool) -> str:
    return "true" if value else "false"


def float_cpp(value: float) -> str:
    text = f"{float(value):.9g}"
    if "e" not in text.lower() and "." not in text:
        text += ".0"
    return f"{text}f"


def replace_const(text: str, ctype: str, name: str, value: str) -> str:
    pattern = rf"static (?:const )?{re.escape(ctype)} {re.escape(name)} = .*?;"
    replacement = f"static {ctype} {name} = {value};"
    new_text, count = re.subn(pattern, replacement, text)
    if count != 1:
        raise RuntimeError(f"Constante não encontrada ou ambígua: {ctype} {name}")
    return new_text


def load_config(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"JSON inválido: {path}")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Aplica config/proteções no firmware embarcado embedded_receiver.ino."
    )
    parser.add_argument("--config", required=True, help="config.json gerado pela unidade geradora")
    parser.add_argument("--normalize-to-comtrade", action="store_true")
    parser.add_argument("--distance", nargs=4, type=float, metavar=("LINE_Z_OHM", "Z1_PCT", "Z2_PCT", "DELAY_Z2_S"))
    parser.add_argument("--distance-line-angle", type=float, default=None)
    parser.add_argument("--directional-67", choices=("forward", "reverse"))
    parser.add_argument("--directional-67-power-min", type=float, default=None)
    parser.add_argument("--over-current", nargs=3, type=float, metavar=("INST_50_RISE_PCT", "TIMED_51_RISE_PCT", "DELAY_51_S"))
    parser.add_argument("--under-voltage", nargs=3, type=float, metavar=("INST_LEVEL_PCT", "TIMED_DROP_PCT", "DELAY_S"))
    parser.add_argument("--over-voltage", nargs=3, type=float, metavar=("INST_RISE_PCT", "TIMED_RISE_PCT", "DELAY_S"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.over_current is not None:
        oc50, oc51, delay = args.over_current
        if oc50 <= oc51 or oc51 < 0.0 or delay < 0.0:
            raise SystemExit("--over-current exige INST_50_RISE_PCT > TIMED_51_RISE_PCT >= 0 e DELAY_51_S >= 0")
    if args.under_voltage is not None:
        instant_level, timed_drop, delay = args.under_voltage
        if not (0.0 < instant_level < 100.0 - timed_drop < 100.0) or delay < 0.0:
            raise SystemExit("--under-voltage exige 0 < INST_LEVEL_PCT < 100-TIMED_DROP_PCT < 100 e DELAY_S >= 0")
    if args.over_voltage is not None:
        instant_rise, timed_rise, delay = args.over_voltage
        if instant_rise <= timed_rise or timed_rise < 0.0 or delay < 0.0:
            raise SystemExit("--over-voltage exige INST_RISE_PCT > TIMED_RISE_PCT >= 0 e DELAY_S >= 0")

    cfg_path = Path(args.config)
    payload = load_config(cfg_path)
    rec = payload.get("receiver_recommendation", {})
    cref = payload.get("comtrade_reference", {})
    if not isinstance(rec, dict) or not isinstance(cref, dict):
        raise SystemExit("Config precisa conter receiver_recommendation e comtrade_reference.")

    required = [
        ("receiver_recommendation", rec, "v_scale_eng_per_volt"),
        ("receiver_recommendation", rec, "i_scale_eng_per_volt"),
        ("receiver_recommendation", rec, "freq_hz"),
        ("receiver_recommendation", rec, "sample_rate_hz"),
        ("comtrade_reference", cref, "v_nom_rms"),
        ("comtrade_reference", cref, "i_nom_rms"),
    ]
    for section, obj, key in required:
        if key not in obj:
            raise SystemExit(f"Campo ausente em {section}: {key}")

    text = SKETCH_PATH.read_text(encoding="utf-8")

    text = replace_const(text, "float", "V_SCALE_ENG_PER_VOLT", float_cpp(rec["v_scale_eng_per_volt"]))
    text = replace_const(text, "float", "I_SCALE_ENG_PER_VOLT", float_cpp(rec["i_scale_eng_per_volt"]))
    text = replace_const(text, "float", "V_NOMINAL_RMS", float_cpp(cref["v_nom_rms"]))
    text = replace_const(text, "float", "I_NOMINAL_RMS", float_cpp(cref["i_nom_rms"]))
    text = replace_const(text, "float", "FREQ_REF_HZ", float_cpp(rec["freq_hz"]))
    text = replace_const(text, "uint32_t", "SAMPLE_RATE_HZ", str(int(round(float(rec["sample_rate_hz"])))))
    text = replace_const(text, "bool", "NORMALIZE_TO_COMTRADE", bool_cpp(args.normalize_to_comtrade))

    text = replace_const(text, "bool", "OC_ENABLED", bool_cpp(args.over_current is not None))
    if args.over_current is not None:
        oc50, oc51, delay = args.over_current
        text = replace_const(text, "float", "OC_51_PCT", float_cpp(oc51))
        text = replace_const(text, "float", "OC_50_PCT", float_cpp(oc50))
        text = replace_const(text, "float", "OC_51_DELAY_S", float_cpp(delay))

    text = replace_const(text, "bool", "DIST_ENABLED", bool_cpp(args.distance is not None))
    if args.distance is not None:
        line_z, z1_pct, z2_pct, delay = args.distance
        text = replace_const(text, "float", "DIST_LINE_Z_OHM", float_cpp(line_z))
        text = replace_const(text, "float", "DIST_Z1_PCT", float_cpp(z1_pct))
        text = replace_const(text, "float", "DIST_Z2_PCT", float_cpp(z2_pct))
        text = replace_const(text, "float", "DIST_Z2_DELAY_S", float_cpp(delay))
    if args.distance_line_angle is not None:
        text = replace_const(text, "float", "DIST_LINE_ANGLE_DEG", float_cpp(args.distance_line_angle))

    text = replace_const(text, "bool", "DIR67_ENABLED", bool_cpp(args.directional_67 is not None))
    if args.directional_67 is not None:
        text = replace_const(text, "bool", "DIR67_FORWARD", bool_cpp(args.directional_67 == "forward"))
    if args.directional_67_power_min is not None:
        text = replace_const(text, "float", "DIR67_POWER_MIN_W", float_cpp(args.directional_67_power_min))

    text = replace_const(text, "bool", "UV_ENABLED", bool_cpp(args.under_voltage is not None))
    if args.under_voltage is not None:
        instant_level, timed_drop, delay = args.under_voltage
        text = replace_const(text, "float", "UV_INST_LEVEL_PCT", float_cpp(instant_level))
        text = replace_const(text, "float", "UV_TIMED_DROP_PCT", float_cpp(timed_drop))
        text = replace_const(text, "float", "UV_DELAY_S", float_cpp(delay))

    text = replace_const(text, "bool", "OV_ENABLED", bool_cpp(args.over_voltage is not None))
    if args.over_voltage is not None:
        instant_rise, timed_rise, delay = args.over_voltage
        text = replace_const(text, "float", "OV_INST_RISE_PCT", float_cpp(instant_rise))
        text = replace_const(text, "float", "OV_TIMED_RISE_PCT", float_cpp(timed_rise))
        text = replace_const(text, "float", "OV_DELAY_S", float_cpp(delay))

    if args.dry_run:
        print(text)
    else:
        SKETCH_PATH.write_text(text, encoding="utf-8")
        print(f"Firmware configurado: {SKETCH_PATH}")
        print(f"Config usada: {cfg_path}")


if __name__ == "__main__":
    main()

import argparse
import csv
import json
import math
import os
import statistics
import struct
import time
from pathlib import Path

import serial

H0 = 0xA5
H1 = 0x5A
FRAME_SIZE = 13
ADC_MAX = 4095.0

FLAG_LOST_FRAME = 1 << 0
FLAG_BAD_CHECKSUM = 1 << 1
FLAG_DESYNC = 1 << 2
FLAG_ADC_SAT = 1 << 3
FLAG_OUTLIER = 1 << 4
FLAG_INCOMPLETE_CYCLE = 1 << 5
FLAG_FREQ_OOR = 1 << 6


def checksum8(buf: bytes) -> int:
    return sum(buf) & 0xFF


def parse_frame(frame: bytes):
    if len(frame) != FRAME_SIZE:
        return None
    if frame[0] != H0 or frame[1] != H1:
        return None
    if checksum8(frame[:12]) != frame[12]:
        return None

    seq = struct.unpack_from("<H", frame, 2)[0]
    t_us = struct.unpack_from("<I", frame, 4)[0]
    adc_34 = struct.unpack_from("<H", frame, 8)[0]
    adc_35 = struct.unpack_from("<H", frame, 10)[0]
    if adc_34 > 4095 or adc_35 > 4095:
        return None
    return seq, t_us, adc_34, adc_35


def adc_to_volts(adc: int, adc_vref: float) -> float:
    return (float(adc) / ADC_MAX) * adc_vref


def clamp_dt_us(dt_us: int, nominal_dt_us: float) -> int:
    if dt_us <= 0:
        return int(round(nominal_dt_us))
    if dt_us > int(10.0 * nominal_dt_us):
        return int(round(nominal_dt_us))
    return dt_us


def auto_outlier_step(amp_peak: float, freq_hz: float, sample_rate_hz: float) -> float:
    # Estima degrau máximo de uma senoide: A * 2*pi*f / fs.
    # Multiplica por margem para não cortar sinal válido.
    fs = max(1.0, sample_rate_hz)
    f = max(1.0, freq_hz)
    a = max(1.0, abs(amp_peak))
    max_step = a * (2.0 * math.pi * f) / fs
    return max(0.8, 6.0 * max_step)


def wrap_angle_deg(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0:
        return 180.0
    return wrapped


def fundamental_phasor_rms(samples: list[float]) -> tuple[float, float]:
    n = len(samples)
    if n < 2:
        return 0.0, 0.0

    re = 0.0
    im = 0.0
    for k, sample in enumerate(samples):
        theta = (2.0 * math.pi * k) / n
        re += sample * math.cos(theta)
        im -= sample * math.sin(theta)

    peak_mag = (2.0 / n) * math.hypot(re, im)
    rms_mag = peak_mag / math.sqrt(2.0)
    angle_deg = wrap_angle_deg(math.degrees(math.atan2(im, re)))
    return rms_mag, angle_deg


def load_config_file(config_path: Path) -> dict:
    if not config_path.exists():
        raise SystemExit(f"Arquivo de configuração não encontrado: {config_path}")
    try:
        payload = json.loads(config_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise SystemExit(f"Falha ao ler JSON de configuração: {config_path} ({exc})") from exc
    if not isinstance(payload, dict):
        raise SystemExit(f"JSON inválido em {config_path}: esperado objeto no topo.")
    return payload


def resolve_auto_config_path() -> Path | None:
    script_dir = Path(__file__).resolve().parent
    candidates = [
        Path.cwd() / "receiver-config.json",
        Path.cwd() / "receiver_config.json",
        script_dir / "receiver-config.json",
        script_dir / "receiver_config.json",
        script_dir.parents[2] / "generator/pyhton/v_6/receiver-config.json",
        script_dir.parents[2] / "generator/pyhton/v_6/receiver_config.json",
    ]
    for p in candidates:
        if p.exists():
            return p
    return None


def print_startup_summary(
    *,
    args,
    config_path: Path | None,
    cfg_freq_hz: float | None,
    cfg_v_clip_peak: float | None,
    cfg_i_clip_peak: float | None,
    cfg_v_nom_rms: float | None,
    cfg_i_nom_rms: float | None,
) -> None:
    nominal_dt_us = 1_000_000.0 / max(1.0, args.sample_rate)
    print("=== Pre-carregamento ===")
    if config_path is not None:
        print(f"Config: {config_path}")
    else:
        print("Config: nao encontrado (usando CLI/default)")
    print(f"Serial: port={args.port} baud={args.baud}")
    print(f"Amostragem: fs={args.sample_rate:.3f} Hz (dt_nom={nominal_dt_us:.3f} us)")
    print(
        f"Escalas: adc_vref={args.adc_vref:.6f} V "
        f"v_scale={args.v_scale:.9f} i_scale={args.i_scale:.9f}"
    )
    if cfg_freq_hz is not None:
        print(f"Referencia freq_hz (config): {cfg_freq_hz:.6f} Hz")
    print(f"Janela de frequencia valida: {args.f_min:.3f} .. {args.f_max:.3f} Hz")
    if cfg_v_nom_rms is not None or cfg_i_nom_rms is not None:
        print(
            "Referencia COMTRADE: "
            f"v_nom_rms={cfg_v_nom_rms if cfg_v_nom_rms is not None else 'n/a'} "
            f"i_nom_rms={cfg_i_nom_rms if cfg_i_nom_rms is not None else 'n/a'}"
        )
    if cfg_v_clip_peak is not None or cfg_i_clip_peak is not None:
        print(
            "Clipping esperado (config): "
            f"v_clip_peak={cfg_v_clip_peak if cfg_v_clip_peak is not None else 'n/a'} "
            f"i_clip_peak={cfg_i_clip_peak if cfg_i_clip_peak is not None else 'n/a'}"
        )
    print(
        "Outlier thresholds: "
        f"V={args.outlier_step_v:.6f} I={args.outlier_step_i:.6f} "
        "(auto se nao informado via CLI)"
    )
    if args.normalize_to_comtrade:
        print(
            "Normalizacao COMTRADE: ativa "
            f"(prefault_cycles={max(1, args.prefault_cycles)}, norm_min_pu={args.norm_min_pu:.3f})"
        )
    else:
        print("Normalizacao COMTRADE: desativada")
    print("========================")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Leitura 2 canais com RMS robusto em tempo real: auto-offset, "
            "detecção de ciclo por zero-cross e flags de qualidade."
        )
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Caminho do JSON gerado pelo gerador (receiver-config.json).",
    )
    parser.add_argument("--port", default="/dev/ttyUSB1", help="Porta serial da ESP32")
    parser.add_argument("--baud", type=int, default=921600, help="Baudrate serial")
    parser.add_argument("--out", default="adc_capture.csv", help="CSV de saída")
    parser.add_argument("--adc-vref", type=float, default=3.3, help="Referência ADC (V)")
    parser.add_argument("--sample-rate", type=float, default=1000.0, help="Taxa nominal (Hz)")
    parser.add_argument("--offset-tau", type=float, default=1.0, help="Constante de tempo do auto-offset (s)")
    parser.add_argument("--v-scale", type=float, default=1.0, help="Escala para tensão real (V/V)")
    parser.add_argument("--i-scale", type=float, default=1.0, help="Escala para corrente real (A/V)")
    parser.add_argument("--zc-hyst", type=float, default=0.03, help="Histerese de zero-cross em tensão real")
    parser.add_argument(
        "--outlier-step-v",
        type=float,
        default=None,
        help="Degrau máximo por amostra (tensão real). Se omitido, cálculo automático.",
    )
    parser.add_argument(
        "--outlier-step-i",
        type=float,
        default=None,
        help="Degrau máximo por amostra (corrente real). Se omitido, cálculo automático.",
    )
    parser.add_argument("--f-min", type=float, default=55.0, help="Frequência mínima válida (Hz)")
    parser.add_argument("--f-max", type=float, default=65.0, help="Frequência máxima válida (Hz)")
    parser.add_argument(
        "--normalize-to-comtrade",
        action="store_true",
        help=(
            "Normaliza RMS para as referências comtrade_reference.v_nom_rms/i_nom_rms "
            "do receiver-config.json."
        ),
    )
    parser.add_argument(
        "--prefault-cycles",
        type=int,
        default=10,
        help="Quantidade de ciclos válidos para estimar ganho de normalização.",
    )
    parser.add_argument(
        "--norm-min-pu",
        type=float,
        default=0.5,
        help=(
            "Para calibrar normalização, aceita só ciclos com RMS bruto >= este pu do nominal "
            "(evita calibrar em idle/transição)."
        ),
    )
    parser.add_argument(
        "--i-nominal-rms",
        type=float,
        default=None,
        help=(
            "Corrente nominal RMS para proteções. Se omitida, usa "
            "comtrade_reference.i_nom_rms do receiver-config.json."
        ),
    )
    parser.add_argument(
        "--v-nominal-rms",
        type=float,
        default=None,
        help=(
            "Tensão nominal RMS para proteções. Se omitida, usa "
            "comtrade_reference.v_nom_rms do receiver-config.json."
        ),
    )
    parser.add_argument(
        "--over-current",
        nargs=3,
        type=float,
        metavar=("PICKUP_51_PCT", "PICKUP_50_PCT", "DELAY_51_S"),
        default=None,
        help=(
            "Ativa proteção de sobrecorrente 51/50 usando I1 Fourier. "
            "Ex.: --over-current 10 20 0.5."
        ),
    )
    parser.add_argument(
        "--distance",
        nargs=4,
        type=float,
        metavar=("LINE_Z_OHM", "Z1_PCT", "Z2_PCT", "DELAY_Z2_S"),
        default=None,
        help=(
            "Ativa proteção de distância 21 por impedância aparente |V1/I1|. "
            "Ex.: --distance 52.12496 80 120 0.05."
        ),
    )
    parser.add_argument(
        "--under-voltage",
        nargs=2,
        type=float,
        metavar=("PICKUP_27_PCT", "DELAY_27_S"),
        default=None,
        help=(
            "Ativa proteção de subtensão 27 usando V1 Fourier. "
            "Ex.: --under-voltage 90 0.2."
        ),
    )
    parser.add_argument(
        "--over-voltage",
        nargs=2,
        type=float,
        metavar=("PICKUP_59_PCT", "DELAY_59_S"),
        default=None,
        help=(
            "Ativa proteção de sobretensão 59 usando V1 Fourier. "
            "Ex.: --over-voltage 110 0.2."
        ),
    )
    parser.add_argument(
        "--distance-min-current",
        type=float,
        default=None,
        help=(
            "Corrente mínima I1 RMS para avaliar a proteção de distância. "
            "Se omitida, usa 5%% da corrente nominal quando disponível."
        ),
    )
    parser.add_argument(
        "--protection-events",
        action="store_true",
        help="Imprime eventos de proteção no terminal: pickup, reset e trip.",
    )
    parser.add_argument(
        "--protection-event-interval",
        type=float,
        default=0.1,
        help="Intervalo mínimo entre logs repetitivos de temporização (s).",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=0,
        help="Imprimir status resumido a cada N amostras (0 desativa).",
    )
    args = parser.parse_args()

    # Carrega config do gerador automaticamente (ou via --config) e aplica escala/base.
    config_path: Path | None
    if args.config:
        config_path = Path(args.config)
    else:
        config_path = resolve_auto_config_path()

    cfg_freq_hz = None
    cfg_v_clip_peak = None
    cfg_i_clip_peak = None
    cfg_v_nom_rms = None
    cfg_i_nom_rms = None

    if config_path is not None:
        payload = load_config_file(config_path)
        rec = payload.get("receiver_recommendation", {})
        if isinstance(rec, dict):
            args.adc_vref = float(rec.get("adc_vref", args.adc_vref))
            args.v_scale = float(rec.get("v_scale_eng_per_volt", args.v_scale))
            args.i_scale = float(rec.get("i_scale_eng_per_volt", args.i_scale))
            args.sample_rate = float(rec.get("sample_rate_hz", args.sample_rate))
            f_cfg = rec.get("freq_hz")
            if f_cfg is not None and float(f_cfg) > 0.0:
                f_cfg = float(f_cfg)
                cfg_freq_hz = f_cfg
                # Janela de validação de frequência centrada no valor de referência.
                args.f_min = min(args.f_min, 0.9 * f_cfg)
                args.f_max = max(args.f_max, 1.1 * f_cfg)
        cref = payload.get("comtrade_reference", {})
        if isinstance(cref, dict):
            vcp = cref.get("v_clip_peak")
            icp = cref.get("i_clip_peak")
            vnr = cref.get("v_nom_rms")
            inr = cref.get("i_nom_rms")
            if vcp is not None:
                cfg_v_clip_peak = float(vcp)
            if icp is not None:
                cfg_i_clip_peak = float(icp)
            if vnr is not None:
                cfg_v_nom_rms = float(vnr)
            if inr is not None:
                cfg_i_nom_rms = float(inr)
        print(f"ℹ️  Configuração carregada de: {config_path}")
    else:
        print("ℹ️  receiver-config.json não encontrado automaticamente; usando parâmetros CLI/default.")

    # Limites de outlier automáticos (se não vieram por CLI)
    auto_freq = cfg_freq_hz if cfg_freq_hz is not None else 0.5 * (args.f_min + args.f_max)
    if args.outlier_step_v is None:
        amp_v = cfg_v_clip_peak if cfg_v_clip_peak is not None else abs(args.v_scale) * 1.55
        args.outlier_step_v = auto_outlier_step(amp_v, auto_freq, args.sample_rate)
    if args.outlier_step_i is None:
        amp_i = cfg_i_clip_peak if cfg_i_clip_peak is not None else abs(args.i_scale) * 1.55
        args.outlier_step_i = auto_outlier_step(amp_i, auto_freq, args.sample_rate)
    if args.normalize_to_comtrade and (cfg_v_nom_rms is None or cfg_i_nom_rms is None):
        print("⚠️  --normalize-to-comtrade ativo, mas v_nom_rms/i_nom_rms não encontrados no config.")

    print_startup_summary(
        args=args,
        config_path=config_path,
        cfg_freq_hz=cfg_freq_hz,
        cfg_v_clip_peak=cfg_v_clip_peak,
        cfg_i_clip_peak=cfg_i_clip_peak,
        cfg_v_nom_rms=cfg_v_nom_rms,
        cfg_i_nom_rms=cfg_i_nom_rms,
    )

    oc_enabled = args.over_current is not None
    oc_i_nominal = float(args.i_nominal_rms) if args.i_nominal_rms is not None else cfg_i_nom_rms
    oc_51_pct = 0.0
    oc_50_pct = 0.0
    oc_51_delay_s = 0.0
    oc_51_pickup = 0.0
    oc_50_pickup = 0.0
    oc_51_dropout = 0.0

    if oc_enabled:
        oc_51_pct, oc_50_pct, oc_51_delay_s = [float(value) for value in args.over_current]
        if oc_i_nominal is None or oc_i_nominal <= 0.0:
            raise SystemExit(
                "Proteção --over-current precisa de corrente nominal: "
                "use --i-nominal-rms ou informe comtrade_reference.i_nom_rms no config."
            )
        if oc_51_pct < 0.0 or oc_50_pct < 0.0:
            raise SystemExit("Proteção --over-current exige percentuais positivos.")
        if oc_50_pct <= oc_51_pct:
            raise SystemExit("Proteção --over-current exige PICKUP_50_PCT maior que PICKUP_51_PCT.")
        if oc_51_delay_s < 0.0:
            raise SystemExit("Proteção --over-current exige DELAY_51_S >= 0.")

        oc_51_pickup = oc_i_nominal * (1.0 + oc_51_pct / 100.0)
        oc_50_pickup = oc_i_nominal * (1.0 + oc_50_pct / 100.0)
        oc_51_dropout = 0.95 * oc_51_pickup
        print(
            "Proteção 50/51 ativa: "
            f"I_nom={oc_i_nominal:.6f}A "
            f"pickup_51={oc_51_pickup:.6f}A ({oc_51_pct:.3f}%) "
            f"delay_51={oc_51_delay_s:.6f}s "
            f"pickup_50={oc_50_pickup:.6f}A ({oc_50_pct:.3f}%)"
        )
    else:
        print("Proteção 50/51: desativada")

    dist_enabled = args.distance is not None
    dist_line_z_ohm = 0.0
    dist_z1_pct = 0.0
    dist_z2_pct = 0.0
    dist_z1_ohm = 0.0
    dist_z2_ohm = 0.0
    dist_z2_delay_s = 0.0
    dist_z2_dropout_ohm = 0.0
    dist_min_current = 0.0

    if dist_enabled:
        dist_line_z_ohm, dist_z1_pct, dist_z2_pct, dist_z2_delay_s = [
            float(value) for value in args.distance
        ]
        if dist_line_z_ohm <= 0.0:
            raise SystemExit("Proteção --distance exige LINE_Z_OHM positivo.")
        if dist_z1_pct <= 0.0 or dist_z2_pct <= 0.0:
            raise SystemExit("Proteção --distance exige Z1_PCT e Z2_PCT positivos.")
        if dist_z2_pct <= dist_z1_pct:
            raise SystemExit("Proteção --distance exige Z2_PCT maior que Z1_PCT.")
        if dist_z2_delay_s < 0.0:
            raise SystemExit("Proteção --distance exige DELAY_Z2_S >= 0.")

        dist_z1_ohm = dist_line_z_ohm * (dist_z1_pct / 100.0)
        dist_z2_ohm = dist_line_z_ohm * (dist_z2_pct / 100.0)

        if args.distance_min_current is not None:
            dist_min_current = float(args.distance_min_current)
            if dist_min_current < 0.0:
                raise SystemExit("--distance-min-current precisa ser >= 0.")
        elif oc_i_nominal is not None and oc_i_nominal > 0.0:
            dist_min_current = 0.05 * oc_i_nominal
        else:
            dist_min_current = 1e-6
            print(
                "⚠️  --distance sem corrente nominal: usando distance_min_current=1e-6. "
                "Recomenda-se informar --i-nominal-rms ou --distance-min-current."
            )

        dist_z2_dropout_ohm = 1.05 * dist_z2_ohm
        print(
            "Proteção 21 ativa: "
            f"Zlinha={dist_line_z_ohm:.6f}ohm "
            f"Z1={dist_z1_ohm:.6f}ohm ({dist_z1_pct:.3f}%) instantânea "
            f"Z2={dist_z2_ohm:.6f}ohm ({dist_z2_pct:.3f}%) delay={dist_z2_delay_s:.6f}s "
            f"Imin={dist_min_current:.6f}A"
        )
    else:
        print("Proteção 21: desativada")

    vprot_nominal = float(args.v_nominal_rms) if args.v_nominal_rms is not None else cfg_v_nom_rms
    uv_enabled = args.under_voltage is not None
    ov_enabled = args.over_voltage is not None
    uv_pickup_pct = 0.0
    ov_pickup_pct = 0.0
    uv_delay_s = 0.0
    ov_delay_s = 0.0
    uv_pickup = 0.0
    ov_pickup = 0.0
    uv_dropout = 0.0
    ov_dropout = 0.0

    if uv_enabled or ov_enabled:
        if vprot_nominal is None or vprot_nominal <= 0.0:
            raise SystemExit(
                "Proteções --under-voltage/--over-voltage precisam de tensão nominal: "
                "use --v-nominal-rms ou informe comtrade_reference.v_nom_rms no config."
            )

    if uv_enabled:
        uv_pickup_pct, uv_delay_s = [float(value) for value in args.under_voltage]
        if not (0.0 < uv_pickup_pct < 100.0):
            raise SystemExit("--under-voltage exige PICKUP_27_PCT entre 0 e 100.")
        if uv_delay_s < 0.0:
            raise SystemExit("--under-voltage exige DELAY_27_S >= 0.")
        uv_pickup = vprot_nominal * (uv_pickup_pct / 100.0)
        uv_dropout = 1.03 * uv_pickup
        print(
            "Proteção 27 ativa: "
            f"V_nom={vprot_nominal:.6f} "
            f"pickup={uv_pickup:.6f} ({uv_pickup_pct:.3f}%) "
            f"delay={uv_delay_s:.6f}s dropout={uv_dropout:.6f}"
        )
    else:
        print("Proteção 27: desativada")

    if ov_enabled:
        ov_pickup_pct, ov_delay_s = [float(value) for value in args.over_voltage]
        if ov_pickup_pct <= 100.0:
            raise SystemExit("--over-voltage exige PICKUP_59_PCT maior que 100.")
        if ov_delay_s < 0.0:
            raise SystemExit("--over-voltage exige DELAY_59_S >= 0.")
        ov_pickup = vprot_nominal * (ov_pickup_pct / 100.0)
        ov_dropout = 0.97 * ov_pickup
        print(
            "Proteção 59 ativa: "
            f"V_nom={vprot_nominal:.6f} "
            f"pickup={ov_pickup:.6f} ({ov_pickup_pct:.3f}%) "
            f"delay={ov_delay_s:.6f}s dropout={ov_dropout:.6f}"
        )
    else:
        print("Proteção 59: desativada")

    nominal_dt_us = 1_000_000.0 / max(1.0, args.sample_rate)
    out_path = Path(args.out)

    ser = serial.Serial(args.port, args.baud, timeout=0.25)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(f"ℹ️  RX serial aberta em {args.port} @ {args.baud}. Aguardando frames válidos...")

    buf = bytearray()
    t0 = time.perf_counter()

    def protection_log(message: str) -> None:
        if args.protection_events:
            print(message, flush=True)

    # Estado global de comunicação
    last_seq = None
    last_dev_t_us = None
    lost_total = 0
    bad_checksum_total = 0
    desync_total = 0
    sample_count = 0
    rx_link_reported = False

    # Auto-offset (inicia em midscale)
    v_offset_est = 1.65
    i_offset_est = 1.65

    # Sinais instantâneos (após offset + escala)
    v_inst_prev = 0.0
    i_inst_prev = 0.0
    have_prev_inst = False

    # Estado de zero-cross com histerese
    state = 0  # -1 abaixo, +1 acima, 0 indefinido

    # Acumuladores do ciclo atual
    cycle_id = 0
    cycle_start_us = None
    cyc_n = 0
    cyc_sum_v2 = 0.0
    cyc_sum_i2 = 0.0
    cyc_v_samples: list[float] = []
    cyc_i_samples: list[float] = []
    cyc_flags = 0

    # Último RMS válido publicado
    v_rms = 0.0
    i_rms = 0.0
    v_rms_raw = 0.0
    i_rms_raw = 0.0
    f_est = 0.0
    rms_valid = 0
    v1_mag_raw = 0.0
    i1_mag_raw = 0.0
    v1_mag = 0.0
    i1_mag = 0.0
    v1_angle_deg = 0.0
    i1_angle_deg = 0.0
    vi_angle_deg = 0.0
    fourier_valid = 0
    last_cycle_quality_flags = 0
    norm_ready = 0
    norm_gain_v = 1.0
    norm_gain_i = 1.0
    prefault_v_raw: list[float] = []
    prefault_i_raw: list[float] = []

    # Estado da proteção de sobrecorrente 50/51.
    oc_51_timer_s = 0.0
    oc_51_active = 0
    oc_50_trip = 0
    oc_51_trip = 0
    oc_trip = 0
    oc_trip_code = "NONE"
    oc_51_timing_report_s = 0.0

    # Estado da proteção de distância 21.
    dist_z_mag_ohm = 0.0
    dist_z_angle_deg = 0.0
    dist_r_ohm = 0.0
    dist_x_ohm = 0.0
    dist_fault_pct = 0.0
    dist_fault_ohm = 0.0
    dist_z1_active = 0
    dist_z2_active = 0
    dist_z2_timer_s = 0.0
    dist_z1_trip = 0
    dist_z2_trip = 0
    dist_trip = 0
    dist_trip_code = "NONE"
    dist_z2_timing_report_s = 0.0
    breaker_trip_reported = 0

    # Estado das proteções de tensão 27/59.
    uv_timer_s = 0.0
    ov_timer_s = 0.0
    uv_active = 0
    ov_active = 0
    uv_trip = 0
    ov_trip = 0
    voltage_trip = 0
    voltage_trip_code = "NONE"
    uv_timing_report_s = 0.0
    ov_timing_report_s = 0.0

    with out_path.open("w", newline="", encoding="utf-8", buffering=1) as f:
        w = csv.writer(f)
        w.writerow([
            "host_t_s",
            "dev_t_s",
            "dev_t_us",
            "seq",
            "adc_34",
            "adc_35",
            "v_adc_34",
            "v_adc_35",
            "v_offset_est",
            "i_offset_est",
            "v_inst",
            "i_inst",
            "v_rms_raw",
            "i_rms_raw",
            "v_rms",
            "i_rms",
            "v1_mag_raw",
            "i1_mag_raw",
            "v1_mag",
            "i1_mag",
            "v1_angle_deg",
            "i1_angle_deg",
            "vi_angle_deg",
            "fourier_valid",
            "f_est_hz",
            "rms_valid",
            "norm_ready",
            "norm_gain_v",
            "norm_gain_i",
            "oc_enabled",
            "oc_i_nominal",
            "oc_51_pickup",
            "oc_50_pickup",
            "oc_51_timer_s",
            "oc_51_active",
            "oc_50_trip",
            "oc_51_trip",
            "oc_trip",
            "oc_trip_code",
            "dist_enabled",
            "dist_line_z_ohm",
            "dist_z1_pct",
            "dist_z2_pct",
            "dist_z1_ohm",
            "dist_z2_ohm",
            "dist_min_current",
            "dist_z_mag_ohm",
            "dist_z_angle_deg",
            "dist_r_ohm",
            "dist_x_ohm",
            "dist_fault_pct",
            "dist_fault_ohm",
            "dist_z1_active",
            "dist_z2_active",
            "dist_z2_timer_s",
            "dist_z1_trip",
            "dist_z2_trip",
            "dist_trip",
            "dist_trip_code",
            "uv_enabled",
            "ov_enabled",
            "v_nominal_rms",
            "uv_pickup",
            "ov_pickup",
            "uv_timer_s",
            "ov_timer_s",
            "uv_active",
            "ov_active",
            "uv_trip",
            "ov_trip",
            "voltage_trip",
            "voltage_trip_code",
            "cycle_id",
            "cycle_update",
            "quality_flags",
            "lost_frames_total",
            "bad_checksum_total",
            "desync_total",
        ])

        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= FRAME_SIZE:
                if not (buf[0] == H0 and buf[1] == H1):
                    del buf[0]
                    desync_total += 1
                    cyc_flags |= FLAG_DESYNC
                    continue

                frame = bytes(buf[:FRAME_SIZE])
                parsed = parse_frame(frame)
                if parsed is None:
                    del buf[0]
                    bad_checksum_total += 1
                    cyc_flags |= FLAG_BAD_CHECKSUM
                    continue

                del buf[:FRAME_SIZE]
                seq, dev_t_us, adc_34, adc_35 = parsed
                sample_count += 1
                if not rx_link_reported:
                    print(
                        "✅ Comunicação RX estabelecida: "
                        f"primeiro frame válido seq={seq} dev_t_us={dev_t_us}"
                    )
                    rx_link_reported = True
                    f.flush()
                    os.fsync(f.fileno())

                # Perda de frame (seq de 16 bits)
                if last_seq is not None:
                    delta_seq = (seq - last_seq) & 0xFFFF
                    if delta_seq > 1:
                        lost_total += (delta_seq - 1)
                        cyc_flags |= FLAG_LOST_FRAME
                last_seq = seq

                # dt usando clock da ESP32 (não host PC)
                if last_dev_t_us is None:
                    dt_us = int(round(nominal_dt_us))
                else:
                    dt_us = clamp_dt_us(int((dev_t_us - last_dev_t_us) & 0xFFFFFFFF), nominal_dt_us)
                last_dev_t_us = dev_t_us
                dt_s = dt_us / 1_000_000.0

                # Conversão ADC -> Volts
                v_adc_34 = adc_to_volts(adc_34, args.adc_vref)
                v_adc_35 = adc_to_volts(adc_35, args.adc_vref)

                # Saturação próxima dos trilhos
                sat = (adc_34 <= 1 or adc_34 >= 4094 or adc_35 <= 1 or adc_35 >= 4094)
                if sat:
                    cyc_flags |= FLAG_ADC_SAT

                # Auto-offset lento (track de drift)
                alpha = dt_s / max(1e-6, args.offset_tau)
                if alpha > 1.0:
                    alpha = 1.0
                v_offset_est += alpha * (v_adc_34 - v_offset_est)
                i_offset_est += alpha * (v_adc_35 - i_offset_est)

                # Valores instantâneos reais (AC)
                v_inst = (v_adc_34 - v_offset_est) * args.v_scale
                i_inst = (v_adc_35 - i_offset_est) * args.i_scale
                # Sinal AC em volts na entrada ADC (independente de escala física)
                v_ac_adc = (v_adc_34 - v_offset_est)

                # Rejeição simples de outlier por degrau
                outlier = False
                if have_prev_inst:
                    if abs(v_inst - v_inst_prev) > args.outlier_step_v:
                        v_inst = v_inst_prev
                        outlier = True
                    if abs(i_inst - i_inst_prev) > args.outlier_step_i:
                        i_inst = i_inst_prev
                        outlier = True
                v_inst_prev = v_inst
                i_inst_prev = i_inst
                have_prev_inst = True
                if outlier:
                    cyc_flags |= FLAG_OUTLIER

                # Zero-cross usando sinal AC em volts na entrada ADC.
                # A histerese passada em unidade "real" é convertida para volts ADC
                # para reduzir dependência da escala v_scale.
                v_scale_abs = max(1e-9, abs(args.v_scale))
                zc_hyst_adc = max(0.002, args.zc_hyst / v_scale_abs)  # piso de 2 mV
                if v_ac_adc < -zc_hyst_adc:
                    new_state = -1
                elif v_ac_adc > zc_hyst_adc:
                    new_state = 1
                else:
                    new_state = state

                cycle_update = 0

                # Fecha ciclo em cruzamento ascendente: -1 -> +1
                if state == -1 and new_state == 1:
                    if cycle_start_us is not None:
                        period_us = int((dev_t_us - cycle_start_us) & 0xFFFFFFFF)
                        local_flags = cyc_flags
                        local_valid = 1

                        if cyc_n < 8:
                            local_flags |= FLAG_INCOMPLETE_CYCLE
                            local_valid = 0
                            local_f = 0.0
                            local_vrms = 0.0
                            local_irms = 0.0
                            local_v1_mag = 0.0
                            local_i1_mag = 0.0
                            local_v1_angle = 0.0
                            local_i1_angle = 0.0
                        else:
                            local_vrms = math.sqrt(max(0.0, cyc_sum_v2 / cyc_n))
                            local_irms = math.sqrt(max(0.0, cyc_sum_i2 / cyc_n))
                            local_v1_mag, local_v1_angle = fundamental_phasor_rms(cyc_v_samples)
                            local_i1_mag, local_i1_angle = fundamental_phasor_rms(cyc_i_samples)
                            local_f = 1_000_000.0 / max(1.0, float(period_us))
                            if not (args.f_min <= local_f <= args.f_max):
                                local_flags |= FLAG_FREQ_OOR
                                local_valid = 0

                        # Publica último resultado de ciclo
                        v_rms_raw = local_vrms
                        i_rms_raw = local_irms
                        f_est = local_f
                        rms_valid = local_valid
                        last_cycle_quality_flags = local_flags
                        v1_mag_raw = local_v1_mag
                        i1_mag_raw = local_i1_mag
                        v1_angle_deg = local_v1_angle
                        i1_angle_deg = local_i1_angle
                        vi_angle_deg = wrap_angle_deg(i1_angle_deg - v1_angle_deg)
                        fourier_valid = local_valid

                        # Normalização para a referência do COMTRADE.
                        if (
                            args.normalize_to_comtrade
                            and cfg_v_nom_rms is not None
                            and cfg_i_nom_rms is not None
                            and local_valid == 1
                        ):
                            min_v_raw = max(1e-9, args.norm_min_pu * cfg_v_nom_rms)
                            min_i_raw = max(1e-9, args.norm_min_pu * cfg_i_nom_rms)
                            good_for_norm = (v_rms_raw >= min_v_raw) and (i_rms_raw >= min_i_raw)
                            if norm_ready == 0:
                                if good_for_norm:
                                    prefault_v_raw.append(v_rms_raw)
                                    prefault_i_raw.append(i_rms_raw)
                                if len(prefault_v_raw) >= max(1, args.prefault_cycles):
                                    base_v = max(1e-9, statistics.median(prefault_v_raw))
                                    base_i = max(1e-9, statistics.median(prefault_i_raw))
                                    norm_gain_v = cfg_v_nom_rms / base_v
                                    norm_gain_i = cfg_i_nom_rms / base_i
                                    norm_ready = 1
                                    print(
                                        "ℹ️  Normalização calibrada: "
                                        f"gain_v={norm_gain_v:.6f} gain_i={norm_gain_i:.6f} "
                                        f"(base_v={base_v:.6f}, base_i={base_i:.6f})"
                                    )
                            if norm_ready == 1:
                                v_rms = v_rms_raw * norm_gain_v
                                i_rms = i_rms_raw * norm_gain_i
                                v1_mag = v1_mag_raw * norm_gain_v
                                i1_mag = i1_mag_raw * norm_gain_i
                            else:
                                v_rms = v_rms_raw
                                i_rms = i_rms_raw
                                v1_mag = v1_mag_raw
                                i1_mag = i1_mag_raw
                        else:
                            v_rms = v_rms_raw
                            i_rms = i_rms_raw
                            v1_mag = v1_mag_raw
                            i1_mag = i1_mag_raw

                        if oc_enabled:
                            oc_eval_ready = fourier_valid == 1 and (
                                not args.normalize_to_comtrade or norm_ready == 1
                            )
                            cycle_dt_s = period_us / 1_000_000.0

                            if oc_eval_ready:
                                prev_oc_51_active = oc_51_active
                                prev_oc_50_trip = oc_50_trip
                                prev_oc_51_trip = oc_51_trip

                                if i1_mag >= oc_50_pickup:
                                    oc_50_trip = 1
                                    oc_trip = 1
                                    if oc_trip_code == "NONE":
                                        oc_trip_code = "50"
                                    if prev_oc_50_trip == 0:
                                        protection_log(
                                            "[OC50 TRIP] "
                                            f"t={dev_t_s:.6f}s I1={i1_mag:.6f}A "
                                            f"pickup={oc_50_pickup:.6f}A delay=instantaneous"
                                        )

                                if i1_mag >= oc_51_pickup:
                                    oc_51_active = 1
                                    oc_51_timer_s += cycle_dt_s
                                    if prev_oc_51_active == 0:
                                        protection_log(
                                            "[OC51 PICKUP] "
                                            f"t={dev_t_s:.6f}s I1={i1_mag:.6f}A "
                                            f"pickup={oc_51_pickup:.6f}A "
                                            f"timer={oc_51_timer_s:.6f}s delay={oc_51_delay_s:.6f}s"
                                        )
                                        oc_51_timing_report_s = oc_51_timer_s
                                    elif (
                                        args.protection_events
                                        and args.protection_event_interval > 0.0
                                        and oc_51_timer_s - oc_51_timing_report_s
                                        >= args.protection_event_interval
                                        and oc_51_trip == 0
                                    ):
                                        protection_log(
                                            "[OC51 TIMING] "
                                            f"t={dev_t_s:.6f}s I1={i1_mag:.6f}A "
                                            f"timer={oc_51_timer_s:.6f}/{oc_51_delay_s:.6f}s"
                                        )
                                        oc_51_timing_report_s = oc_51_timer_s
                                    if oc_51_timer_s >= oc_51_delay_s:
                                        oc_51_trip = 1
                                        oc_trip = 1
                                        if oc_trip_code == "NONE":
                                            oc_trip_code = "51"
                                        if prev_oc_51_trip == 0:
                                            protection_log(
                                                "[OC51 TRIP] "
                                                f"t={dev_t_s:.6f}s I1={i1_mag:.6f}A "
                                                f"pickup={oc_51_pickup:.6f}A "
                                                f"elapsed={oc_51_timer_s:.6f}s"
                                            )
                                elif i1_mag < oc_51_dropout:
                                    if oc_51_active == 1 and oc_51_trip == 0:
                                        protection_log(
                                            "[OC51 RESET] "
                                            f"t={dev_t_s:.6f}s I1={i1_mag:.6f}A "
                                            f"dropout={oc_51_dropout:.6f}A "
                                            f"timer_reset={oc_51_timer_s:.6f}s"
                                        )
                                    oc_51_active = 0
                                    oc_51_timer_s = 0.0
                                    oc_51_timing_report_s = 0.0
                                else:
                                    oc_51_active = 0
                            else:
                                oc_51_active = 0
                                oc_51_timer_s = 0.0
                                oc_51_timing_report_s = 0.0

                        if dist_enabled:
                            dist_eval_ready = fourier_valid == 1 and (
                                not args.normalize_to_comtrade or norm_ready == 1
                            )
                            cycle_dt_s = period_us / 1_000_000.0

                            if dist_eval_ready and i1_mag > max(1e-12, dist_min_current):
                                prev_dist_z2_active = dist_z2_active
                                prev_dist_z1_trip = dist_z1_trip
                                prev_dist_z2_trip = dist_z2_trip

                                dist_z_mag_ohm = v1_mag / max(1e-12, i1_mag)
                                dist_z_angle_deg = wrap_angle_deg(v1_angle_deg - i1_angle_deg)
                                dist_angle_rad = math.radians(dist_z_angle_deg)
                                dist_r_ohm = dist_z_mag_ohm * math.cos(dist_angle_rad)
                                dist_x_ohm = dist_z_mag_ohm * math.sin(dist_angle_rad)
                                dist_fault_ohm = dist_z_mag_ohm
                                dist_fault_pct = (dist_fault_ohm / dist_line_z_ohm) * 100.0

                                if dist_z_mag_ohm <= dist_z1_ohm:
                                    dist_z1_active = 1
                                    dist_z2_active = 0
                                    dist_z1_trip = 1
                                    dist_trip = 1
                                    if dist_trip_code == "NONE":
                                        dist_trip_code = "21Z1"
                                    if prev_dist_z1_trip == 0:
                                        protection_log(
                                            "[D21Z1 TRIP] "
                                            f"t={dev_t_s:.6f}s |Z|={dist_z_mag_ohm:.6f}ohm "
                                            f"R={dist_r_ohm:.6f}ohm X={dist_x_ohm:.6f}ohm "
                                            f"fault={dist_fault_ohm:.6f}ohm ({dist_fault_pct:.3f}% line) "
                                            f"I1={i1_mag:.6f}A V1={v1_mag:.6f} "
                                            f"zone=Z1 reach={dist_z1_ohm:.6f}ohm "
                                            "delay=instantaneous"
                                        )
                                elif dist_z_mag_ohm <= dist_z2_ohm:
                                    dist_z1_active = 0
                                    dist_z2_active = 1
                                    dist_z2_timer_s += cycle_dt_s
                                    if prev_dist_z2_active == 0 and dist_z2_trip == 0:
                                        protection_log(
                                            "[D21Z2 PICKUP] "
                                            f"t={dev_t_s:.6f}s |Z|={dist_z_mag_ohm:.6f}ohm "
                                            f"R={dist_r_ohm:.6f}ohm X={dist_x_ohm:.6f}ohm "
                                            f"fault={dist_fault_ohm:.6f}ohm ({dist_fault_pct:.3f}% line) "
                                            f"I1={i1_mag:.6f}A V1={v1_mag:.6f} "
                                            f"timer={dist_z2_timer_s:.6f}s "
                                            f"delay={dist_z2_delay_s:.6f}s"
                                        )
                                        dist_z2_timing_report_s = dist_z2_timer_s
                                    elif (
                                        args.protection_events
                                        and args.protection_event_interval > 0.0
                                        and dist_z2_timer_s - dist_z2_timing_report_s
                                        >= args.protection_event_interval
                                        and dist_z2_trip == 0
                                    ):
                                        protection_log(
                                            "[D21Z2 TIMING] "
                                            f"t={dev_t_s:.6f}s |Z|={dist_z_mag_ohm:.6f}ohm "
                                            f"R={dist_r_ohm:.6f}ohm X={dist_x_ohm:.6f}ohm "
                                            f"fault={dist_fault_ohm:.6f}ohm ({dist_fault_pct:.3f}% line) "
                                            f"timer={dist_z2_timer_s:.6f}/{dist_z2_delay_s:.6f}s"
                                        )
                                        dist_z2_timing_report_s = dist_z2_timer_s
                                    if dist_z2_timer_s >= dist_z2_delay_s:
                                        dist_z2_trip = 1
                                        dist_trip = 1
                                        if dist_trip_code == "NONE":
                                            dist_trip_code = "21Z2"
                                        if prev_dist_z2_trip == 0:
                                            protection_log(
                                                "[D21Z2 TRIP] "
                                                f"t={dev_t_s:.6f}s |Z|={dist_z_mag_ohm:.6f}ohm "
                                                f"R={dist_r_ohm:.6f}ohm X={dist_x_ohm:.6f}ohm "
                                                f"fault={dist_fault_ohm:.6f}ohm ({dist_fault_pct:.3f}% line) "
                                                f"I1={i1_mag:.6f}A V1={v1_mag:.6f} "
                                                f"elapsed={dist_z2_timer_s:.6f}s"
                                            )
                                elif dist_z_mag_ohm > dist_z2_dropout_ohm:
                                    if dist_z2_active == 1 and dist_z2_trip == 0:
                                        protection_log(
                                            "[D21Z2 RESET] "
                                            f"t={dev_t_s:.6f}s |Z|={dist_z_mag_ohm:.6f}ohm "
                                            f"fault={dist_fault_ohm:.6f}ohm ({dist_fault_pct:.3f}% line) "
                                            f"dropout={dist_z2_dropout_ohm:.6f}ohm "
                                            f"timer_reset={dist_z2_timer_s:.6f}s"
                                        )
                                    dist_z1_active = 0
                                    dist_z2_active = 0
                                    dist_z2_timer_s = 0.0
                                    dist_z2_timing_report_s = 0.0
                                else:
                                    dist_z1_active = 0
                                    dist_z2_active = 0
                            else:
                                dist_z1_active = 0
                                dist_z2_active = 0
                                dist_z2_timer_s = 0.0
                                dist_z2_timing_report_s = 0.0

                        if uv_enabled or ov_enabled:
                            voltage_eval_ready = fourier_valid == 1 and (
                                not args.normalize_to_comtrade or norm_ready == 1
                            )
                            cycle_dt_s = period_us / 1_000_000.0

                            if voltage_eval_ready:
                                if uv_enabled:
                                    prev_uv_active = uv_active
                                    prev_uv_trip = uv_trip
                                    if v1_mag <= uv_pickup:
                                        uv_active = 1
                                        uv_timer_s += cycle_dt_s
                                        if prev_uv_active == 0:
                                            protection_log(
                                                "[UV27 PICKUP] "
                                                f"t={dev_t_s:.6f}s V1={v1_mag:.6f} "
                                                f"pickup={uv_pickup:.6f} "
                                                f"timer={uv_timer_s:.6f}s delay={uv_delay_s:.6f}s"
                                            )
                                            uv_timing_report_s = uv_timer_s
                                        elif (
                                            args.protection_events
                                            and args.protection_event_interval > 0.0
                                            and uv_timer_s - uv_timing_report_s
                                            >= args.protection_event_interval
                                            and uv_trip == 0
                                        ):
                                            protection_log(
                                                "[UV27 TIMING] "
                                                f"t={dev_t_s:.6f}s V1={v1_mag:.6f} "
                                                f"timer={uv_timer_s:.6f}/{uv_delay_s:.6f}s"
                                            )
                                            uv_timing_report_s = uv_timer_s
                                        if uv_timer_s >= uv_delay_s:
                                            uv_trip = 1
                                            voltage_trip = 1
                                            if voltage_trip_code == "NONE":
                                                voltage_trip_code = "27"
                                            if prev_uv_trip == 0:
                                                protection_log(
                                                    "[UV27 TRIP] "
                                                    f"t={dev_t_s:.6f}s V1={v1_mag:.6f} "
                                                    f"pickup={uv_pickup:.6f} "
                                                    f"elapsed={uv_timer_s:.6f}s"
                                                )
                                    elif v1_mag > uv_dropout:
                                        if uv_active == 1 and uv_trip == 0:
                                            protection_log(
                                                "[UV27 RESET] "
                                                f"t={dev_t_s:.6f}s V1={v1_mag:.6f} "
                                                f"dropout={uv_dropout:.6f} "
                                                f"timer_reset={uv_timer_s:.6f}s"
                                            )
                                        uv_active = 0
                                        uv_timer_s = 0.0
                                        uv_timing_report_s = 0.0
                                    else:
                                        uv_active = 0

                                if ov_enabled:
                                    prev_ov_active = ov_active
                                    prev_ov_trip = ov_trip
                                    if v1_mag >= ov_pickup:
                                        ov_active = 1
                                        ov_timer_s += cycle_dt_s
                                        if prev_ov_active == 0:
                                            protection_log(
                                                "[OV59 PICKUP] "
                                                f"t={dev_t_s:.6f}s V1={v1_mag:.6f} "
                                                f"pickup={ov_pickup:.6f} "
                                                f"timer={ov_timer_s:.6f}s delay={ov_delay_s:.6f}s"
                                            )
                                            ov_timing_report_s = ov_timer_s
                                        elif (
                                            args.protection_events
                                            and args.protection_event_interval > 0.0
                                            and ov_timer_s - ov_timing_report_s
                                            >= args.protection_event_interval
                                            and ov_trip == 0
                                        ):
                                            protection_log(
                                                "[OV59 TIMING] "
                                                f"t={dev_t_s:.6f}s V1={v1_mag:.6f} "
                                                f"timer={ov_timer_s:.6f}/{ov_delay_s:.6f}s"
                                            )
                                            ov_timing_report_s = ov_timer_s
                                        if ov_timer_s >= ov_delay_s:
                                            ov_trip = 1
                                            voltage_trip = 1
                                            if voltage_trip_code == "NONE":
                                                voltage_trip_code = "59"
                                            if prev_ov_trip == 0:
                                                protection_log(
                                                    "[OV59 TRIP] "
                                                    f"t={dev_t_s:.6f}s V1={v1_mag:.6f} "
                                                    f"pickup={ov_pickup:.6f} "
                                                    f"elapsed={ov_timer_s:.6f}s"
                                                )
                                    elif v1_mag < ov_dropout:
                                        if ov_active == 1 and ov_trip == 0:
                                            protection_log(
                                                "[OV59 RESET] "
                                                f"t={dev_t_s:.6f}s V1={v1_mag:.6f} "
                                                f"dropout={ov_dropout:.6f} "
                                                f"timer_reset={ov_timer_s:.6f}s"
                                            )
                                        ov_active = 0
                                        ov_timer_s = 0.0
                                        ov_timing_report_s = 0.0
                                    else:
                                        ov_active = 0
                            else:
                                uv_active = 0
                                ov_active = 0
                                uv_timer_s = 0.0
                                ov_timer_s = 0.0
                                uv_timing_report_s = 0.0
                                ov_timing_report_s = 0.0

                        if (
                            args.protection_events
                            and breaker_trip_reported == 0
                            and (oc_trip == 1 or dist_trip == 1 or voltage_trip == 1)
                        ):
                            trip_sources: list[str] = []
                            if oc_50_trip == 1:
                                trip_sources.append("50")
                            if oc_51_trip == 1:
                                trip_sources.append("51")
                            if dist_z1_trip == 1:
                                trip_sources.append("21Z1")
                            if dist_z2_trip == 1:
                                trip_sources.append("21Z2")
                            if uv_trip == 1:
                                trip_sources.append("27")
                            if ov_trip == 1:
                                trip_sources.append("59")
                            protection_log(
                                "[BREAKER TRIP] "
                                f"t={dev_t_s:.6f}s source={';'.join(trip_sources)} "
                                f"I1={i1_mag:.6f}A V1={v1_mag:.6f} "
                                f"|Z|={dist_z_mag_ohm:.6f}ohm "
                                f"R={dist_r_ohm:.6f}ohm X={dist_x_ohm:.6f}ohm "
                                f"fault={dist_fault_ohm:.6f}ohm ({dist_fault_pct:.3f}% line)"
                            )
                            breaker_trip_reported = 1

                        cycle_update = 1
                        cycle_id += 1

                    # Inicia novo ciclo
                    cycle_start_us = dev_t_us
                    cyc_n = 0
                    cyc_sum_v2 = 0.0
                    cyc_sum_i2 = 0.0
                    cyc_v_samples = []
                    cyc_i_samples = []
                    cyc_flags = 0

                state = new_state

                # Acumula no ciclo atual
                if cycle_start_us is not None:
                    cyc_n += 1
                    cyc_sum_v2 += v_inst * v_inst
                    cyc_sum_i2 += i_inst * i_inst
                    cyc_v_samples.append(v_inst)
                    cyc_i_samples.append(i_inst)

                host_t_s = time.perf_counter() - t0
                dev_t_s = dev_t_us / 1_000_000.0

                w.writerow([
                    f"{host_t_s:.9f}",
                    f"{dev_t_s:.9f}",
                    dev_t_us,
                    seq,
                    adc_34,
                    adc_35,
                    f"{v_adc_34:.9f}",
                    f"{v_adc_35:.9f}",
                    f"{v_offset_est:.9f}",
                    f"{i_offset_est:.9f}",
                    f"{v_inst:.9f}",
                    f"{i_inst:.9f}",
                    f"{v_rms_raw:.9f}",
                    f"{i_rms_raw:.9f}",
                    f"{v_rms:.9f}",
                    f"{i_rms:.9f}",
                    f"{v1_mag_raw:.9f}",
                    f"{i1_mag_raw:.9f}",
                    f"{v1_mag:.9f}",
                    f"{i1_mag:.9f}",
                    f"{v1_angle_deg:.6f}",
                    f"{i1_angle_deg:.6f}",
                    f"{vi_angle_deg:.6f}",
                    fourier_valid,
                    f"{f_est:.6f}",
                    rms_valid,
                    norm_ready,
                    f"{norm_gain_v:.9f}",
                    f"{norm_gain_i:.9f}",
                    int(oc_enabled),
                    f"{oc_i_nominal if oc_i_nominal is not None else 0.0:.9f}",
                    f"{oc_51_pickup:.9f}",
                    f"{oc_50_pickup:.9f}",
                    f"{oc_51_timer_s:.9f}",
                    oc_51_active,
                    oc_50_trip,
                    oc_51_trip,
                    oc_trip,
                    oc_trip_code,
                    int(dist_enabled),
                    f"{dist_line_z_ohm:.9f}",
                    f"{dist_z1_pct:.9f}",
                    f"{dist_z2_pct:.9f}",
                    f"{dist_z1_ohm:.9f}",
                    f"{dist_z2_ohm:.9f}",
                    f"{dist_min_current:.9f}",
                    f"{dist_z_mag_ohm:.9f}",
                    f"{dist_z_angle_deg:.6f}",
                    f"{dist_r_ohm:.9f}",
                    f"{dist_x_ohm:.9f}",
                    f"{dist_fault_pct:.9f}",
                    f"{dist_fault_ohm:.9f}",
                    dist_z1_active,
                    dist_z2_active,
                    f"{dist_z2_timer_s:.9f}",
                    dist_z1_trip,
                    dist_z2_trip,
                    dist_trip,
                    dist_trip_code,
                    int(uv_enabled),
                    int(ov_enabled),
                    f"{vprot_nominal if vprot_nominal is not None else 0.0:.9f}",
                    f"{uv_pickup:.9f}",
                    f"{ov_pickup:.9f}",
                    f"{uv_timer_s:.9f}",
                    f"{ov_timer_s:.9f}",
                    uv_active,
                    ov_active,
                    uv_trip,
                    ov_trip,
                    voltage_trip,
                    voltage_trip_code,
                    cycle_id,
                    cycle_update,
                    last_cycle_quality_flags,
                    lost_total,
                    bad_checksum_total,
                    desync_total,
                ])

                # Evita perder dados quando o processo for interrompido externamente.
                if (sample_count % 20) == 0:
                    f.flush()
                    os.fsync(f.fileno())

                if args.print_every > 0 and (sample_count % args.print_every) == 0:
                    print(
                        f"status amostras={sample_count} "
                        f"V={v_inst:.4f} I={i_inst:.4f} "
                        f"Vrms={v_rms:.4f} Irms={i_rms:.4f} "
                        f"V1={v1_mag:.4f} I1={i1_mag:.4f} "
                        f"f={f_est:.3f}Hz valid={rms_valid} qf={last_cycle_quality_flags} "
                        f"OC51={oc_51_active}/{oc_51_timer_s:.3f}s "
                        f"OC50={oc_50_trip} trip={oc_trip_code} "
                        f"Z={dist_z_mag_ohm:.4f}ohm ({dist_fault_pct:.2f}% line) D21={dist_trip_code} "
                        f"V27={uv_trip} V59={ov_trip} "
                        f"norm={norm_ready} lost={lost_total} bad={bad_checksum_total} ds={desync_total}"
                    )
                    f.flush()


if __name__ == "__main__":
    main()

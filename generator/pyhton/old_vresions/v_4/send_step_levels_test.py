#!/usr/bin/env python3
"""
Teste de niveis com loopback interno na ESP32 geradora.

Cicla:
- 0 mV por 3 s
- 1650 mV por 3 s
- 3000 mV por 3 s

Enquanto isso, recebe amostras ADC da propria geradora e salva CSV.
"""

from __future__ import annotations

import argparse
import csv
import struct
import time
import tempfile
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial nao esta instalado. Use: pip install pyserial") from exc

H0 = 0xAB
H1 = 0xCD
TAIL = 0xBA
FRAME_LEN = 14


def _stable_rows(rows):
    """
    Mantém apenas regiões estáveis:
    - set_mv_cmd == set_mv_esp
    - remove bordas de transição de cada run (fica com miolo)
    """
    if not rows:
        return []

    runs = []
    start = 0
    prev = rows[0]["set_mv_esp"]
    for i in range(1, len(rows)):
        cur = rows[i]["set_mv_esp"]
        if cur != prev:
            runs.append((start, i))
            start = i
            prev = cur
    runs.append((start, len(rows)))

    stable = []
    for a, b in runs:
        run = rows[a:b]
        n = len(run)
        if n < 20:
            continue
        cut = max(3, int(0.15 * n))
        core = run[cut:n - cut] if (n - 2 * cut) > 0 else []
        for r in core:
            if r["set_mv_cmd"] == r["set_mv_esp"]:
                stable.append(r)
    return stable


def _build_points(rows, adc_key: str):
    by_set = {}
    for r in rows:
        set_mv = r["set_mv_esp"]
        by_set.setdefault(set_mv, []).append(r[adc_key])

    pts = []
    for set_mv in sorted(by_set.keys()):
        arr = by_set[set_mv]
        if not arr:
            continue
        mean_adc = sum(arr) / len(arr)
        pts.append((mean_adc, float(set_mv)))
    return pts


def _map_piecewise(adc: int, points):
    """
    Interpolação por segmentos de (adc_mean -> mv).
    Faz extrapolação linear nas pontas.
    """
    if not points:
        return adc * (3300.0 / 4095.0)

    pts = sorted(points, key=lambda p: p[0])
    if len(pts) == 1:
        x0, y0 = pts[0]
        if abs(x0) < 1e-9:
            return y0
        return (adc / x0) * y0

    if adc <= pts[0][0]:
        x0, y0 = pts[0]
        x1, y1 = pts[1]
        m = (y1 - y0) / max(1e-9, (x1 - x0))
        return y0 + m * (adc - x0)

    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        if x0 <= adc <= x1:
            alpha = (adc - x0) / max(1e-9, (x1 - x0))
            return y0 + alpha * (y1 - y0)

    x0, y0 = pts[-2]
    x1, y1 = pts[-1]
    m = (y1 - y0) / max(1e-9, (x1 - x0))
    return y1 + m * (adc - x1)


def calibrate_csv_inplace(
    csv_path: str,
    *,
    midpoint_mv: int = 1650,
    anchor_midpoint: bool = True,
) -> None:
    """
    Le CSV bruto, calcula calibracao por loopback (por canal) e
    reescreve o mesmo CSV com colunas corrigidas.
    """
    src = Path(csv_path)
    rows = []

    with src.open("r", newline="") as f:
        rd = csv.DictReader(f)
        for r in rd:
            rows.append({
                "host_t_s": r["host_t_s"],
                "esp_t_us": int(r["esp_t_us"]),
                "set_mv_cmd": int(r["set_mv_cmd"]),
                "set_mv_esp": int(r["set_mv_esp"]),
                "adc_v": int(r["adc_v"]),
                "adc_i": int(r["adc_i"]),
                "adc_v_volts": r["adc_v_volts"],
                "adc_i_volts": r["adc_i_volts"],
            })

    stable = _stable_rows(rows)
    calib_rows = stable if len(stable) >= 30 else rows

    points_v = _build_points(calib_rows, "adc_v")
    points_i = _build_points(calib_rows, "adc_i")

    # Ancoragem opcional: força ponto médio conhecido.
    if anchor_midpoint:
        mid_v = [r["adc_v"] for r in calib_rows if r["set_mv_esp"] == midpoint_mv]
        mid_i = [r["adc_i"] for r in calib_rows if r["set_mv_esp"] == midpoint_mv]
        if mid_v and mid_i:
            mean_mid_v = sum(mid_v) / len(mid_v)
            mean_mid_i = sum(mid_i) / len(mid_i)

            def _anchor(points, x_mid):
                y_mid_now = _map_piecewise(int(round(x_mid)), points)
                delta = float(midpoint_mv) - y_mid_now
                return [(x, y + delta) for (x, y) in points]

            points_v = _anchor(points_v, mean_mid_v)
            points_i = _anchor(points_i, mean_mid_i)

    with tempfile.NamedTemporaryFile("w", newline="", delete=False, dir=str(src.parent)) as tf:
        wr = csv.writer(tf)
        wr.writerow([
            "host_t_s",
            "esp_t_us",
            "set_mv_cmd",
            "set_mv_esp",
            "adc_v",
            "adc_i",
            "adc_v_volts",
            "adc_i_volts",
            "mv_v_corr",
            "mv_i_corr",
            "err_v_mv",
            "err_i_mv",
        ])

        for r in rows:
            set_mv_cmd = r["set_mv_cmd"]
            set_mv_esp = r["set_mv_esp"]
            adc_v = r["adc_v"]
            adc_i = r["adc_i"]

            mv_v_corr = _map_piecewise(adc_v, points_v)
            mv_i_corr = _map_piecewise(adc_i, points_i)
            err_v = mv_v_corr - set_mv_esp
            err_i = mv_i_corr - set_mv_esp

            wr.writerow([
                r["host_t_s"],
                r["esp_t_us"],
                set_mv_cmd,
                set_mv_esp,
                adc_v,
                adc_i,
                r["adc_v_volts"],
                r["adc_i_volts"],
                f"{mv_v_corr:.3f}",
                f"{mv_i_corr:.3f}",
                f"{err_v:.3f}",
                f"{err_i:.3f}",
            ])

    Path(tf.name).replace(src)

    print("Calibracao loopback aplicada ao CSV (segmentada):")
    print(f"  Pontos V (adc->mV): {[(round(x,2), round(y,2)) for x, y in points_v]}")
    print(f"  Pontos I (adc->mV): {[(round(x,2), round(y,2)) for x, y in points_i]}")
    print(f"  Amostras usadas na calibracao: {len(calib_rows)} de {len(rows)}")


def send_cmd(ser: serial.Serial, cmd: str) -> None:
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()


def parse_frames(buf: bytearray):
    """Extrai frames validos do buffer e retorna lista de tuplas."""
    out = []
    while True:
        i = buf.find(bytes((H0, H1)))
        if i < 0:
            # preserva no maximo 1 byte para detectar cabecalho quebrado
            if len(buf) > 1:
                del buf[:-1]
            break

        if len(buf) < i + FRAME_LEN:
            if i > 0:
                del buf[:i]
            break

        fr = bytes(buf[i:i + FRAME_LEN])
        if fr[13] != TAIL:
            del buf[:i + 1]
            continue

        chk = sum(fr[:12]) & 0xFF
        if fr[12] != chk:
            del buf[:i + 1]
            continue

        t_us = struct.unpack_from("<I", fr, 2)[0]
        adc_v = struct.unpack_from("<H", fr, 6)[0]
        adc_i = struct.unpack_from("<H", fr, 8)[0]
        set_mv_esp = struct.unpack_from("<H", fr, 10)[0]

        if adc_v <= 4095 and adc_i <= 4095 and set_mv_esp <= 3300:
            out.append((t_us, adc_v, adc_i, set_mv_esp))

        del buf[:i + FRAME_LEN]

    return out


def capture_for(
    ser: serial.Serial,
    writer: csv.writer,
    parser_buf: bytearray,
    duration_s: float,
    set_mv_cmd: int,
    host_t0: float,
) -> int:
    end_t = time.perf_counter() + duration_s
    rows = 0

    while time.perf_counter() < end_t:
        n = ser.in_waiting
        data = ser.read(n if n > 0 else 1)
        if not data:
            continue

        parser_buf.extend(data)
        frames = parse_frames(parser_buf)

        for t_us, adc_v, adc_i, set_mv_esp in frames:
            host_ts = time.perf_counter() - host_t0
            v_v = (adc_v * 3.3) / 4095.0
            v_i = (adc_i * 3.3) / 4095.0
            writer.writerow([
                f"{host_ts:.6f}",
                t_us,
                set_mv_cmd,
                set_mv_esp,
                adc_v,
                adc_i,
                f"{v_v:.6f}",
                f"{v_i:.6f}",
            ])
            rows += 1

    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Teste 0/1.65/3.0 V com CSV de loopback ADC da geradora")
    parser.add_argument("--port", required=True, help="Porta serial (ex.: /dev/ttyUSB0 ou COM5)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    parser.add_argument("--hold", type=float, default=3.0, help="Tempo por nivel em segundos (default: 3.0)")
    parser.add_argument("--csv", default="adc_capture_generator_loopback.csv", help="Arquivo CSV de saida")
    parser.add_argument(
        "--midpoint-mv",
        type=int,
        default=1650,
        help="Ponto de referencia (mV) para ancorar automaticamente a calibracao (default: 1650)",
    )
    parser.add_argument(
        "--no-mid-anchor",
        action="store_true",
        help="Nao ancora a calibracao no ponto medio",
    )
    parser.add_argument(
        "--no-calibrate",
        action="store_true",
        help="Nao aplica calibracao automatica por loopback ao final",
    )
    args = parser.parse_args()

    levels_mv = [0, 1650, 3000]

    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=1.0) as ser, \
         open(args.csv, "w", newline="") as f:

        writer = csv.writer(f)
        writer.writerow([
            "host_t_s",
            "esp_t_us",
            "set_mv_cmd",
            "set_mv_esp",
            "adc_v",
            "adc_i",
            "adc_v_volts",
            "adc_i_volts",
        ])

        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        parser_buf = bytearray()
        host_t0 = time.perf_counter()

        print(f"Conectado em {args.port} @ {args.baud} bps")
        print(f"Salvando CSV em: {args.csv}")
        print("Ciclo: 0mV -> 1650mV -> 3000mV (Ctrl+C para parar)")

        total_rows = 0

        try:
            while True:
                for mv in levels_mv:
                    send_cmd(ser, f"SET {mv}")
                    rows = capture_for(
                        ser=ser,
                        writer=writer,
                        parser_buf=parser_buf,
                        duration_s=args.hold,
                        set_mv_cmd=mv,
                        host_t0=host_t0,
                    )
                    total_rows += rows
                    f.flush()
                    print(f"Nivel {mv} mV por {args.hold:.3f}s -> {rows} amostras")
        except KeyboardInterrupt:
            send_cmd(ser, "STOP")
            f.flush()
            print(f"\nInterrompido pelo usuario. Total de amostras: {total_rows}")

    if not args.no_calibrate:
        calibrate_csv_inplace(
            args.csv,
            midpoint_mv=args.midpoint_mv,
            anchor_midpoint=not args.no_mid_anchor,
        )
    else:
        print("Calibracao automatica desativada (--no-calibrate).")


if __name__ == "__main__":
    main()

import argparse
import csv
import math
import struct
import time
from collections import deque

import serial

H0 = 0xA5
H1 = 0x5A
FRAME_SIZE = 13
ADC_MAX = 4095.0


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


class SlidingRMS:
    def __init__(self, window_n: int) -> None:
        self.window_n = max(8, int(window_n))
        self.buf = deque()
        self.sum_x = 0.0
        self.sum_x2 = 0.0

    def push(self, x: float) -> float:
        self.buf.append(x)
        self.sum_x += x
        self.sum_x2 += x * x

        if len(self.buf) > self.window_n:
            old = self.buf.popleft()
            self.sum_x -= old
            self.sum_x2 -= old * old

        n = len(self.buf)
        if n < 2:
            return 0.0

        mean = self.sum_x / n
        mean2 = self.sum_x2 / n
        var = max(0.0, mean2 - mean * mean)
        return math.sqrt(var)


def adc_to_volts(adc: int, adc_vref: float) -> float:
    return (float(adc) / ADC_MAX) * adc_vref


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Leitura 2 canais (GPIO34=V, GPIO35=I) com RMS em tempo real."
    )
    parser.add_argument("--port", default="/dev/ttyUSB1", help="Porta serial da ESP32")
    parser.add_argument("--baud", type=int, default=921600, help="Baudrate serial")
    parser.add_argument("--out", default="adc_capture.csv", help="CSV de saída")
    parser.add_argument("--freq", type=float, default=60.0, help="Frequência fundamental (Hz)")
    parser.add_argument("--cycles", type=float, default=1.0, help="Janela RMS em ciclos")
    parser.add_argument("--sample-rate", type=float, default=2000.0, help="Taxa de amostragem esperada (Hz)")
    parser.add_argument("--adc-vref", type=float, default=3.3, help="Referência ADC (V)")
    parser.add_argument("--v-offset", type=float, default=1.65, help="Offset do canal de tensão (V)")
    parser.add_argument("--i-offset", type=float, default=1.65, help="Offset do canal de corrente (V)")
    parser.add_argument("--v-scale", type=float, default=1.0, help="Escala para tensão real (V/V)")
    parser.add_argument("--i-scale", type=float, default=1.0, help="Escala para corrente real (A/V)")
    parser.add_argument("--print-every", type=int, default=200, help="Imprimir status a cada N amostras")
    args = parser.parse_args()

    window_n = max(8, int(round(args.sample_rate * args.cycles / args.freq)))
    v_rms_est = SlidingRMS(window_n)
    i_rms_est = SlidingRMS(window_n)

    ser = serial.Serial(args.port, args.baud, timeout=0.25)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    buf = bytearray()
    t0 = time.perf_counter()
    last_seq = None
    lost_total = 0
    bad_checksum = 0
    desync = 0
    sample_count = 0

    with open(args.out, "w", newline="", encoding="utf-8") as f:
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
            "v_real",
            "i_real",
            "v_rms",
            "i_rms",
            "lost_frames_total",
            "bad_checksum_total",
            "desync_total",
            "window_samples",
        ])

        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= FRAME_SIZE:
                if not (buf[0] == H0 and buf[1] == H1):
                    del buf[0]
                    desync += 1
                    continue

                frame = bytes(buf[:FRAME_SIZE])
                parsed = parse_frame(frame)
                if parsed is None:
                    del buf[0]
                    bad_checksum += 1
                    continue

                del buf[:FRAME_SIZE]
                seq, dev_t_us, adc_34, adc_35 = parsed

                if last_seq is not None:
                    delta = (seq - last_seq) & 0xFFFF
                    if delta > 1:
                        lost_total += (delta - 1)
                last_seq = seq
                sample_count += 1

                host_t_s = time.perf_counter() - t0
                dev_t_s = dev_t_us / 1_000_000.0

                v_adc_34 = adc_to_volts(adc_34, args.adc_vref)
                v_adc_35 = adc_to_volts(adc_35, args.adc_vref)

                # Canal 34 = tensão, canal 35 = corrente
                v_real = (v_adc_34 - args.v_offset) * args.v_scale
                i_real = (v_adc_35 - args.i_offset) * args.i_scale

                v_rms = v_rms_est.push(v_real)
                i_rms = i_rms_est.push(i_real)

                w.writerow([
                    f"{host_t_s:.9f}",
                    f"{dev_t_s:.9f}",
                    dev_t_us,
                    seq,
                    adc_34,
                    adc_35,
                    f"{v_adc_34:.9f}",
                    f"{v_adc_35:.9f}",
                    f"{v_real:.9f}",
                    f"{i_real:.9f}",
                    f"{v_rms:.9f}",
                    f"{i_rms:.9f}",
                    lost_total,
                    bad_checksum,
                    desync,
                    window_n,
                ])

                if (sample_count % args.print_every) == 0:
                    print(
                        f"seq={seq} V={v_real:.4f} I={i_real:.4f} "
                        f"Vrms={v_rms:.4f} Irms={i_rms:.4f} "
                        f"lost={lost_total} bad={bad_checksum} desync={desync}"
                    )
                    f.flush()


if __name__ == "__main__":
    main()

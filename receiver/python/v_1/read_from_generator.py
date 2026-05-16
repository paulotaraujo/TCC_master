import csv
import serial
import struct
import time

PORT = "/dev/ttyUSB1"
BAUD = 921600
OUT_CSV = "adc_capture.csv"

H0 = 0xA5
H1 = 0x5A
FRAME_SIZE = 13


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


def main() -> None:
    ser = serial.Serial(PORT, BAUD, timeout=0.25)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    buf = bytearray()
    t0 = time.perf_counter()
    last_seq = None
    lost_total = 0
    bad_checksum = 0
    desync = 0

    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "host_t_s",
            "dev_t_s",
            "dev_t_us",
            "seq",
            "adc_34",
            "adc_35",
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

                host_t_s = time.perf_counter() - t0
                dev_t_s = dev_t_us / 1_000_000.0

                w.writerow([
                    f"{host_t_s:.9f}",
                    f"{dev_t_s:.9f}",
                    dev_t_us,
                    seq,
                    adc_34,
                    adc_35,
                    lost_total,
                    bad_checksum,
                    desync,
                ])


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
import struct
import time
from pathlib import Path

import serial

SYNC = b"\xAA\x55"
FRAME_LEN = 8                      # 2 sync + 4 time + 2 adc
PAYLOAD_FMT = "<IH"                # time_us(uint32), adc_raw(uint16)
PAYLOAD_LEN = 6

def find_sync(buf: bytearray) -> int:
    """Retorna o índice do sync AA55 no buffer, ou -1 se não achar."""
    # procura AA55
    return buf.find(SYNC)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Ex: /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", default="rx.csv")
    ap.add_argument("--duration", type=float, default=0.0, help="segundos (0 = infinito)")
    ap.add_argument("--chunk", type=int, default=4096, help="bytes por leitura")
    ap.add_argument("--report_every", type=float, default=2.0, help="segundos")
    args = ap.parse_args()

    out_path = Path(args.out)

    with serial.Serial(args.port, args.baud, timeout=1) as ser, out_path.open("w", buffering=1) as f:
        # evita reset chato em algumas interfaces
        ser.dtr = False
        ser.rts = False

        print(f"[INFO] Abrindo {args.port} @ {args.baud}")
        print("[INFO] Aguardando 1.5s (boot/estabilizar)...")
        time.sleep(1.5)
        ser.reset_input_buffer()

        f.write("time_us,adc_raw\n")
        print(f"[INFO] Gravando em {out_path.resolve()}")

        buf = bytearray()
        t0 = time.time()
        last_report = t0
        n = 0
        desync_drops = 0

        while True:
            if args.duration > 0 and (time.time() - t0) >= args.duration:
                break

            chunk = ser.read(args.chunk)
            if not chunk:
                continue
            buf.extend(chunk)

            # tenta extrair frames completos
            while True:
                i = find_sync(buf)
                if i < 0:
                    # mantém só o finalzinho para não crescer infinito
                    if len(buf) > 4096:
                        del buf[:-2]
                    break

                # descarta lixo antes do sync
                if i > 0:
                    desync_drops += i
                    del buf[:i]

                # precisa ter frame completo
                if len(buf) < FRAME_LEN:
                    break

                # agora buf[0:2] é SYNC, buf[2:8] é payload
                payload = bytes(buf[2:8])
                del buf[:FRAME_LEN]

                t_us, adc_raw = struct.unpack(PAYLOAD_FMT, payload)

                # sanity check opcional (ajuda a detectar qualquer bug)
                if adc_raw > 4095:
                    # Se isso acontecer, ainda tem algo errado no caminho,
                    # mas com sync é muito raro. Vamos ignorar essa amostra.
                    continue

                f.write(f"{t_us},{adc_raw}\n")
                n += 1

            now = time.time()
            if now - last_report >= args.report_every:
                elapsed = now - t0
                rate = n / elapsed if elapsed > 0 else 0.0
                print(f"[INFO] {n} amostras | {rate:.1f} Hz | bytes descartados (dessync): {desync_drops}")
                last_report = now

        elapsed = time.time() - t0
        rate = n / elapsed if elapsed > 0 else 0.0
        print(f"\n[OK] Finalizado. Total: {n} amostras em {elapsed:.2f}s | {rate:.1f} Hz")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Interrompido.")

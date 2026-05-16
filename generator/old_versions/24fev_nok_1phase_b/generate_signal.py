#!/usr/bin/env python3
import argparse
import struct
import time
import serial
from pathlib import Path

def wait_ready(ser, timeout_s=10):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("ESP32:", line)
        if line == "READY":
            return True
    return False

def send_cfg(ser, cfg_path: Path):
    text = cfg_path.read_text(encoding="utf-8", errors="ignore")
    # envia exatamente como arquivo (garante \n no final)
    if not text.endswith("\n"):
        text += "\n"
    ser.write(text.encode("utf-8", errors="ignore"))
    ser.write(b"ENDCFG\n")
    ser.flush()

def compute_total_records(bdat_path: Path, rec_size: int) -> int:
    size = bdat_path.stat().st_size
    if size % rec_size != 0:
        raise ValueError(f"Tamanho do BDAT ({size}) não é múltiplo de rec_size ({rec_size}).")
    return size // rec_size

def send_bdat(ser, bdat_path: Path, ts64: int, rec_size: int, total_records: int):
    # Header: magic[4], ts64(uint8), totalRecords(uint32)
    hdr = struct.pack("<4sBI", b"BDAT", int(ts64) & 0xFF, int(total_records) & 0xFFFFFFFF)
    ser.write(hdr)

    with bdat_path.open("rb") as f:
        while True:
            chunk = f.read(64 * 1024)
            if not chunk:
                break
            ser.write(chunk)
    ser.flush()

def main():
    ap = argparse.ArgumentParser(description="Envia COMTRADE (.cfg + .bdat) para ESP32 via serial.")
    ap.add_argument("--port", required=True, help="Ex: /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="Baudrate (tem que bater com o Serial.begin do sketch)")
    ap.add_argument("--cfg", required=True, help="Caminho do .cfg")
    ap.add_argument("--bdat", required=True, help="Caminho do .bdat")
    ap.add_argument("--ts64", type=int, choices=[0,1], default=0, help="0=timestamp 32-bit, 1=timestamp 64-bit")
    ap.add_argument("--rec-size", type=int, required=True, help="Tamanho do registro BDAT em bytes (igual ao seu parser)")
    ap.add_argument("--total-records", type=int, default=-1, help="Se não passar, calcula pelo tamanho do arquivo/rec-size")
    args = ap.parse_args()

    cfg_path = Path(args.cfg)
    bdat_path = Path(args.bdat)

    if args.total_records < 0:
        total_records = compute_total_records(bdat_path, args.rec_size)
    else:
        total_records = args.total_records

    print(f"CFG:  {cfg_path}")
    print(f"BDAT: {bdat_path}")
    print(f"port={args.port} baud={args.baud} ts64={args.ts64} rec_size={args.rec_size} total_records={total_records}")

    ser = serial.Serial(args.port, args.baud, timeout=1)

    # muito comum abrir a porta resetar a ESP32
    time.sleep(1.5)
    ser.reset_input_buffer()

    if not wait_ready(ser, timeout_s=10):
        raise RuntimeError("Não recebi READY da ESP32. Verifique porta/baud e se o sketch está rodando.")

    print("Enviando CFG...")
    send_cfg(ser, cfg_path)

    print("Enviando BDAT header + dados...")
    send_bdat(ser, bdat_path, args.ts64, args.rec_size, total_records)

    print("✅ Envio concluído.")
    ser.close()

if __name__ == "__main__":
    main()
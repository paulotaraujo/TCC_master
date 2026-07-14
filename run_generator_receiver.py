#!/usr/bin/env python3
import argparse
import struct
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None


def _resolve_existing_path(path_str: str, base_dirs: list[Path]) -> Path:
    p = Path(path_str)
    if p.is_absolute() and p.exists():
        return p
    if p.exists():
        return p.resolve()
    for base in base_dirs:
        c = (base / p).resolve()
        if c.exists():
            return c
    return p.resolve()


def _split_extra(extra_values: list[str]) -> list[str]:
    out: list[str] = []
    for item in extra_values:
        out.extend(shlex.split(item))
    return out


def _reader_thread(name: str, stream) -> None:
    try:
        for line in iter(stream.readline, ""):
            if not line:
                break
            print(f"[{name}] {line}", end="")
    finally:
        try:
            stream.close()
        except Exception:
            pass


def _terminate_process(proc: subprocess.Popen, label: str, timeout_s: float = 3.0) -> None:
    if proc.poll() is not None:
        return
    print(f"[CTRL] Encerrando {label} (pid={proc.pid})...")
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        proc.terminate()
        proc.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    print(f"[CTRL] Forçando kill de {label} (pid={proc.pid})...")
    try:
        proc.kill()
        proc.wait(timeout=timeout_s)
    except Exception:
        pass


def _probe_esp32_ready(label: str, port: str | None, baud: int, timeout_s: float) -> bool:
    if not port:
        print(f"[CTRL] {label}: porta não informada, pulando checagem de READY.")
        return False
    if serial is None:
        print(f"[CTRL] {label}: pyserial indisponível no runner, pulando checagem de READY.")
        return False

    print(f"[CTRL] {label}: checando comunicação em {port} @ {baud}...")
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.25, write_timeout=1.0)
    except Exception as exc:
        print(f"[CTRL] {label}: falha ao abrir porta {port}: {exc}")
        return False

    try:
        try:
            ser.setDTR(False)
            ser.setRTS(False)
        except Exception:
            pass
        time.sleep(2.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        t0 = time.time()
        seen_ready = False
        while time.time() - t0 < timeout_s:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode(errors="ignore").strip()
            print(f"[{label}] [ESP32] {text}")
            if "READY" in text:
                seen_ready = True
                break
        if seen_ready:
            print(f"[CTRL] {label}: READY confirmado.")
        else:
            print(f"[CTRL] {label}: READY não recebido em {timeout_s:.1f}s.")
        return seen_ready
    finally:
        try:
            ser.close()
        except Exception:
            pass


def _checksum8(buf: bytes) -> int:
    return sum(buf) & 0xFF


def _probe_rx_binary(label: str, port: str | None, baud: int, timeout_s: float) -> bool:
    if not port:
        print(f"[CTRL] {label}: porta não informada, pulando checagem binária.")
        return False
    if serial is None:
        print(f"[CTRL] {label}: pyserial indisponível no runner, pulando checagem binária.")
        return False

    print(f"[CTRL] {label}: checando stream binário em {port} @ {baud}...")
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.25, write_timeout=1.0)
    except Exception as exc:
        print(f"[CTRL] {label}: falha ao abrir porta {port}: {exc}")
        return False

    h0 = 0xA5
    h1 = 0x5A
    frame_size = 13
    valid = 0
    total = 0
    last_seq = None
    t0 = time.time()
    buf = bytearray()
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        while time.time() - t0 < timeout_s:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= frame_size:
                if buf[0] != h0 or buf[1] != h1:
                    del buf[0]
                    continue
                frame = bytes(buf[:frame_size])
                del buf[:frame_size]
                total += 1
                if _checksum8(frame[:12]) != frame[12]:
                    continue
                seq = struct.unpack_from("<H", frame, 2)[0]
                t_us = struct.unpack_from("<I", frame, 4)[0]
                adc_34 = struct.unpack_from("<H", frame, 8)[0]
                adc_35 = struct.unpack_from("<H", frame, 10)[0]
                valid += 1
                last_seq = seq
                if valid <= 5:
                    print(f"[{label}] [BIN] seq={seq} t_us={t_us} adc34={adc_34} adc35={adc_35}")
            if valid >= 8:
                break

        if valid > 0:
            print(f"[CTRL] {label}: stream binário OK (frames_validos={valid}, frames_lidos={total}, last_seq={last_seq}).")
            return True
        print(f"[CTRL] {label}: nenhum frame binário válido detectado em {timeout_s:.1f}s.")
        return False
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main() -> None:
    root = Path(__file__).resolve().parent
    default_gen = root / "generator/python/embedded_generator/run_embedded_generator.py"
    default_rx = root / "receiver/python/v_1/read_from_generator_rms_robust.py"

    parser = argparse.ArgumentParser(
        description=(
            "Runner unificado para iniciar gerador COMTRADE e receptor RMS robusto "
            "em paralelo, com logs prefixados."
        )
    )

    parser.add_argument("--python", default="python3", help="Interpretador Python a usar")
    parser.add_argument("--gen-script", default=str(default_gen), help="Script do gerador")
    parser.add_argument("--rx-script", default=str(default_rx), help="Script do receptor")
    parser.add_argument("--start-delay", type=float, default=1.0, help="Atraso (s) entre iniciar RX e GEN")
    parser.add_argument(
        "--prepare-config-first",
        action="store_true",
        default=True,
        help="Executa gerador em --export-config-only antes de iniciar RX/GEN (recomendado).",
    )
    parser.add_argument(
        "--no-prepare-config-first",
        dest="prepare_config_first",
        action="store_false",
        help="Desativa a pré-geração do config.json.",
    )
    parser.add_argument(
        "--stop-on-first-exit",
        action="store_true",
        help="Se um processo terminar, encerra o outro automaticamente.",
    )
    parser.add_argument(
        "--probe-ready",
        action="store_true",
        default=False,
        help=(
            "Checa e imprime READY/OCC da ESP32 antes de iniciar os processos. "
            "Atenção: pode reabrir as portas seriais e reinicializar placas."
        ),
    )
    parser.add_argument(
        "--no-probe-ready",
        dest="probe_ready",
        action="store_false",
        help="Desativa checagem prévia de READY/OCC.",
    )
    parser.add_argument(
        "--probe-timeout",
        type=float,
        default=12.0,
        help="Timeout (s) para checagem READY/OCC de cada porta serial.",
    )

    # Gerador (baseado no run_embedded_generator.py)
    parser.add_argument("--gen-cfg", required=True, help="Caminho do .cfg do COMTRADE")
    parser.add_argument("--gen-bdat", required=True, help="Caminho do .bdat do COMTRADE")
    parser.add_argument("--gen-port", default=None, help="Porta serial da ESP32 geradora")
    parser.add_argument("--gen-baud", type=int, default=921600, help="Baudrate do gerador")
    parser.add_argument("--gen-out", default=None, help="CSV opcional do gerador")
    parser.add_argument("--gen-ready-timeout", type=float, default=12.0, help="Timeout READY gerador")
    parser.add_argument(
        "--gen-receiver-config",
        default=None,
        help="Arquivo JSON de config a ser gerado pelo gerador para o receptor",
    )
    parser.add_argument(
        "--gen-export-config-only",
        action="store_true",
        help="Somente exporta config no gerador e encerra (sem reprodução serial).",
    )
    parser.add_argument(
        "--gen-extra",
        action="append",
        default=[],
        help="Argumentos extras crus para o gerador (pode repetir, usa shlex).",
    )

    # Receptor (baseado no read_from_generator_rms_robust.py)
    parser.add_argument("--rx-port", default="/dev/ttyUSB1", help="Porta serial da ESP32 receptora")
    parser.add_argument("--rx-baud", type=int, default=921600, help="Baudrate do receptor")
    parser.add_argument("--rx-out", default="adc_capture.csv", help="CSV de saída do receptor (canônico no root do projeto)")
    parser.add_argument("--rx-config", default=None, help="Config JSON para receptor")
    parser.add_argument("--rx-adc-vref", type=float, default=3.3, help="Referência ADC receptor")
    parser.add_argument("--rx-sample-rate", type=float, default=1000.0, help="Taxa nominal receptor (Hz)")
    parser.add_argument("--rx-offset-tau", type=float, default=1.0, help="Tau auto-offset receptor (s)")
    parser.add_argument("--rx-v-scale", type=float, default=1.0, help="Escala tensão receptor")
    parser.add_argument("--rx-i-scale", type=float, default=1.0, help="Escala corrente receptor")
    parser.add_argument("--rx-zc-hyst", type=float, default=0.03, help="Histerese zero-cross receptor")
    parser.add_argument("--rx-outlier-step-v", type=float, default=None, help="Outlier step V receptor")
    parser.add_argument("--rx-outlier-step-i", type=float, default=None, help="Outlier step I receptor")
    parser.add_argument("--rx-f-min", type=float, default=55.0, help="Frequência mínima válida receptor")
    parser.add_argument("--rx-f-max", type=float, default=65.0, help="Frequência máxima válida receptor")
    parser.add_argument(
        "--rx-normalize-to-comtrade",
        action="store_true",
        help="Ativa normalização RMS para referência COMTRADE no receptor",
    )
    parser.add_argument("--rx-prefault-cycles", type=int, default=10, help="Ciclos pré-falta para normalização")
    parser.add_argument("--rx-norm-min-pu", type=float, default=0.5, help="Patamar mínimo pu para normalização")
    parser.add_argument("--rx-print-every", type=int, default=0, help="Status a cada N amostras no receptor")
    parser.add_argument(
        "--rx-extra",
        action="append",
        default=[],
        help="Argumentos extras crus para o receptor (pode repetir, usa shlex).",
    )

    args = parser.parse_args()

    gen_script_path = Path(args.gen_script).resolve()
    rx_script_path = Path(args.rx_script).resolve()
    root_guess = Path(__file__).resolve().parent
    generator_dir = gen_script_path.parent
    resolve_bases = [Path.cwd(), root_guess, generator_dir]

    gen_cfg = _resolve_existing_path(args.gen_cfg, resolve_bases)
    gen_bdat = _resolve_existing_path(args.gen_bdat, resolve_bases)
    if not gen_cfg.exists():
        raise SystemExit(f"CFG não encontrado: {args.gen_cfg}")
    if not gen_bdat.exists():
        raise SystemExit(f"BDAT não encontrado: {args.gen_bdat}")

    if args.gen_receiver_config:
        receiver_cfg = _resolve_existing_path(args.gen_receiver_config, [Path.cwd(), root_guess])
    else:
        receiver_cfg = (Path.cwd() / "config.json").resolve()
    rx_config = _resolve_existing_path(args.rx_config, [Path.cwd(), root_guess]) if args.rx_config else receiver_cfg

    # Saída canônica única do CSV no root do projeto.
    rx_out_name = Path(args.rx_out).name or "adc_capture.csv"
    rx_out_path = (root_guess / rx_out_name).resolve()

    gen_cmd = [
        args.python,
        "-u",
        str(gen_script_path),
        "--cfg",
        str(gen_cfg),
        "--bdat",
        str(gen_bdat),
        "--baud",
        str(args.gen_baud),
        "--ready-timeout",
        str(args.gen_ready_timeout),
        "--receiver-config",
        str(receiver_cfg),
    ]
    if args.gen_port:
        gen_cmd.extend(["--port", str(args.gen_port)])
    if args.gen_out:
        gen_cmd.extend(["--out", str(args.gen_out)])
    if args.gen_export_config_only:
        gen_cmd.append("--export-config-only")
    gen_cmd.extend(_split_extra(args.gen_extra))

    rx_cmd = [
        args.python,
        "-u",
        str(rx_script_path),
        "--config",
        str(rx_config),
        "--port",
        str(args.rx_port),
        "--baud",
        str(args.rx_baud),
        "--out",
        str(rx_out_path),
        "--adc-vref",
        str(args.rx_adc_vref),
        "--sample-rate",
        str(args.rx_sample_rate),
        "--offset-tau",
        str(args.rx_offset_tau),
        "--v-scale",
        str(args.rx_v_scale),
        "--i-scale",
        str(args.rx_i_scale),
        "--zc-hyst",
        str(args.rx_zc_hyst),
        "--f-min",
        str(args.rx_f_min),
        "--f-max",
        str(args.rx_f_max),
        "--prefault-cycles",
        str(args.rx_prefault_cycles),
        "--norm-min-pu",
        str(args.rx_norm_min_pu),
        "--print-every",
        str(args.rx_print_every),
    ]
    if args.rx_outlier_step_v is not None:
        rx_cmd.extend(["--outlier-step-v", str(args.rx_outlier_step_v)])
    if args.rx_outlier_step_i is not None:
        rx_cmd.extend(["--outlier-step-i", str(args.rx_outlier_step_i)])
    if args.rx_normalize_to_comtrade:
        rx_cmd.append("--normalize-to-comtrade")
    rx_cmd.extend(_split_extra(args.rx_extra))

    print("[CTRL] Comando receptor:")
    print("       " + " ".join(shlex.quote(x) for x in rx_cmd))
    print(f"[CTRL] CSV canônico de saída: {rx_out_path}")
    print("[CTRL] Comando gerador:")
    print("       " + " ".join(shlex.quote(x) for x in gen_cmd))

    # Remove arquivo legado para evitar ambiguidade na leitura.
    legacy_rx_csv = (root_guess / "receiver/python/v_1/adc_capture.csv").resolve()
    if legacy_rx_csv != rx_out_path and legacy_rx_csv.exists():
        try:
            legacy_rx_csv.unlink()
            print(f"[CTRL] Removido CSV legado para evitar conflito: {legacy_rx_csv}")
        except Exception as exc:
            print(f"[CTRL] Aviso: não foi possível remover CSV legado {legacy_rx_csv}: {exc}")

    if args.probe_ready:
        # Ordem solicitada: primeiro gerador, depois receptor.
        _probe_esp32_ready("GEN", args.gen_port, args.gen_baud, args.probe_timeout)
        _probe_rx_binary("RX", args.rx_port, args.rx_baud, args.probe_timeout)

    if args.prepare_config_first:
        prep_cmd = list(gen_cmd)
        if "--export-config-only" not in prep_cmd:
            prep_cmd.append("--export-config-only")
        print("[CTRL] Pré-gerando config.json...")
        prep = subprocess.run(prep_cmd, text=True)
        if prep.returncode != 0:
            raise SystemExit(f"Falha ao pré-gerar config do receptor (exit={prep.returncode})")
        if not receiver_cfg.exists():
            raise SystemExit(f"Config do receptor não foi gerado: {receiver_cfg}")

    rx_proc = subprocess.Popen(
        rx_cmd,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    rx_thread = threading.Thread(target=_reader_thread, args=("RX", rx_proc.stdout), daemon=True)
    rx_thread.start()

    print(f"[CTRL] Receptor iniciado (pid={rx_proc.pid}). Aguardando {args.start_delay:.2f}s para iniciar gerador...")
    time.sleep(max(0.0, args.start_delay))

    # Gerador mantém stdin no terminal para aceitar comandos interativos.
    gen_proc = subprocess.Popen(
        gen_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=None if sys.platform.startswith("win") else lambda: signal.signal(signal.SIGINT, signal.SIG_DFL),
    )
    gen_thread = threading.Thread(target=_reader_thread, args=("GEN", gen_proc.stdout), daemon=True)
    gen_thread.start()
    print(f"[CTRL] Gerador iniciado (pid={gen_proc.pid}).")
    print("[CTRL] Comandos do gerador: t, s, f, fr, p, r, q, x (digite e pressione Enter).")

    try:
        while True:
            rx_rc = rx_proc.poll()
            gen_rc = gen_proc.poll()

            if args.stop_on_first_exit and (rx_rc is not None or gen_rc is not None):
                print("[CTRL] stop-on-first-exit ativo.")
                _terminate_process(rx_proc, "receptor")
                _terminate_process(gen_proc, "gerador")
                break

            if rx_rc is not None and gen_rc is not None:
                break

            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[CTRL] Interrupção recebida (Ctrl+C).")
        _terminate_process(rx_proc, "receptor")
        _terminate_process(gen_proc, "gerador")

    rx_thread.join(timeout=1.0)
    gen_thread.join(timeout=1.0)

    rx_rc = rx_proc.poll()
    gen_rc = gen_proc.poll()
    print(f"[CTRL] Finalizado. Exit codes: receptor={rx_rc} gerador={gen_rc}")


if __name__ == "__main__":
    main()

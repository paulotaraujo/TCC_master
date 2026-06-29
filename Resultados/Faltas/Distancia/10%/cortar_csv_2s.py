#!/usr/bin/env python3
"""
Corta CSVs de ensaio mantendo o cabecalho e reorganizando timestamps.

Uso:
    python3 cortar_csv_2s.py

O ponto de corte usa a numeracao fisica do arquivo CSV, contando o cabecalho
como linha 1. Se o usuario informar corte=800 e anteriores=500, o script salva
o cabecalho e as linhas 300..800 do arquivo original.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


DEFAULT_DURATION_S = 2.0


def ask_yes_no(prompt: str) -> bool:
    while True:
        answer = input(prompt).strip().lower()
        if answer in {"s", "sim", "y", "yes"}:
            return True
        if answer in {"n", "nao", "não", "no"}:
            return False
        print("Responda com 's' para sim ou 'n' para nao.")


def ask_int(prompt: str, min_value: int, max_value: int) -> int:
    while True:
        raw = input(prompt).strip()
        try:
            value = int(raw)
        except ValueError:
            print("Digite um numero inteiro.")
            continue
        if min_value <= value <= max_value:
            return value
        print(f"Digite um valor entre {min_value} e {max_value}.")


def ask_float_default(prompt: str, default: float, min_value: float) -> float:
    while True:
        raw = input(prompt).strip()
        if not raw:
            return default
        try:
            value = float(raw.replace(",", "."))
        except ValueError:
            print("Digite um numero valido.")
            continue
        if value > min_value:
            return value
        print(f"Digite um valor maior que {min_value}.")


def ask_existing_csv(prompt: str) -> Path:
    while True:
        raw = input(prompt).strip().strip('"').strip("'")
        if not raw:
            print("Informe o caminho do arquivo CSV.")
            continue
        csv_path = Path(raw).expanduser()
        if not csv_path.is_absolute():
            csv_path = Path.cwd() / csv_path
        csv_path = csv_path.resolve()
        if not csv_path.exists():
            print(f"Arquivo nao encontrado: {csv_path}")
            continue
        if csv_path.suffix.lower() != ".csv":
            print(f"O arquivo precisa ser .csv: {csv_path}")
            continue
        return csv_path


def ask_output_path(prompt: str, default: Path) -> Path:
    raw = input(prompt).strip().strip('"').strip("'")
    if not raw:
        return default
    out_path = Path(raw).expanduser()
    if not out_path.is_absolute():
        out_path = Path.cwd() / out_path
    if out_path.suffix.lower() != ".csv":
        out_path = out_path.with_suffix(".csv")
    return out_path.resolve()


def default_output_path(csv_path: Path) -> Path:
    return csv_path.with_name(f"{csv_path.stem}_cortado_2s{csv_path.suffix}")


def count_lines(csv_path: Path) -> int:
    with csv_path.open("r", encoding="utf-8", newline="") as file:
        return sum(1 for _ in file)


def normalize_timestamps(header: list[str], rows: list[list[str]], duration_s: float) -> None:
    n = len(rows)
    if n <= 0:
        return

    def col_index(name: str) -> int | None:
        try:
            return header.index(name)
        except ValueError:
            return None

    idx_host_t_s = col_index("host_t_s")
    idx_dev_t_s = col_index("dev_t_s")
    idx_dev_t_us = col_index("dev_t_us")

    if n == 1:
        times_s = [0.0]
    else:
        dt = duration_s / float(n - 1)
        times_s = [i * dt for i in range(n)]

    for row, t_s in zip(rows, times_s):
        if idx_host_t_s is not None and idx_host_t_s < len(row):
            row[idx_host_t_s] = f"{t_s:.9f}"
        if idx_dev_t_s is not None and idx_dev_t_s < len(row):
            row[idx_dev_t_s] = f"{t_s:.9f}"
        if idx_dev_t_us is not None and idx_dev_t_us < len(row):
            row[idx_dev_t_us] = str(int(round(t_s * 1_000_000.0)))


def cut_csv(csv_path: Path, out_path: Path, start_line: int, end_line: int, duration_s: float) -> int:
    with csv_path.open("r", encoding="utf-8", newline="") as src:
        reader = csv.reader(src)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise SystemExit(f"CSV vazio: {csv_path}") from exc

        rows: list[list[str]] = []
        for physical_line, row in enumerate(reader, start=2):
            if start_line <= physical_line <= end_line:
                rows.append(row)
            elif physical_line > end_line:
                break

    normalize_timestamps(header, rows, duration_s)

    with out_path.open("w", encoding="utf-8", newline="") as dst:
        writer = csv.writer(dst)
        writer.writerow(header)
        writer.writerows(rows)

    return len(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Corta um CSV de ensaio e reorganiza host_t_s/dev_t_s/dev_t_us para 0..2s."
    )
    parser.add_argument("--csv", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--out", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--duration", type=float, default=None, help=argparse.SUPPRESS)
    args = parser.parse_args()

    print("=== Corte de CSV - janela de 2 segundos ===")
    if args.csv:
        csv_path = Path(args.csv).expanduser().resolve()
        if not csv_path.exists():
            raise SystemExit(f"Arquivo nao encontrado: {csv_path}")
        if csv_path.suffix.lower() != ".csv":
            raise SystemExit(f"O arquivo precisa ser .csv: {csv_path}")
    else:
        csv_path = ask_existing_csv("Informe o arquivo CSV de entrada: ")

    if args.duration is not None:
        if args.duration <= 0.0:
            raise SystemExit("--duration precisa ser maior que zero.")
        duration_s = args.duration
    else:
        duration_s = ask_float_default(
            "Duracao da janela salva em segundos [2.0]: ",
            default=DEFAULT_DURATION_S,
            min_value=0.0,
        )

    total_lines = count_lines(csv_path)
    if total_lines <= 1:
        raise SystemExit("CSV sem amostras de dados. Apenas cabecalho encontrado.")

    print(f"Arquivo: {csv_path}")
    print(f"Total de linhas no CSV: {total_lines}")
    print(f"Linhas de dados: {total_lines - 1}")

    if not ask_yes_no("Gostaria de cortar o arquivo? [s/n]: "):
        print("Nenhum corte realizado.")
        return

    cut_line = ask_int(
        "Qual linha deve ser a ultima amostra util? ",
        min_value=2,
        max_value=total_lines,
    )
    previous_lines = ask_int(
        "Quantas linhas anteriores ao corte devem ser mantidas? ",
        min_value=0,
        max_value=cut_line - 2,
    )

    start_line = max(2, cut_line - previous_lines)
    default_out = default_output_path(csv_path)
    if args.out:
        out_path = Path(args.out).expanduser().resolve()
    else:
        out_path = ask_output_path(
            f"Arquivo de saida [{default_out.name}]: ",
            default=default_out,
        )

    saved_rows = cut_csv(csv_path, out_path, start_line, cut_line, duration_s)

    print("Corte concluido.")
    print(f"Intervalo salvo do arquivo original: linhas {start_line}..{cut_line}")
    print(f"Amostras salvas: {saved_rows}")
    print(f"Timestamps reorganizados: 0..{duration_s:g} s")
    print(f"Arquivo de saida: {out_path}")


if __name__ == "__main__":
    main()

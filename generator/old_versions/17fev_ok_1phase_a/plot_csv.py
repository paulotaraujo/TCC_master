#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import matplotlib.pyplot as plt


def sniff_dialect(path: Path) -> csv.Dialect:
    sample = path.read_text(errors="ignore")[:4096]
    sniffer = csv.Sniffer()
    try:
        return sniffer.sniff(sample, delimiters=[",", ";", "\t"])
    except Exception:
        d = csv.excel
        d.delimiter = ","
        return d


def to_int(x: Optional[str], default: int = 0) -> int:
    if x is None:
        return default
    s = str(x).strip()
    if not s:
        return default
    return int(float(s.replace(",", ".")))


def to_float(x: Optional[str], default: float = float("nan")) -> float:
    if x is None:
        return default
    s = str(x).strip()
    if not s:
        return default
    return float(s.replace(",", "."))


def read_rows(path: Path) -> Tuple[List[float], Dict[str, List[float]]]:
    """
    Lê CSV com header (recomendado). Tolerante a campos vazios.
    Prioriza tempo por t_us (se existir) senão usa ts_comtrade.
    """
    dialect = sniff_dialect(path)

    with path.open("r", newline="", errors="ignore") as f:
        reader = csv.reader(f, dialect)
        first = next(reader, None)
        if first is None:
            raise RuntimeError("CSV vazio.")

        # Detecta header
        has_header = True
        try:
            _ = to_int(first[0])
            has_header = False
        except Exception:
            has_header = True

        rows: List[Dict[str, str]] = []

        if not has_header:
            # sem header: tenta mapear por formatos comuns
            f.seek(0)
            reader = csv.reader(f, dialect)
            first_data = next(reader, None)
            if first_data is None:
                raise RuntimeError("CSV vazio.")
            ncols = len(first_data)

            # formatos suportados (replay geralmente 9/10/12)
            # 9 : sample,ts_comtrade,dt_us,engA,engB,adcA_V,adcB_V,dacA_code,dacB_code
            # 10: sample,ts_comtrade,dt_us,t_us,engA,engB,voutA,voutB,dacA_code,dacB_code
            # 12: ... + adcA_V,adcB_V
            if ncols == 9:
                fieldnames = ["sample","ts_comtrade","dt_us","engA","engB","adcA_V","adcB_V","dacA_code","dacB_code"]
            elif ncols == 10:
                fieldnames = ["sample","ts_comtrade","dt_us","t_us","engA","engB","voutA","voutB","dacA_code","dacB_code"]
            elif ncols == 12:
                fieldnames = ["sample","ts_comtrade","dt_us","t_us","engA","engB","voutA","voutB","dacA_code","dacB_code","adcA_V","adcB_V"]
            else:
                raise RuntimeError(f"CSV sem header com {ncols} colunas não suportado.")

            f.seek(0)
            reader = csv.reader(f, dialect)
            for r in reader:
                if len(r) != ncols:
                    continue
                rows.append(dict(zip(fieldnames, r)))

        else:
            header = [h.strip() for h in first]
            if header and header[0].startswith("\ufeff"):
                header[0] = header[0].replace("\ufeff", "")

            dict_reader = csv.DictReader(f, fieldnames=header, dialect=dialect)
            for r in dict_reader:
                if r:
                    rows.append(r)

        # remove linhas totalmente vazias / inválidas
        cleaned: List[Dict[str, str]] = []
        for r in rows:
            if not r:
                continue
            # considera válida se tem ao menos sample ou ts_comtrade
            if (r.get("sample") is None or str(r.get("sample")).strip() == "") and \
               (r.get("ts_comtrade") is None or str(r.get("ts_comtrade")).strip() == ""):
                continue
            cleaned.append(r)

        rows = cleaned
        if not rows:
            raise RuntimeError("Nenhuma linha de dados foi lida.")

        # Decide eixo do tempo: t_us (ideal) > ts_comtrade
        use_t_us = "t_us" in rows[0]

        t: List[float] = []
        data: Dict[str, List[float]] = {}

        def push(k: str, v: float) -> None:
            data.setdefault(k, []).append(v)

        # base temporal
        if use_t_us:
            t0 = to_int(rows[0].get("t_us", "0"))
            for r in rows:
                tu = to_int(r.get("t_us", None), default=t0)
                t.append((tu - t0) * 1e-6)
        else:
            ts0 = to_int(rows[0].get("ts_comtrade", "0"))
            for r in rows:
                ts = to_int(r.get("ts_comtrade", None), default=ts0)
                t.append((ts - ts0) * 1e-6)

        # Colunas possíveis (tolerante a NaN)
        for r in rows:
            if "engA" in r:   push("engA",   to_float(r.get("engA")))
            if "engB" in r:   push("engB",   to_float(r.get("engB")))
            if "voutA" in r:  push("voutA",  to_float(r.get("voutA")))
            if "voutB" in r:  push("voutB",  to_float(r.get("voutB")))
            if "adcA_V" in r: push("adcA_V", to_float(r.get("adcA_V")))
            if "adcB_V" in r: push("adcB_V", to_float(r.get("adcB_V")))

        print(f"✅ Linhas: {len(t)} | duração ~ {t[-1]:.6f} s")
        print(f"📌 Colunas disponíveis: {list(data.keys())}")
        print("🕒 Tempo usando:", "t_us (reprodução)" if use_t_us else "ts_comtrade")

        return t, data


def mask_nan(x: List[float], y: List[float]) -> Tuple[List[float], List[float]]:
    """Remove pares onde y é NaN."""
    xs, ys = [], []
    for xi, yi in zip(x, y):
        if yi != yi:  # NaN
            continue
        xs.append(xi)
        ys.append(yi)
    return xs, ys


def plot_two_figures(t: List[float], data: Dict[str, List[float]], mode: str) -> None:
    """
    mode:
      - "vout": plota voutA/voutB (volts enviados ao DAC)
      - "eng" : plota engA/engB
      - "adc" : plota adcA_V/adcB_V
    """
    if mode == "vout":
        yA = data["voutA"]
        yB = data["voutB"]
        ylabA = "Tensão (voutA, V)"
        ylabB = "Corrente (voutB, V)"
        titleA = "Sinal reproduzido no DAC (Tensão) - voutA"
        titleB = "Sinal reproduzido no DAC (Corrente) - voutB"
    elif mode == "eng":
        yA = data["engA"]
        yB = data["engB"]
        ylabA = "Tensão (engA)"
        ylabB = "Corrente (engB)"
        titleA = "Sinal de Tensão (engenharia) - engA"
        titleB = "Sinal de Corrente (engenharia) - engB"
    else:
        yA = data["adcA_V"]
        yB = data["adcB_V"]
        ylabA = "Tensão (adcA_V, V)"
        ylabB = "Corrente (adcB_V, V)"
        titleA = "Saída medida no ADC (Tensão) - adcA_V"
        titleB = "Saída medida no ADC (Corrente) - adcB_V"

    txA, yyA = mask_nan(t, yA)
    txB, yyB = mask_nan(t, yB)

    plt.figure()
    plt.plot(txA, yyA)
    plt.title(titleA)
    plt.xlabel("Tempo (s)")
    plt.ylabel(ylabA)
    plt.grid(True)

    plt.figure()
    plt.plot(txB, yyB)
    plt.title(titleB)
    plt.xlabel("Tempo (s)")
    plt.ylabel(ylabB)
    plt.grid(True)

    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Plot CSV COMTRADE (replay) - 2 gráficos separados (tensão/corrente)."
    )
    ap.add_argument("csv", type=str, help="CSV (ex: output.csv)")
    ap.add_argument("--mode", choices=["auto", "vout", "eng", "adc"], default="auto",
                    help="auto=prioriza vout -> eng -> adc")
    args = ap.parse_args()

    path = Path(args.csv).expanduser().resolve()
    if not path.exists():
        raise SystemExit(f"Arquivo não encontrado: {path}")

    t, data = read_rows(path)

    if args.mode == "auto":
        if "voutA" in data and "voutB" in data:
            mode = "vout"
        elif "engA" in data and "engB" in data:
            mode = "eng"
        elif "adcA_V" in data and "adcB_V" in data:
            mode = "adc"
        else:
            raise SystemExit(
                "Não encontrei pares de colunas válidos "
                "(voutA/voutB, engA/engB ou adcA_V/adcB_V)."
            )
    else:
        mode = args.mode

    print(f"📊 Modo de plot: {mode}")

    if mode == "vout" and not ("voutA" in data and "voutB" in data):
        raise SystemExit("CSV não contém voutA/voutB.")
    if mode == "eng" and not ("engA" in data and "engB" in data):
        raise SystemExit("CSV não contém engA/engB.")
    if mode == "adc" and not ("adcA_V" in data and "adcB_V" in data):
        raise SystemExit("CSV não contém adcA_V/adcB_V.")

    plot_two_figures(t, data, mode)


if __name__ == "__main__":
    main()


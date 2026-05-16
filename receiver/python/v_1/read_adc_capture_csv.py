import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def estimate_adc_per_volt(adc_vals: np.ndarray, volt_vals: np.ndarray) -> float | None:
    mask = np.isfinite(adc_vals) & np.isfinite(volt_vals) & (np.abs(volt_vals) > 1e-9)
    if not np.any(mask):
        return None
    ratios = adc_vals[mask] / volt_vals[mask]
    ratios = ratios[np.isfinite(ratios)]
    if ratios.size == 0:
        return None
    return float(np.median(ratios))


def add_background_scale(ax, y_vals: np.ndarray, levels: int) -> None:
    if levels < 2:
        return
    y = np.asarray(y_vals, dtype=float)
    y = y[np.isfinite(y)]
    if y.size < 2:
        return

    # Faixa robusta para evitar que outliers estiquem a escala visual.
    y_min = float(np.percentile(y, 2.0))
    y_max = float(np.percentile(y, 98.0))
    if not np.isfinite(y_min) or not np.isfinite(y_max):
        return
    if y_max <= y_min:
        span = max(1e-9, abs(y_max))
        y_min -= 0.05 * span
        y_max += 0.05 * span

    ref_vals = np.linspace(y_min, y_max, int(levels))
    for ref in ref_vals:
        ax.axhline(ref, color="#b0b0b0", lw=0.8, ls=":", alpha=0.45, zorder=0)


def get_amplitude_scaled(df: pd.DataFrame) -> tuple[np.ndarray | None, np.ndarray | None]:
    if "v_inst" not in df.columns or "i_inst" not in df.columns:
        return None, None

    v = df["v_inst"].to_numpy(dtype=float)
    i = df["i_inst"].to_numpy(dtype=float)

    # Alinha com rms_scaled: aplica os ganhos de normalização quando disponíveis.
    if "norm_gain_v" in df.columns:
        v = v * df["norm_gain_v"].to_numpy(dtype=float)
    if "norm_gain_i" in df.columns:
        i = i * df["norm_gain_i"].to_numpy(dtype=float)

    return v, i


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Leitura rápida do adc_capture.csv (preview, RMS e plot)."
    )
    parser.add_argument("--csv", default="adc_capture.csv", help="Caminho do CSV")
    parser.add_argument("--head", type=int, default=10, help="Quantidade de linhas iniciais")
    parser.add_argument("--tail", type=int, default=10, help="Quantidade de linhas finais")
    parser.add_argument("--plot", action="store_true", help="Plotar séries temporais")
    parser.add_argument(
        "--plot-mode",
        choices=["adc", "adc_no_offset", "volts", "volts_no_offset", "amplitude_scaled", "rms_real", "rms_scaled"],
        default="rms_scaled",
        help=(
            "Modo de plot: adc(0..4095), adc_no_offset, volts(0..3.3V), "
            "volts_no_offset, amplitude_scaled (instante escalonado), "
            "rms_real(bruto) e rms_scaled(normalizado/escalonado)."
        ),
    )
    parser.add_argument(
        "--plot-all",
        action="store_true",
        help="No plot, usar todas as linhas (sem filtro de qualidade).",
    )
    parser.add_argument(
        "--scatter",
        action="store_true",
        help="Plotar RMS em pontos (sem ligar por linhas), evitando rampas visuais artificiais.",
    )
    parser.add_argument(
        "--break-gap-ms",
        type=float,
        default=100.0,
        help=(
            "No modo linha, quebra a curva quando o intervalo de tempo entre amostras "
            "ultrapassar este valor (ms)."
        ),
    )
    parser.add_argument(
        "--plot-bg-scale",
        type=int,
        default=0,
        help=(
            "Niveis de escala de fundo no eixo Y (linhas horizontais calculadas "
            "pela faixa robusta do sinal). 0 desativa."
        ),
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"CSV não encontrado: {csv_path}")

    df = pd.read_csv(csv_path)
    print(f"Arquivo: {csv_path}")
    print(f"Linhas: {len(df)}")
    print(f"Colunas: {list(df.columns)}")

    print("\n--- HEAD ---")
    print(df.head(args.head).to_string(index=False))

    print("\n--- TAIL ---")
    print(df.tail(args.tail).to_string(index=False))

    rms_cols = [c for c in ("v_rms_raw", "i_rms_raw", "v_rms", "i_rms") if c in df.columns]
    if rms_cols:
        print("\n--- ESTATÍSTICAS RMS ---")
        print(df[rms_cols].describe().to_string())
    else:
        print("\nCSV não possui colunas v_rms/i_rms.")

    if args.plot:
        plot_df = df
        base_mode = args.plot_mode
        if base_mode.endswith("_no_offset"):
            base_mode = base_mode.replace("_no_offset", "")
        rms_mode = base_mode in {"rms_real", "rms_scaled"}
        amp_mode = base_mode == "amplitude_scaled"
        if rms_mode:
            if "cycle_update" in plot_df.columns:
                plot_df = plot_df[plot_df["cycle_update"] == 1]
            if not args.plot_all:
                if "rms_valid" in plot_df.columns:
                    plot_df = plot_df[plot_df["rms_valid"] == 1]
                if base_mode == "rms_scaled" and "norm_ready" in plot_df.columns:
                    plot_df = plot_df[plot_df["norm_ready"] == 1]
                print(f"\nPlot RMS com filtro de qualidade: {len(plot_df)} linhas.")
            else:
                print(f"\nPlot RMS integral (updates): {len(plot_df)} linhas.")
        else:
            print(f"\nPlot de amostras integrais: {len(plot_df)} linhas.")
            if amp_mode and (not args.plot_all) and ("quality_flags" in plot_df.columns):
                # Remove pontos com outlier/saturação/checksum/desync para estabilizar amplitude.
                bad_mask = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4)
                qf = plot_df["quality_flags"].to_numpy(dtype=np.int64)
                keep = (qf & bad_mask) == 0
                plot_df = plot_df[keep]
                print(f"Plot amplitude_scaled com filtro de qualidade: {len(plot_df)} linhas.")

        # Tempo global para padronizar escala em todos os gráficos/modos.
        # Preferência: host_t_s (tempo da captura no PC).
        if "host_t_s" in df.columns:
            global_t_max = float(np.nanmax(df["host_t_s"].to_numpy()))
        elif "dev_t_s" in df.columns:
            global_t_max = float(np.nanmax(df["dev_t_s"].to_numpy()))
        else:
            global_t_max = float(len(df) - 1)
        if not np.isfinite(global_t_max) or global_t_max < 0.0:
            global_t_max = 0.0

        if "host_t_s" in plot_df.columns:
            t = plot_df["host_t_s"].to_numpy()
            x_label = "host_t_s (s)"
        elif "dev_t_s" in plot_df.columns:
            t = plot_df["dev_t_s"].to_numpy()
            x_label = "dev_t_s (s)"
        else:
            t = plot_df.index.to_numpy()
            x_label = "amostra"

        def break_large_gaps(x: np.ndarray, y: np.ndarray, gap_s: float) -> np.ndarray:
            yb = y.astype(float).copy()
            if len(x) > 1:
                dt = np.diff(x)
                cut_idx = np.where(dt > gap_s)[0]
                for idx in cut_idx:
                    yb[idx + 1] = np.nan
            return yb

        fig, (ax_v, ax_i) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

        gap_s = max(0.0, args.break_gap_ms) / 1000.0

        if base_mode == "adc":
            v_col, i_col = "adc_34", "adc_35"
            v_title, i_title = "Tensão ADC (GPIO34) ao longo do tempo", "Corrente ADC (GPIO35) ao longo do tempo"
            v_label, i_label = "ADC V (0..4095)", "ADC I (0..4095)"
        elif base_mode == "volts":
            v_col, i_col = "v_adc_34", "v_adc_35"
            v_title, i_title = "Tensão em volts no ADC (GPIO34)", "Corrente em volts no ADC (GPIO35)"
            v_label, i_label = "V ADC34 (V)", "V ADC35 (V)"
        elif base_mode == "amplitude_scaled":
            v_col, i_col = "__amp_scaled_v__", "__amp_scaled_i__"
            v_title, i_title = "Tensão instantânea escalonada", "Corrente instantânea escalonada"
            v_label, i_label = "V inst escalonada", "I inst escalonada"
        elif base_mode == "rms_real":
            v_col, i_col = "v_rms_raw", "i_rms_raw"
            v_title, i_title = "Tensão RMS real (bruta)", "Corrente RMS real (bruta)"
            v_label, i_label = "V RMS real", "I RMS real"
        else:
            v_col, i_col = "v_rms", "i_rms"
            v_title, i_title = "Tensão RMS escalonada", "Corrente RMS escalonada"
            v_label, i_label = "V RMS escalonada", "I RMS escalonada"

        yv_amp = yi_amp = None
        if base_mode == "amplitude_scaled":
            yv_amp, yi_amp = get_amplitude_scaled(plot_df)

        has_v = (base_mode == "amplitude_scaled" and yv_amp is not None) or (v_col in plot_df.columns)
        if has_v:
            yv = yv_amp if base_mode == "amplitude_scaled" else plot_df[v_col].to_numpy()
            if args.scatter:
                ax_v.scatter(t, yv, color="#1f77b4", s=8, label=v_col)
            else:
                ax_v.plot(t, break_large_gaps(t, yv, gap_s), color="#1f77b4", label=v_col)

            # Modos dedicados sem offset.
            if args.plot_mode == "volts_no_offset" and "v_offset_est" in plot_df.columns:
                yv_no_offset = yv - plot_df["v_offset_est"].to_numpy()
                yv = yv_no_offset
                ax_v.clear()
                if args.scatter:
                    ax_v.scatter(t, yv, color="#ff7f0e", s=8, label=f"{v_col}_no_offset")
                else:
                    ax_v.plot(
                        t,
                        break_large_gaps(t, yv, gap_s),
                        color="#ff7f0e",
                        ls="-",
                        label=f"{v_col}_no_offset",
                    )
                v_title = "Tensão em volts no ADC (GPIO34) sem offset"
                v_label = "V ADC34 sem offset (V)"
            elif (
                args.plot_mode == "adc_no_offset"
                and "v_offset_est" in plot_df.columns
                and "v_adc_34" in plot_df.columns
            ):
                adc_per_v = estimate_adc_per_volt(
                    plot_df["adc_34"].to_numpy(dtype=float),
                    plot_df["v_adc_34"].to_numpy(dtype=float),
                )
                if adc_per_v is not None:
                    yv = yv - (plot_df["v_offset_est"].to_numpy() * adc_per_v)
                    ax_v.clear()
                    if args.scatter:
                        ax_v.scatter(t, yv, color="#ff7f0e", s=8, label=f"{v_col}_no_offset")
                    else:
                        ax_v.plot(
                            t,
                            break_large_gaps(t, yv, gap_s),
                            color="#ff7f0e",
                            ls="-",
                            label=f"{v_col}_no_offset",
                        )
                    v_title = "Tensão ADC (GPIO34) sem offset"
                    v_label = "ADC V sem offset"
            ax_v.set_ylabel(v_label)
            ax_v.set_title(v_title)
            ax_v.grid(True, alpha=0.3)
            add_background_scale(ax_v, yv, args.plot_bg_scale)
            ax_v.legend()
        else:
            ax_v.set_title(f"Coluna indisponível no CSV: {v_col}")

        has_i = (base_mode == "amplitude_scaled" and yi_amp is not None) or (i_col in plot_df.columns)
        if has_i:
            yi = yi_amp if base_mode == "amplitude_scaled" else plot_df[i_col].to_numpy()
            if args.scatter:
                ax_i.scatter(t, yi, color="#d62728", s=8, label=i_col)
            else:
                ax_i.plot(t, break_large_gaps(t, yi, gap_s), color="#d62728", label=i_col)

            # Modos dedicados sem offset.
            if args.plot_mode == "volts_no_offset" and "i_offset_est" in plot_df.columns:
                yi_no_offset = yi - plot_df["i_offset_est"].to_numpy()
                yi = yi_no_offset
                ax_i.clear()
                if args.scatter:
                    ax_i.scatter(t, yi, color="#2ca02c", s=8, label=f"{i_col}_no_offset")
                else:
                    ax_i.plot(
                        t,
                        break_large_gaps(t, yi, gap_s),
                        color="#2ca02c",
                        ls="-",
                        label=f"{i_col}_no_offset",
                    )
                i_title = "Corrente em volts no ADC (GPIO35) sem offset"
                i_label = "V ADC35 sem offset (V)"
            elif (
                args.plot_mode == "adc_no_offset"
                and "i_offset_est" in plot_df.columns
                and "v_adc_35" in plot_df.columns
            ):
                adc_per_v = estimate_adc_per_volt(
                    plot_df["adc_35"].to_numpy(dtype=float),
                    plot_df["v_adc_35"].to_numpy(dtype=float),
                )
                if adc_per_v is not None:
                    yi = yi - (plot_df["i_offset_est"].to_numpy() * adc_per_v)
                    ax_i.clear()
                    if args.scatter:
                        ax_i.scatter(t, yi, color="#2ca02c", s=8, label=f"{i_col}_no_offset")
                    else:
                        ax_i.plot(
                            t,
                            break_large_gaps(t, yi, gap_s),
                            color="#2ca02c",
                            ls="-",
                            label=f"{i_col}_no_offset",
                        )
                    i_title = "Corrente ADC (GPIO35) sem offset"
                    i_label = "ADC I sem offset"
            ax_i.set_ylabel(i_label)
            ax_i.set_title(i_title)
            ax_i.grid(True, alpha=0.3)
            add_background_scale(ax_i, yi, args.plot_bg_scale)
            ax_i.legend()
        else:
            ax_i.set_title(f"Coluna indisponível no CSV: {i_col}")

        ax_v.set_xlim(0.0, global_t_max)
        ax_i.set_xlim(0.0, global_t_max)

        ax_i.set_xlabel(x_label)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()

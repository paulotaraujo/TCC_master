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
        choices=[
            "adc",
            "adc_no_offset",
            "volts",
            "volts_no_offset",
            "amplitude_scaled",
            "rms_real",
            "rms_scaled",
            "fourier_raw",
            "fourier_scaled",
            "fourier_angle",
            "fourier_phase",
            "over_current",
            "over_current_timer",
            "distance",
            "voltage",
        ],
        default="rms_scaled",
        help=(
            "Modo de plot: adc(0..4095), adc_no_offset, volts(0..3.3V), "
            "volts_no_offset, amplitude_scaled (instante escalonado), "
            "rms_real(bruto), rms_scaled(normalizado/escalonado), "
            "fourier_raw/fourier_scaled (fundamental RMS), fourier_angle, "
            "fourier_phase, over_current, over_current_timer, distance e voltage."
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

    rms_cols = [
        c
        for c in (
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
            "oc_51_timer_s",
            "oc_51_active",
            "oc_50_trip",
            "oc_51_trip",
            "oc_trip",
            "dist_z_mag_ohm",
            "dist_z_angle_deg",
            "dist_r_ohm",
            "dist_x_ohm",
            "dist_fault_pct",
            "dist_fault_ohm",
            "dist_z1_trip",
            "dist_z2_trip",
            "dist_trip",
            "uv_timer_s",
            "ov_timer_s",
            "uv_trip",
            "ov_trip",
            "voltage_trip",
        )
        if c in df.columns
    ]
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
        rms_mode = base_mode in {
            "rms_real",
            "rms_scaled",
            "fourier_raw",
            "fourier_scaled",
            "fourier_angle",
            "fourier_phase",
            "over_current",
            "over_current_timer",
            "distance",
            "voltage",
        }
        amp_mode = base_mode == "amplitude_scaled"
        if rms_mode:
            if "cycle_update" in plot_df.columns:
                plot_df = plot_df[plot_df["cycle_update"] == 1]
            if not args.plot_all:
                if (
                    (
                        base_mode.startswith("fourier")
                        or base_mode.startswith("over_current")
                        or base_mode == "distance"
                        or base_mode == "voltage"
                    )
                    and "fourier_valid" in plot_df.columns
                ):
                    plot_df = plot_df[plot_df["fourier_valid"] == 1]
                elif "rms_valid" in plot_df.columns:
                    plot_df = plot_df[plot_df["rms_valid"] == 1]
                if base_mode in {"rms_scaled", "fourier_scaled"} and "norm_ready" in plot_df.columns:
                    plot_df = plot_df[plot_df["norm_ready"] == 1]
                print(f"\nPlot fasorial/RMS com filtro de qualidade: {len(plot_df)} linhas.")
            else:
                print(f"\nPlot fasorial/RMS integral (updates): {len(plot_df)} linhas.")
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

        if base_mode == "over_current":
            required_cols = ["v1_mag", "i1_mag", "oc_50_trip", "oc_51_trip"]
            missing_cols = [col for col in required_cols if col not in plot_df.columns]
            if missing_cols:
                raise SystemExit(f"CSV sem colunas para plot over_current: {missing_cols}")

            plt.close(fig)
            fig, (ax_v1, ax_i1, ax_trip) = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

            v1 = plot_df["v1_mag"].to_numpy(dtype=float)
            i1 = plot_df["i1_mag"].to_numpy(dtype=float)
            oc50 = plot_df["oc_50_trip"].to_numpy(dtype=float)
            oc51 = plot_df["oc_51_trip"].to_numpy(dtype=float)

            ax_v1.plot(t, break_large_gaps(t, v1, gap_s), color="#1f77b4", label="V1 Fourier")
            ax_i1.plot(t, break_large_gaps(t, i1, gap_s), color="#d62728", label="I1 Fourier")
            ax_trip.step(t, oc50, where="post", color="#ff7f0e", lw=1.6, label="Trip 50")
            ax_trip.step(t, oc51, where="post", color="#9467bd", lw=1.6, label="Trip 51")

            if "oc_50_pickup" in plot_df.columns:
                pickup_50 = plot_df["oc_50_pickup"].to_numpy(dtype=float)
                ax_i1.plot(t, break_large_gaps(t, pickup_50, gap_s), color="#ff7f0e", ls="--", alpha=0.65, label="Pickup 50")
            if "oc_51_pickup" in plot_df.columns:
                pickup_51 = plot_df["oc_51_pickup"].to_numpy(dtype=float)
                ax_i1.plot(t, break_large_gaps(t, pickup_51, gap_s), color="#9467bd", ls="--", alpha=0.65, label="Pickup 51")

            ax_v1.set_title("Proteção de sobrecorrente com fasores de Fourier")
            ax_v1.set_ylabel("V1 RMS")
            ax_i1.set_title("Corrente fundamental e pickups")
            ax_i1.set_ylabel("I1 RMS")
            ax_trip.set_title("Atuação das funções 50/51")
            ax_trip.set_ylabel("Trip")
            ax_trip.set_xlabel(x_label)
            ax_trip.set_ylim(-0.05, 1.15)
            ax_trip.set_yticks([0, 1])

            for ax in (ax_v1, ax_i1, ax_trip):
                ax.grid(True, alpha=0.3)
                ax.set_xlim(0.0, global_t_max)
                ax.legend(loc="upper right")

            plt.tight_layout()
            plt.show()
            return

        if base_mode == "voltage":
            required_cols = ["v1_mag", "uv_trip", "ov_trip"]
            missing_cols = [col for col in required_cols if col not in plot_df.columns]
            if missing_cols:
                raise SystemExit(f"CSV sem colunas para plot voltage: {missing_cols}")

            plt.close(fig)
            fig, (ax_v1, ax_timer, ax_trip) = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

            v1 = plot_df["v1_mag"].to_numpy(dtype=float)
            uv_trip_arr = plot_df["uv_trip"].to_numpy(dtype=float)
            ov_trip_arr = plot_df["ov_trip"].to_numpy(dtype=float)

            ax_v1.plot(t, break_large_gaps(t, v1, gap_s), color="#1f77b4", label="V1 Fourier")
            if "uv_pickup" in plot_df.columns:
                uv_pickup_arr = plot_df["uv_pickup"].to_numpy(dtype=float)
                ax_v1.plot(
                    t,
                    break_large_gaps(t, uv_pickup_arr, gap_s),
                    color="#ff7f0e",
                    ls="--",
                    alpha=0.75,
                    label="Pickup 27",
                )
            if "ov_pickup" in plot_df.columns:
                ov_pickup_arr = plot_df["ov_pickup"].to_numpy(dtype=float)
                ax_v1.plot(
                    t,
                    break_large_gaps(t, ov_pickup_arr, gap_s),
                    color="#9467bd",
                    ls="--",
                    alpha=0.75,
                    label="Pickup 59",
                )

            if "uv_timer_s" in plot_df.columns:
                ax_timer.plot(
                    t,
                    break_large_gaps(t, plot_df["uv_timer_s"].to_numpy(dtype=float), gap_s),
                    color="#ff7f0e",
                    label="Timer 27",
                )
            if "ov_timer_s" in plot_df.columns:
                ax_timer.plot(
                    t,
                    break_large_gaps(t, plot_df["ov_timer_s"].to_numpy(dtype=float), gap_s),
                    color="#9467bd",
                    label="Timer 59",
                )

            ax_trip.step(t, uv_trip_arr, where="post", color="#ff7f0e", lw=1.6, label="Trip 27")
            ax_trip.step(t, ov_trip_arr, where="post", color="#9467bd", lw=1.6, label="Trip 59")

            ax_v1.set_title("Proteção de tensão 27/59")
            ax_v1.set_ylabel("V1 RMS")
            ax_timer.set_title("Temporizadores 27/59")
            ax_timer.set_ylabel("tempo (s)")
            ax_trip.set_title("Atuação das funções 27/59")
            ax_trip.set_ylabel("Trip")
            ax_trip.set_xlabel(x_label)
            ax_trip.set_ylim(-0.05, 1.15)
            ax_trip.set_yticks([0, 1])

            for ax in (ax_v1, ax_timer, ax_trip):
                ax.grid(True, alpha=0.3)
                ax.set_xlim(0.0, global_t_max)
                ax.legend(loc="upper right")

            plt.tight_layout()
            plt.show()
            return

        if base_mode == "distance":
            required_cols = ["dist_z_mag_ohm", "dist_z1_ohm", "dist_z2_ohm", "dist_z1_trip", "dist_z2_trip"]
            missing_cols = [col for col in required_cols if col not in plot_df.columns]
            if missing_cols:
                raise SystemExit(f"CSV sem colunas para plot distance: {missing_cols}")

            plt.close(fig)
            fig = plt.figure(figsize=(11, 8))
            grid = fig.add_gridspec(3, 1)
            ax_rx = fig.add_subplot(grid[0, 0])
            ax_z = fig.add_subplot(grid[1, 0])
            ax_trip = fig.add_subplot(grid[2, 0], sharex=ax_z)

            z_mag = plot_df["dist_z_mag_ohm"].to_numpy(dtype=float)
            z1 = plot_df["dist_z1_ohm"].to_numpy(dtype=float)
            z2 = plot_df["dist_z2_ohm"].to_numpy(dtype=float)
            z1_trip = plot_df["dist_z1_trip"].to_numpy(dtype=float)
            z2_trip = plot_df["dist_z2_trip"].to_numpy(dtype=float)

            ax_z.plot(t, break_large_gaps(t, z_mag, gap_s), color="#1f77b4", label="|Z| aparente")
            ax_z.plot(t, break_large_gaps(t, z1, gap_s), color="#ff7f0e", ls="--", alpha=0.75, label="Zona 1")
            ax_z.plot(t, break_large_gaps(t, z2, gap_s), color="#9467bd", ls="--", alpha=0.75, label="Zona 2")

            if "dist_r_ohm" in plot_df.columns and "dist_x_ohm" in plot_df.columns:
                r_ohm = plot_df["dist_r_ohm"].to_numpy(dtype=float)
                x_ohm = plot_df["dist_x_ohm"].to_numpy(dtype=float)
                ax_rx.plot(r_ohm, x_ohm, color="#2ca02c", lw=1.0, label="Trajetória R-X")
                theta = np.linspace(0.0, 2.0 * np.pi, 240)
                z1_ref = float(np.nanmedian(z1[np.isfinite(z1)])) if np.any(np.isfinite(z1)) else 0.0
                z2_ref = float(np.nanmedian(z2[np.isfinite(z2)])) if np.any(np.isfinite(z2)) else 0.0
                if z1_ref > 0.0:
                    ax_rx.plot(z1_ref * np.cos(theta), z1_ref * np.sin(theta), color="#ff7f0e", ls="--", alpha=0.75, label="Zona 1")
                if z2_ref > 0.0:
                    ax_rx.plot(z2_ref * np.cos(theta), z2_ref * np.sin(theta), color="#9467bd", ls="--", alpha=0.75, label="Zona 2")
                ax_rx.axhline(0.0, color="#888888", lw=0.8, alpha=0.5)
                ax_rx.axvline(0.0, color="#888888", lw=0.8, alpha=0.5)
            else:
                ax_rx.text(0.5, 0.5, "CSV sem dist_r_ohm/dist_x_ohm", transform=ax_rx.transAxes, ha="center")

            ax_trip.step(t, z1_trip, where="post", color="#ff7f0e", lw=1.6, label="Trip 21Z1")
            ax_trip.step(t, z2_trip, where="post", color="#9467bd", lw=1.6, label="Trip 21Z2")

            ax_z.set_title("Proteção de distância 21")
            ax_z.set_ylabel("|Z| (ohm)")
            ax_rx.set_title("Plano R-X")
            ax_rx.set_xlabel("R (ohm)")
            ax_rx.set_ylabel("X (ohm)")
            ax_rx.axis("equal")
            ax_trip.set_title("Atuação das zonas 21")
            ax_trip.set_ylabel("Trip")
            ax_trip.set_xlabel(x_label)
            ax_trip.set_ylim(-0.05, 1.15)
            ax_trip.set_yticks([0, 1])

            for ax in (ax_z, ax_rx, ax_trip):
                ax.grid(True, alpha=0.3)
                ax.legend(loc="upper right")
            ax_z.set_xlim(0.0, global_t_max)

            plt.tight_layout()
            plt.show()
            return

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
        elif base_mode == "fourier_raw":
            v_col, i_col = "v1_mag_raw", "i1_mag_raw"
            v_title, i_title = "Tensão fundamental RMS (Fourier bruta)", "Corrente fundamental RMS (Fourier bruta)"
            v_label, i_label = "V1 RMS bruto", "I1 RMS bruto"
        elif base_mode == "fourier_scaled":
            v_col, i_col = "v1_mag", "i1_mag"
            v_title, i_title = "Tensão fundamental RMS (Fourier escalonada)", "Corrente fundamental RMS (Fourier escalonada)"
            v_label, i_label = "V1 RMS escalonada", "I1 RMS escalonada"
        elif base_mode == "fourier_angle":
            v_col, i_col = "v1_angle_deg", "i1_angle_deg"
            v_title, i_title = "Ângulo da tensão fundamental", "Ângulo da corrente fundamental"
            v_label, i_label = "ângulo V1 (graus)", "ângulo I1 (graus)"
        elif base_mode == "fourier_phase":
            v_col, i_col = "vi_angle_deg", "vi_angle_deg"
            v_title, i_title = "Diferença angular I1 - V1", ""
            v_label, i_label = "I1 - V1 (graus)", ""
        elif base_mode == "over_current":
            v_col, i_col = "oc_50_trip", "oc_51_trip"
            v_title, i_title = "Trip instantâneo 50", "Trip temporizado 51"
            v_label, i_label = "OC50 trip", "OC51 trip"
        elif base_mode == "over_current_timer":
            v_col, i_col = "i1_mag", "oc_51_timer_s"
            v_title, i_title = "Corrente fundamental usada na proteção", "Temporizador 51"
            v_label, i_label = "I1 RMS", "tempo 51 (s)"
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
            if base_mode == "fourier_phase":
                ax_i.axis("off")
                ax_v.set_xlim(0.0, global_t_max)
                ax_v.set_xlabel(x_label)
                plt.tight_layout()
                plt.show()
                return
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

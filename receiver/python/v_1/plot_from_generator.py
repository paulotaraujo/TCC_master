import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ============================
# leitura do csv (2 canais)
# ============================
df = pd.read_csv("adc_capture.csv")

if "dev_t_s" in df.columns:
    t = df["dev_t_s"].values
elif "host_t_s" in df.columns:
    t = df["host_t_s"].values
elif "t_s" in df.columns:
    t = df["t_s"].values
elif "t" in df.columns:
    t = df["t"].values
else:
    raise ValueError("CSV sem coluna de tempo ('dev_t_s', 'host_t_s', 't' ou 't_s').")

required = {"adc_34", "adc_35"}
missing = required - set(df.columns)
if missing:
    raise ValueError(f"CSV sem colunas obrigatórias: {sorted(missing)}")

y34 = df["adc_34"].values
y35 = df["adc_35"].values

# ============================
# criação dos gráficos
# ============================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
ax1.plot(t, y34, lw=1, color="#1f77b4", label="ADC GPIO34")
ax2.plot(t, y35, lw=1, color="#d62728", label="ADC GPIO35")

expected = [0, 2048, 3723]
labels = ["0 V", "1.65 V", "3.0 V"]
colors = ["#2ca02c", "#7f7f7f", "#9467bd"]

for val, lab, c in zip(expected, labels, colors):
    ax1.axhline(val, color=c, lw=1, ls="--", alpha=0.6)
    ax2.axhline(val, color=c, lw=1, ls="--", alpha=0.6)

ax1.set_ylabel("ADC34 (0..4095)")
ax2.set_ylabel("ADC35 (0..4095)")
ax2.set_xlabel("tempo (s)")
ax1.set_title("Sinal capturado da ESP32 geradora - Canal GPIO34")
ax2.set_title("Sinal capturado da ESP32 geradora - Canal GPIO35")
ax1.legend(loc="upper right")
ax2.legend(loc="upper right")

# ============================
# cursor compartilhado
# ============================
vline1 = ax1.axvline(color="gray", lw=0.8, linestyle="--")
vline2 = ax2.axvline(color="gray", lw=0.8, linestyle="--")
hline1 = ax1.axhline(color="gray", lw=0.8, linestyle="--")
hline2 = ax2.axhline(color="gray", lw=0.8, linestyle="--")
text = ax1.text(1.01, 0.98, "", transform=ax1.transAxes, va="top")


def on_move(event):
    if event.xdata is None:
        return

    x = event.xdata
    idx = np.searchsorted(t, x)
    idx = min(max(idx, 0), len(t) - 1)

    tx = t[idx]
    ty34 = y34[idx]
    ty35 = y35[idx]

    vline1.set_xdata([tx, tx])
    vline2.set_xdata([tx, tx])
    hline1.set_ydata([ty34, ty34])
    hline2.set_ydata([ty35, ty35])

    text.set_text(
        f"tempo: {tx:.6f} s\nGPIO34: {int(ty34)}\nGPIO35: {int(ty35)}"
    )
    fig.canvas.draw_idle()


fig.canvas.mpl_connect("motion_notify_event", on_move)

for _, lab, c in zip(expected, labels, colors):
    ax1.plot([], [], ls="--", color=c, label=f"Esperado {lab}")
ax1.legend(loc="upper right")

plt.tight_layout()
plt.show()

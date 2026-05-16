import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ============================
# leitura do csv (compatível com t ou t_s)
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

if "adc" not in df.columns:
    raise ValueError("CSV sem coluna 'adc'.")

y = df["adc"].values

# ============================
# criação do gráfico
# ============================
fig, ax = plt.subplots(figsize=(11, 5))
ax.plot(t, y, lw=1, label="ADC lido")

# Linhas de referência esperadas para 0V, 1.65V, 3.0V
expected = [0, 2048, 3723]
labels = ["0 V", "1.65 V", "3.0 V"]
colors = ["#2ca02c", "#1f77b4", "#d62728"]

for val, lab, c in zip(expected, labels, colors):
    ax.axhline(val, color=c, lw=1, ls="--", alpha=0.7, label=f"Esperado {lab}: {val}")

ax.set_xlabel("tempo (s)")
ax.set_ylabel("ADC (0..4095)")
ax.set_title("Sinal capturado da ESP32 geradora")
ax.legend(loc="upper right")

# ============================
# cursor
# ============================
vline = ax.axvline(color="gray", lw=0.8, linestyle="--")
hline = ax.axhline(color="gray", lw=0.8, linestyle="--")
text = ax.text(1.02, 0.95, "", transform=ax.transAxes, va="top")

def on_move(event):
    if not event.inaxes or event.xdata is None:
        return

    x = event.xdata
    idx = np.searchsorted(t, x)
    idx = min(max(idx, 0), len(t) - 1)

    tx = t[idx]
    ty = y[idx]

    vline.set_xdata([tx, tx])
    hline.set_ydata([ty, ty])

    text.set_text(f"tempo: {tx:.6f} s\nadc: {int(ty)}")
    fig.canvas.draw_idle()

fig.canvas.mpl_connect("motion_notify_event", on_move)

plt.tight_layout()
plt.show()

#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

csv_path = "adc_capture_generator_loopback.csv"
df = pd.read_csv(csv_path)

t = df["host_t_s"].values
adc_v = df["adc_v"].values
adc_i = df["adc_i"].values
set_mv = df["set_mv_esp"].values

fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

ax[0].plot(t, adc_v, lw=0.8, label="ADC V")
ax[0].plot(t, adc_i, lw=0.8, label="ADC I", alpha=0.8)
ax[0].set_ylabel("ADC (0..4095)")
ax[0].legend()
ax[0].grid(True, alpha=0.3)

ax[1].plot(t, set_mv, lw=1.0, color="tab:red", label="Setpoint (mV)")
ax[1].set_xlabel("Tempo (s)")
ax[1].set_ylabel("mV")
ax[1].legend()
ax[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print(df.head())
print(df.describe())

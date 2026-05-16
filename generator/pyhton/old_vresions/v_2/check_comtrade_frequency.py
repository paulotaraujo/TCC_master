import struct
import numpy as np
import matplotlib.pyplot as plt

BDAT_FILE = "samples/sample_2/export.bdat"


def read_samples(n=5000):
    timestamps = []
    values = []

    with open(BDAT_FILE, "rb") as f:
        for _ in range(n):
            data = f.read(8)
            if not data or len(data) < 8:
                break

            sample, ts = struct.unpack("<II", data)

            val = struct.unpack("<h", f.read(2))[0]

            # ajuste conforme seu arquivo:
            f.seek(6, 1)

            timestamps.append(ts)
            values.append(val)

    return np.array(timestamps, dtype=np.float64), np.array(values, dtype=np.float64)


def estimate_freq_from_fft(ts, signal):
    signal = signal - np.mean(signal)

    dts = np.diff(ts)
    dt = np.median(dts)

    freqs = np.fft.rfftfreq(len(signal), d=dt)
    spectrum = np.fft.rfft(signal)

    peak = np.argmax(np.abs(spectrum[1:])) + 1
    return freqs[peak], dt


def main():
    ts, sig = read_samples()

    freq, dt = estimate_freq_from_fft(ts, sig)

    print(f"dt médio entre amostras: {dt}")
    print(f"Frequência estimada: {freq:.6f} Hz")

    plt.plot(sig[:500])
    plt.title("Primeiras amostras do COMTRADE")
    plt.show()


if __name__ == "__main__":
    main()
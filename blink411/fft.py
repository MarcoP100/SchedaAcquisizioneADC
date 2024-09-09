import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq

# Carica i dati dal file CSV
data = np.loadtxt('testFft.csv', delimiter=',', skiprows=1)  # Sostituisci con il tuo file CSV
sampling_rate = 8e6
sonda1 = data[:, 0]
sonda2 = data[:, 1]

# Calcola la FFT
N = len(sonda2)
fft_values = fft(sonda2)
fft_freqs = fftfreq(N, 1/sampling_rate)

# Plot del diagramma spettrale
plt.plot(fft_freqs[:N//2], np.abs(fft_values[:N//2]))
plt.xlabel('Frequenza (Hz)')
plt.ylabel('Ampiezza')
plt.show()

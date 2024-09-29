#include <SPI.h>
#include <AD9833.h>

// Pin di controllo del AD9833 (FSYNC)
const int FSYNC = 10;

// Crea l'oggetto AD9833
AD9833 gen(FSYNC);

void setup() {
  // Inizializza la comunicazione SPI
  SPI.begin();
  gen.begin();  // Inizializza l'AD9833
  Serial.begin(9600);

  
  gen.setWave(AD9833_SINE);  // Genera un'onda sinusoidale
  gen.setFrequency(5000);
}

void loop() {

  /*for (int freq = 0; freq <= 1000; freq += 300) {
    gen.setFrequency(freq);  // Imposta la frequenza dell'AD9833
    // Stampa la frequenza sul monitor seriale
    Serial.print("Frequenza: ");
    Serial.print(freq);
    Serial.println(" Hz");
    delay(3000);  // Aspetta 2 secondi prima di passare alla prossima frequenza
    
  }

  // Seconda fase: da 1000 Hz a 10 kHz con incrementi di 1000 Hz
  for (int freq = 1000; freq <= 10000; freq += 2000) {
    gen.setFrequency(freq);  // Imposta la frequenza dell'AD9833
    // Stampa la frequenza sul monitor seriale
    Serial.print("Frequenza: ");
    Serial.print(freq);
    Serial.println(" Hz");
    delay(3000);  // Aspetta 2 secondi prima di passare alla prossima frequenza
  }*/
}

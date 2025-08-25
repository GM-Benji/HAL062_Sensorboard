# HAL062_Sensorboard
## Sensorboard

Sensorboard odpowiada za obsługę wag i prostego indykatora.  
Komunikuje się z mainboardem przez magistralę CAN.

### Funkcje
- Pomiar masy z 3 wag i wysyłanie wyników (`int32_t` w 4 bajtach).
- Tarowanie wybranej wagi (komenda z CAN).
- Włączanie/wyłączanie indykatora.
## Communication

### Ramki odbierane
- **ID = 120**
- `data[0]`  
  - `1` → włączenie indykatora  
  - `2` → wyłączenie indykatora
- `data[1]`  
  - `1`, `2`, `3` → tarowanie wagi nr 1, 2, 3

### Ramki wysyłane
- **ID = 121**
- `data[0]` → ID wagi (`1`, `2`, `3`)
- `data[1..4]` → wynik ważenia (`int32_t`, zapisany w 4 bajtach)

# Wiring / Verdrahtung (LaserCat)

## Überblick
LaserCat nutzt:
- ESP8266 (Wemos D1 mini)
- 2 Servos (Pan/Tilt)
- 5V Laser-Modul (empfohlen über MOSFET/Transistor geschaltet)

**Wichtig:** Servos unbedingt über separates 5V Netzteil betreiben (>= 2A).  
**GND muss gemeinsam sein** (Servo-Netzteil GND ↔ Wemos GND).

---

## Pins (Default)
- Servo Pan Signal:  D5 (GPIO14)
- Servo Tilt Signal: D6 (GPIO12)
- Laser Signal:      D7 (GPIO13)  -> über MOSFET/Transistor

---

## Servos (Power + Signal)
### Servo-Netzteil (separat)
- Servo +5V   -> +5V Netzteil
- Servo GND   -> GND Netzteil

### Wemos Signale
- D5 -> Servo Pan (Signal)
- D6 -> Servo Tilt (Signal)

### Gemeinsame Masse
- Wemos GND -> Servo-Netzteil GND

---

## Laser über MOSFET (empfohlen)
### Beispiel mit N-Kanal Logic-Level MOSFET
- Wemos D7 -> Gate (über ~100–220Ω in Serie)
- Gate -> GND über Pull-Down (z.B. 100k)
- Laser +5V -> +5V Netzteil
- Laser GND -> MOSFET Drain
- MOSFET Source -> GND (Netzteil)
- GND Netzteil -> Wemos GND (gemeinsam)

> Dadurch schaltest du den Laser sauber, ohne den ESP-Pin zu überlasten.

---

## Optional: Stabilität
- 470–1000µF Elko an der Servo-5V-Schiene (nah an den Servos)
- kurze Kabel, saubere Masseführung

# 🔌 Wiring Guide

## System Overview

```
12V LiPo Battery
      │
      ├──────────────────────────────┐
      │                              │
   Buck Converter               BusLinker
   (12V → 5V)                  (12V Power)
      │                              │
   Raspberry Pi 4              LX-16A Motors (x8)
      │                         (daisy chained)
      └── USB → BusLinker
```

---

## Component Connections

### Battery → Buck Converter
| Battery | Buck Converter |
|---------|---------------|
| + (12V) | VIN+ |
| - (GND) | VIN- |

### Buck Converter → Raspberry Pi
| Buck Converter | Raspberry Pi |
|---------------|-------------|
| VOUT+ (5V) | USB-C Power |
| VOUT- (GND) | GND |

### Battery → BusLinker
| Battery | BusLinker |
|---------|-----------|
| + (12V) | VCC (motor power) |
| - (GND) | GND |

### Raspberry Pi → BusLinker
| Raspberry Pi | BusLinker |
|-------------|-----------|
| USB-A port | USB (CH340 data) |

### BusLinker → LX-16A Motors
Motors are daisy-chained using the 3-pin servo cable:

```
BusLinker → Motor 1 → Motor 2 → Motor 3 → ... → Motor 8
```

Each connection uses the 3-pin servo connector:
| Pin | Signal |
|-----|--------|
| Red | VCC (motor power from BusLinker) |
| Black | GND |
| Yellow | Serial data |

---

## Motor Chain Order

```
BusLinker
    └── FL_PITCH (ID 1)
        └── FL_YAW (ID 2)
            └── FR_YAW (ID 3)
                └── FR_PITCH (ID 4)
                    └── BL_PITCH (ID 5)
                        └── BL_YAW (ID 6)
                            └── BR_YAW (ID 7)
                                └── BR_PITCH (ID 8)
```

---

## Raspberry Pi GPIO (if using UART instead of USB)

| Pi GPIO Pin | Signal |
|-------------|--------|
| Pin 1 (3.3V) | — |
| Pin 6 (GND) | GND |
| Pin 8 (TX) | Serial TX to BusLinker |
| Pin 10 (RX) | Serial RX from BusLinker |

> **Note:** If using GPIO UART, change `SERIAL_PORT = "/dev/ttyAMA0"` in `crab_gait.py`

---

## Safety Notes

- Always connect battery **last** after all wiring is complete
- Buck converter output must be set to **5V** before connecting to Pi
- Never exceed **12V** on the BusLinker motor power input
- Always run shutdown procedure (`q`) before powering off

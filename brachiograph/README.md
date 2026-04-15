Dit project gaat uit van het voorbeeld project [Brachiograph.art](https://brachiograph.art/). Alleen dat project is gemaakt voor de Raspberry Pi. De Raspberry Pi Pico is wat beperkter in opslag (voor het opslaan van plaatjes in JSON formaat), geheugen (voor het inlezen van de JSON plaatjes) en in beschikbare libraries. Zo is `numpy` bijvoorbeeld niet zomaar beschikbaar.

Ik heb Claude Sonnet 4.6 via GitHub CoPilot gevraagd de code om te schrijven naar MicroPython.

> User: Can you port plotter.py to MicroPython instead of Python
> 
> GitHub Copilot:
> 
> Here's the full breakdown of changes needed before I write the port:
> 
> | Original | MicroPython replacement |
> |---|---|
> | `pigpio` | `machine.PWM` / `machine.Pin` |
> | `numpy.polyfit` / `numpy.poly1d` | Pure-Python Gaussian elimination |
> | `numpy.mean` | Manual mean |
> | `tqdm` | Plain `for` loops with `print` progress |
> | `readchar` | `sys.stdin.read(1)` |
> | `pprint` | `json.dumps` + `print` |
> | `time.monotonic()` | `ticks_ms()` / `ticks_diff()` |
> | Turtle support | Removed (unavailable on MicroPython) | 
> 
> **Removed dependencies**
> - `pigpio` → `machine.PWM` / `machine.Pin`. Servo pulse widths (µs) are converted to `duty_u16` via `pw_us * 65535 // 20000` (50 Hz = 20 ms period).
> - `numpy` → pure-Python `polyfit` (Gaussian elimination on the normal equations) and `poly1d` (Horner's method). `numpy.mean` → `_mean()`.
> - `tqdm` → plain `for` loops; `plot_lines` prints `Lines: n/total` instead.
> - `readchar` → `sys.stdin.read(1)` (reads one character from the serial REPL).
> - `pprint` → `json.dumps()` + `print()`.
> - `turtle` support → removed entirely.
> 
> **Time handling**
> - `time.monotonic()` → `ticks_ms()` / `ticks_diff()` from MicroPython's `time` module. `ticks_diff` is used to correctly handle the millisecond counter rollover; the result is divided by 1000 to get seconds.
> 
> **Hardware detection**
> - `pigpio` daemon fallback heuristic replaced with a single `try/except ImportError` on `machine`, setting `HARDWARE_AVAILABLE`. Virtual mode is entered when `virtual=True` or when running on a host without `machine`.
> 
> **PWM read-back**
> - `pigpio` can read back the current servo pulse width; `machine.PWM` cannot. `_current_pw_1`, `_current_pw_2`, and `Pen._current_pw` shadow the last-written values so `get_pulse_widths()` / `get_pw()` still work correctly.
> 
> **Other fixes**
> - Mutable default arguments (`lines=[]`) replaced with `None` guards — important on MicroPython where module-level objects persist across calls.
> - `quiet()` default changed from `servos=[14,15,18]` to `servos=None` with a `None`-guard for the same reason.

Hierna heb ik nog de juiste pinnen van de servo's ingesteld en branchiograph.py aangepast, omdat die nog referenties naar het verwijderde turtle support had.

Upload de `branchiograph.py`, `plotter.py` en `demo.json` naar de Raspberry Pi Pico. Open het bestand bg.py en voer het uit (F5). Nu kun je onder in de Shell commando's uitvoeren, zoals beschreven in bijvoorbeeld [Start up the brachiograph](https://www.brachiograph.art/tutorial/power/):
```
bg.set_angles(0, 90)
bg.park()
bg.plot_file("demo.json")
```

Let op dat `bg.test_pattern()` niet werkt, omdat dat patroon uit gaat van een onderarm van 8 cm, en die van ons is maar 6 cm.

Ook de demo.json heb ik moeten verkleinen (het is maar een kwart van het originele plaatje), omdat de brachiograph alles in een keer in het geheugen probeert te laden, maar daar niet genoeg geheugen voor heeft. Nog een toekomstig verbeterpunt.
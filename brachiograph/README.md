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

Upload de `branchiograph.py`, `plotter.py`, `demo-small.json` en `demo.json` naar de Raspberry Pi Pico. Open het bestand bg.py en voer het uit (F5). Nu kun je onder in de Shell commando's uitvoeren, zoals beschreven in [Start up the brachiograph](https://www.brachiograph.art/tutorial/power/). Let op dat deze brachiograph ervan uit gaat dat de bovenarm 90 graden naar links gedraait begint en de onderarm daar haaks op staat.
```
bg.set_angles(0, 90)
bg.park()
bg.plot_file("demo-small.json")
```

Let op dat `bg.test_pattern()` niet werkt, omdat dat patroon uit gaat van een onderarm van 8 cm, en die van ons is maar 6 cm.

Ook de `demo.json` heb ik moeten verkleinen naar `demo-small.json` (het is maar een kwart van het originele plaatje), omdat de brachiograph alles in een keer in het geheugen probeert te laden, maar daar niet genoeg geheugen voor heeft. Ook daar kan Claude mee helpen:

> User: Due to memory restrictions on the Raspberry Pi Pico, plotting big files using the `plot_file` method runs out of memory. Can you improve the code to implement either streaming or chunking in the plot_files method?
> 
> GitHub Copilot: Looking at `plot_file`, the issue is twofold: `json.load()` loads the entire file at once, and `rotate_and_scale_lines` also keeps all data in memory. I'll replace it with a two-pass streaming approach: first pass computes the transform parameters (min/max bounds), second pass reads and plots one polyline at a time. 
> 
> Replacing 8 lines with 145 lines in plotter.py.
> 
> Looks good. Here's what the implementation does:
> 
> **`_iter_lines_from_file`** — a generator that reads the file in 256-byte chunks and tracks bracket depth. When depth goes from 1→2, it starts accumulating characters into a buffer; when depth drops back to 1, it calls `json.loads()` on that buffer (one polyline) and yields it. Only one polyline's text is ever in `buf` at a time.
> 
> **`_compute_file_transform`** — the first pass. Iterates using the generator, tracking only `min_x/max_x/min_y/max_y` (four floats). Replicates the logic from `analyse_lines` to determine rotation, scaling, and midpoints.
> 
> **`plot_file`** — now does two passes over the file. The first computes the transform; the second streams polylines one at a time, applies the transform point-by-point, and plots them directly — no list of all lines, no `rotate_and_scale_lines` call on the full dataset.
> 
> Peak memory use is now proportional to one polyline rather than the entire file.

Ik heb zelf nog een paar kleine verbeteringen uitgevoerd, maar nu kan die inderdaad in een keer de hele demo.json tekenen met `bg.plot_file("demo.json")`

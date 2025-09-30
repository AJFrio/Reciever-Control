## Receiver Control

This application detects a raised, open hand with MediaPipe and drives an L9110S H-bridge from a Raspberry Pi to spin a single DC motor. The motor direction follows the detected hand tilt:

- Left tilt → forward (by default)
- Right tilt → reverse
- Vertical (within `45°`) → motor stops

## Housing CAD

- [Onshape enclosure model](https://cad.onshape.com/documents/6807801388cd1d3c9d6b9f2a/w/23686d7df571e290bd4fe000/e/c297ecbb82ea916d8e0fea8e?renderMode=0&uiState=68dac285ec910c36b04511ce)

## Hardware Wiring

- `MOTOR_FORWARD_PIN` (`GPIO17` by default) → Driver board `a-1a`
- `MOTOR_BACKWARD_PIN` (`GPIO18` by default) → Driver board `a-1b`
- Raspberry Pi `5 V` (or external motor supply within 2.5–12 V) → Driver board `VCC`
- Raspberry Pi `GND` → Driver board `GND`
- Motor leads connect to motor output pins (typically `OA1`/`OA2` for motor A channel)

> **Tip:** Use an external supply for motors that draw more than the Pi’s 5 V rail can safely provide. Always share ground between the Pi and the driver supply.

```
Raspberry Pi (BCM)          Driver Board              DC Motor
┌────────────────────┐         ┌────────────────────┐       ┌──────────┐
│ GPIO17 ────────────┼────────▶│ a-1a               │──────▶│ Motor +  │
│ GPIO18 ────────────┼────────▶│ a-1b               │──────▶│ Motor -  │
│ GND   ─────────────┼────────▶│ GND                │       └──────────┘
│ 5V*  ──────────────┼────────▶│ VCC (2.5–12 V)     │
└────────────────────┘         └────────────────────┘
            ▲                             ▲
            │                             │
            └─────────────── Shared Ground ────────────────┘

*Use an external supply within the driver’s range when your motor needs more
 current than the Pi’s 5 V pin can deliver. Connect its positive lead to `VCC`
 and its ground to the common ground line.
```

## Configuration

Open `main.py` and adjust the constants near the top as needed:

- `MOTOR_FORWARD_PIN` / `MOTOR_BACKWARD_PIN`: change to the BCM pins you wired to `a-1a` and `a-1b`
- `LEFT_DIRECTION_IS_FORWARD`: set `False` if you wire the driver differently or prefer the right tilt to be forward
- `MOTOR_NEUTRAL_ANGLE`: widen or narrow the neutral zone that keeps the motor stopped
- Motor speed is controlled via PWM and set to 20% by default (80% reduction from full speed)

You can also modify the speed by changing the `speed` parameter when creating the `MotorController` instance:

```python
motor_controller = MotorController(MOTOR_FORWARD_PIN, MOTOR_BACKWARD_PIN, speed=0.5)  # 50% speed
```

Or use the `set_speed()` method at runtime:

```python
motor_controller.set_speed(0.3)  # 30% speed
```

## Running

```bash
python main.py
```

On non-Raspberry Pi systems the motor calls fall back to console logging so you can test the hand-tracking logic without GPIO access.


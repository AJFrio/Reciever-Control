## Receiver Control

This application detects a raised, open hand with MediaPipe and drives an L9110S H-bridge from a Raspberry Pi to spin a single DC motor. The motor direction follows the detected hand tilt:

- Left tilt → forward (by default)
- Right tilt → reverse
- Vertical (within `45°`) → motor stops
- **Face or hand not detected** → motor stops immediately

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

### Performance Optimizations

For better performance on Raspberry Pi (currently optimized for ~15-20 FPS):

- **Frame Resolution**: Reduced to 320×240 (from 640×480) - 4× less processing
- **Hand Model**: Set to `model_complexity=0` (lightest) for faster processing
- **Face Mesh**: Runs every 5th frame (set `FACE_MESH_UPDATE_INTERVAL=0` to disable entirely)
- **Target FPS**: Set to 15 FPS for smoother performance

For maximum speed, you can disable face mesh entirely by setting `FACE_MESH_UPDATE_INTERVAL = 0`.

You can also modify the speed by changing the `speed` parameter when creating the `MotorController` instance:

## Autostart Setup (Raspberry Pi)

To automatically start the Receiver Control system when your Raspberry Pi boots:

### Files Created:
- `start_receiver_control.sh` - Startup script that runs the application using UV
- `receiver-control.service` - Systemd service file for autostart

### Step-by-Step Installation:

1. **Copy the startup files to your Raspberry Pi:**
   ```bash
   # Copy these files to your Raspberry Pi's Reciever-Control-main directory
   # (Make sure you're in the project directory on your Pi)
   ```

2. **Make the startup script executable:**
   ```bash
   chmod +x start_receiver_control.sh
   ```

3. **Copy the service file to systemd:**
   ```bash
   sudo cp /home/frioa1/Downloads/Reciever-Control-main/receiver-control.service /etc/systemd/system/
   ```

4. **Reload systemd to recognize the new service:**
   ```bash
   sudo systemctl daemon-reload
   ```

5. **Enable the service to start on boot:**
   ```bash
   sudo systemctl enable receiver-control.service
   ```

6. **Start the service immediately (optional):**
   ```bash
   sudo systemctl start receiver-control.service
   # Or start from the project directory:
   # cd /home/frioa1/Downloads/Reciever-Control-main && ./start_receiver_control.sh
   ```

7. **Check service status:**
   ```bash
   sudo systemctl status receiver-control.service
   ```

### Service Management:

- **Check if running:** `sudo systemctl status receiver-control.service`
- **View logs:** `sudo journalctl -u receiver-control.service -f`
- **Restart:** `sudo systemctl restart receiver-control.service`
- **Stop:** `sudo systemctl stop receiver-control.service`
- **Disable autostart:** `sudo systemctl disable receiver-control.service`

### Important Notes:

- The service runs as the `pi` user and assumes your project is in `/home/frioa1/Downloads/Reciever-Control-main`
- If using a different username or path, edit the service file accordingly
- The service includes display environment variables for GUI applications
- If you modify the script, restart the service: `sudo systemctl restart receiver-control.service`

### Runtime Speed Control:

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

## Safety Features

The system includes automatic motor shutdown for safety:
- **Face or Hand Detection Loss**: If either the face or hand disappears from view, or if the hand doesn't meet the detection criteria (open hand at head height), the motor immediately stops
- **PWM Speed Control**: Motor runs at reduced speed (20% by default) to prevent excessive current draw
- **GPIO Error Handling**: Graceful fallback to simulation mode if GPIO hardware is unavailable


## Receiver Control

This application detects a raised, open hand with MediaPipe and drives an L9110S H-bridge from a Raspberry Pi to control a stepper motor. The motor direction follows the detected hand tilt:

- Left tilt → clockwise rotation (by default)
- Right tilt → counterclockwise rotation
- Vertical (within `45°`) → motor stops
- **Face or hand not detected** → motor stops immediately

## Housing CAD

- [Onshape enclosure model](https://cad.onshape.com/documents/6807801388cd1d3c9d6b9f2a/w/23686d7df571e290bd4fe000/e/c297ecbb82ea916d8e0fea8e?renderMode=0&uiState=68dac285ec910c36b04511ce)

## Hardware Wiring

### Stepper Motor Connection

The L9110S driver board uses two H-bridges to control a bipolar stepper motor:

- `STEPPER_COIL_A_PIN1` (`GPIO17` by default) → Driver board `a-1a` (Coil A control 1)
- `STEPPER_COIL_A_PIN2` (`GPIO18` by default) → Driver board `a-1b` (Coil A control 2)
- `STEPPER_COIL_B_PIN1` (`GPIO22` by default) → Driver board `b-1a` (Coil B control 1)
- `STEPPER_COIL_B_PIN2` (`GPIO23` by default) → Driver board `b-2a` (Coil B control 2)
- Raspberry Pi `5 V` (or external motor supply within 2.5–12 V) → Driver board `VCC`
- Raspberry Pi `GND` → Driver board `GND`

### Stepper Motor Wiring

Connect your bipolar stepper motor coils to the driver board motor outputs:

```
Raspberry Pi (BCM)          Driver Board              Stepper Motor
┌────────────────────┐         ┌────────────────────┐       ┌──────────┐
│ GPIO17 ────────────┼────────▶│ a-1a               │       │ Coil A   │
│ GPIO18 ────────────┼────────▶│ a-1b               │──────▶│          │
│ GPIO22 ────────────┼────────▶│ b-1a               │       │ Coil B   │
│ GPIO23 ────────────┼────────▶│ b-2a               │──────▶│          │
│ GND   ─────────────┼────────▶│ GND                │       └──────────┘
│ 5V*  ──────────────┼────────▶│ VCC (2.5–12 V)     │
└────────────────────┘         └────────────────────┘
            ▲                             ▲
            │                             │
            └─────────────── Shared Ground ────────────────┘

*Use an external supply within the driver's range when your motor needs more
 current than the Pi's 5 V pin can deliver. Connect its positive lead to VCC
 and its ground to the common ground line.
```

### Motor Output Connections

The driver board's motor output terminals (typically labeled OA1/OA2 for channel A and OB1/OB2 for channel B) should be connected to your stepper motor coils:

- **Coil A**: Connect to OA1 and OA2 terminals
- **Coil B**: Connect to OB1 and OB2 terminals

> **Important:** Make sure to identify which wires of your stepper motor correspond to which coil. You may need to experiment or consult your motor's datasheet to determine the correct coil pairing.

## Configuration

Open `main.py` and adjust the constants near the top as needed:

- `STEPPER_COIL_A_PIN1` / `STEPPER_COIL_A_PIN2`: BCM pins wired to `a-1a` and `a-1b` (controls coil A)
- `STEPPER_COIL_B_PIN1` / `STEPPER_COIL_B_PIN2`: BCM pins wired to `b-1a` and `b-2a` (controls coil B)
- `LEFT_DIRECTION_IS_FORWARD`: set `False` if you prefer right tilt to trigger clockwise rotation
- `MOTOR_NEUTRAL_ANGLE`: widen or narrow the neutral zone that keeps the motor stopped
- `STEPPER_SPEED`: motor speed in steps per second (1.0 to 200.0, default: 100.0)
- `STEPPER_MODE`: "full" for full-step mode, "half" for half-step mode (smoother but slower)

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
   chmod +x /home/frioaj1/Downloads/Reciever-Control-main/start_receiver_control.sh
   ```

3. **Copy the service file to systemd:**
   ```bash
   sudo cp /home/frioaj1/Downloads/Reciever-Control-main/receiver-control.service /etc/systemd/system/
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
   # cd /home/frioaj1/Downloads/Reciever-Control-main && ./start_receiver_control.sh
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

- The service runs as the `frioaj1` user and assumes your project is in `/home/frioaj1/Downloads/Reciever-Control-main`
- If using a different username or path, edit the service file accordingly
- The service includes display environment variables for GUI applications
- If you modify the script, restart the service: `sudo systemctl restart receiver-control.service`
- **Note:** The `frioaj1` user needs sudo privileges to manage (start/stop/enable) the service
- **UV Requirement:** Make sure UV is installed and available in the frioaj1 user's PATH. Install with: `curl -LsSf https://astral.sh/uv/install.sh | sh`

### Runtime Speed Control:

```python
motor_controller = StepperMotorController(
    STEPPER_COIL_A_PIN1, STEPPER_COIL_A_PIN2,
    STEPPER_COIL_B_PIN1, STEPPER_COIL_B_PIN2,
    speed=150.0  # 150 steps per second
)
```

Or use the `set_speed()` method at runtime:

```python
motor_controller.set_speed(75.0)  # 75 steps per second
```

## Running

```bash
python main.py
```

On non-Raspberry Pi systems the motor calls fall back to console logging so you can test the hand-tracking logic without GPIO access.

## Safety Features

The system includes automatic motor shutdown for safety:
- **Face or Hand Detection Loss**: If either the face or hand disappears from view, or if the hand doesn't meet the detection criteria (open hand at head height), the motor immediately stops
- **Speed Control**: Motor speed is controlled via step rate (default: 100 steps/second) to prevent excessive current draw
- **GPIO Error Handling**: Graceful fallback to simulation mode if GPIO hardware is unavailable
- **Coil De-energizing**: When stopped, all motor coils are de-energized to prevent overheating


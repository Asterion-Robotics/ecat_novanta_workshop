import math
import threading
from typing import Dict

from flask import Flask, jsonify, redirect, render_template_string, request, url_for

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

_LOGO_URL = "https://raw.githubusercontent.com/Asterion-Robotics/assets/main/asterion-logo-flat.png"


class GpioGuiNode(Node):
    _PULSE_SECONDS = 0.2
    _DEFAULT_VELOCITY = 0.0
    _WHEEL_JOINT = 'right_wheel_joint'

    def __init__(self):
        super().__init__('gpio_web_node')
        self.publisher = self.create_publisher(
            DynamicInterfaceGroupValues,
            '/gpio_controller/commands',
            10,
        )
        self.drive_reset_publisher = self.create_publisher(
            DynamicInterfaceGroupValues,
            '/drive_reset_controller/commands',
            10,
        )
        self.velocity_publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10,
        )
        self.state_subscriber = self.create_subscription(
            DynamicInterfaceGroupValues,
            '/gpio_controller/gpio_states',
            self._on_gpio_states,
            10,
        )
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_states,
            10,
        )
        self.state = {
            'status_tower_red': 0.0,
            'status_tower_yellow': 0.0,
            'status_tower_green': 0.0,
            'status_tower_buzzer': 0.0,
        }
        self.safety_input_vars = 0
        self.safety_output_vars = 0
        self.safety_logic_state = 0
        self.target_velocity = self._DEFAULT_VELOCITY
        self._joint_position = 0.0
        self._lock = threading.Lock()

    def set_gpio(self, name: str, enabled: bool) -> None:
        with self._lock:
            self.state[name] = 1.0 if enabled else 0.0
        self.publish_gpio_command()

    def set_all_off(self) -> None:
        with self._lock:
            for key in self.state.keys():
                self.state[key] = 0.0
            self.safety_input_vars = 0
        self.publish_gpio_command()

    def get_state(self) -> Dict[str, float]:
        with self._lock:
            return dict(self.state)

    def get_safety_snapshot(self) -> Dict[str, int | str]:
        with self._lock:
            input_vars = int(self.safety_input_vars) & 0xFF
            output_vars = int(self.safety_output_vars) & 0xFF
            logic_state = int(self.safety_logic_state)
            return {
                'safety_input_vars': input_vars,
                'safety_output_vars': output_vars,
                'safety_logic_state': logic_state,
                'run': 1 if (input_vars & (1 << 1)) else 0,
                'error_led': 1 if (output_vars & (1 << 0)) else 0,
                'estop_ok': 1 if (output_vars & (1 << 1)) else 0,
                'logic_state_text': 'RUN' if logic_state == 1 else str(logic_state),
            }

    def get_motor_snapshot(self) -> Dict[str, float]:
        with self._lock:
            return {
                'target_velocity': float(self.target_velocity),
            }

    def set_target_velocity(self, velocity: float) -> None:
        with self._lock:
            self.target_velocity = float(velocity)
        self.publish_velocity_command()

    def publish_velocity_command(self) -> None:
        velocity_snapshot = self.get_motor_snapshot()
        msg = Float64MultiArray()
        msg.data = [velocity_snapshot['target_velocity']]
        self.velocity_publisher.publish(msg)

    def pulse_drive_reset(self) -> None:
        self._publish_drive_reset_command(True)
        timer = threading.Timer(
            self._PULSE_SECONDS,
            self._publish_drive_reset_command,
            args=(False,),
        )
        timer.daemon = True
        timer.start()

    def _publish_drive_reset_command(self, enabled: bool) -> None:
        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = ['right_wheel_joint']
        value = InterfaceValue()
        value.interface_names = ['reset_fault']
        value.values = [1.0 if enabled else 0.0]
        msg.interface_values = [value]
        self.drive_reset_publisher.publish(msg)

    def set_safety_input_bit(self, bit_index: int, enabled: bool) -> None:
        if bit_index < 0 or bit_index > 7:
            return
        with self._lock:
            if enabled:
                self.safety_input_vars |= (1 << bit_index)
            else:
                self.safety_input_vars &= ~(1 << bit_index)
        self.publish_gpio_command()

    def pulse_safety_input_bit(self, bit_index: int) -> None:
        self.set_safety_input_bit(bit_index, True)
        timer = threading.Timer(
            self._PULSE_SECONDS,
            self.set_safety_input_bit,
            args=(bit_index, False),
        )
        timer.daemon = True
        timer.start()

    def _on_gpio_states(self, msg: DynamicInterfaceGroupValues) -> None:
        with self._lock:
            for group_name, interface_value in zip(msg.interface_groups, msg.interface_values):
                if group_name != 'el1918':
                    continue
                for name, value in zip(interface_value.interface_names, interface_value.values):
                    if name == 'safety_logic_state':
                        if not math.isnan(value):
                            self.safety_logic_state = int(value)
                    elif name == 'safety_output_vars':
                        if not math.isnan(value):
                            self.safety_output_vars = int(value)

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            idx = list(msg.name).index(self._WHEEL_JOINT)
            if idx < len(msg.position) and not math.isnan(msg.position[idx]):
                with self._lock:
                    self._joint_position = float(msg.position[idx])
        except ValueError:
            pass

    def get_joint_snapshot(self) -> Dict[str, float]:
        with self._lock:
            return {'position': self._joint_position}

    def publish_gpio_command(self) -> None:
        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = ['el2008', 'el1918']
        state_snapshot = self.get_state()
        safety_snapshot = self.get_safety_snapshot()

        el2008_value = InterfaceValue()
        el2008_value.interface_names = list(state_snapshot.keys())
        el2008_value.values = [state_snapshot[name] for name in el2008_value.interface_names]

        el1918_value = InterfaceValue()
        el1918_value.interface_names = ['safety_input_vars']
        el1918_value.values = [float(safety_snapshot['safety_input_vars'])]

        msg.interface_values = [el2008_value, el1918_value]
        self.publisher.publish(msg)


def create_app(node: GpioGuiNode) -> Flask:
    app = Flask(__name__)

    def get_dashboard_snapshot() -> Dict[str, Dict[str, float] | Dict[str, int | str]]:
        return {
            'gpio': node.get_state(),
            'safety': node.get_safety_snapshot(),
            'motor': node.get_motor_snapshot(),
            'joint': node.get_joint_snapshot(),
        }

    page = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>enwbot Motor Control</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

    :root {
      --accent:       #00c4ff;
      --accent-dim:   rgba(0, 196, 255, 0.14);
      --accent-glow:  rgba(0, 196, 255, 0.38);
      --text:         #ddf0ff;
      --muted:        rgba(180, 220, 255, 0.52);
      --surface:      rgba(0, 48, 110, 0.30);
      --border:       rgba(0, 196, 255, 0.20);
      --danger:       #ff3f55;
      --danger-dim:   rgba(255, 63, 85, 0.12);
      --danger-border:rgba(255, 63, 85, 0.32);
    }

    body {
      min-height: 100vh;
      background:
        radial-gradient(ellipse 90% 55% at 50% -5%, rgba(0, 100, 255, 0.50), transparent),
        linear-gradient(180deg, #000c1e 0%, #000818 100%);
      color: var(--text);
      font-family: "Segoe UI", system-ui, -apple-system, sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 48px 20px 56px;
    }

    /* ── Header ── */
    header {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 14px;
      margin-bottom: 40px;
    }
    header img {
      height: 52px;
      filter: drop-shadow(0 0 14px rgba(0, 196, 255, 0.45));
    }
    header h1 {
      font-size: 1.55rem;
      font-weight: 700;
      letter-spacing: -0.025em;
    }
    header p {
      font-size: 0.88rem;
      color: var(--muted);
    }
    header code {
      background: var(--accent-dim);
      border-radius: 5px;
      padding: 2px 7px;
      font-size: 0.83em;
      color: var(--accent);
    }

    /* ── Card ── */
    .card {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 28px;
      backdrop-filter: blur(24px);
      -webkit-backdrop-filter: blur(24px);
      padding: 40px 44px;
      width: 100%;
      max-width: 460px;
      box-shadow:
        0 0 0 1px rgba(0, 196, 255, 0.07) inset,
        0 28px 64px rgba(0, 0, 0, 0.55),
        0 0 90px rgba(0, 70, 255, 0.10);
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 36px;
    }

    /* ── Wheel ── */
    .wheel-wrap {
      position: relative;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 12px;
    }
    .wheel-glow {
      position: absolute;
      inset: -22px;
      border-radius: 50%;
      background: radial-gradient(circle, var(--accent-glow) 0%, transparent 68%);
      animation: pulse 2.6s ease-in-out infinite;
      pointer-events: none;
    }
    @keyframes pulse {
      0%, 100% { opacity: 0.35; transform: scale(0.94); }
      50%       { opacity: 0.70; transform: scale(1.06); }
    }
    #wheel-svg {
      width: 210px;
      height: 210px;
      filter: drop-shadow(0 0 10px rgba(0, 196, 255, 0.55));
      position: relative;
    }
    .wheel-readout {
      font-size: 0.82rem;
      color: var(--muted);
      letter-spacing: 0.01em;
    }
    .wheel-readout span {
      color: var(--accent);
      font-weight: 700;
      font-variant-numeric: tabular-nums;
    }

    /* ── Divider ── */
    .divider {
      width: 100%;
      height: 1px;
      background: linear-gradient(to right, transparent, var(--border), transparent);
    }

    /* ── Velocity section ── */
    .vel-section { width: 100%; display: flex; flex-direction: column; gap: 14px; }

    .vel-header {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
    }
    .vel-header .label {
      font-size: 0.88rem;
      font-weight: 600;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.07em;
    }
    .vel-header .readout {
      font-size: 1.5rem;
      font-weight: 800;
      color: var(--accent);
      font-variant-numeric: tabular-nums;
      line-height: 1;
    }
    .vel-header .unit {
      font-size: 0.78rem;
      color: var(--muted);
      margin-left: 4px;
      font-weight: 400;
    }

    /* Range input */
    input[type="range"] {
      -webkit-appearance: none;
      appearance: none;
      width: 100%;
      height: 6px;
      border-radius: 999px;
      outline: none;
      cursor: pointer;
      background: linear-gradient(to right,
        var(--accent) 50%,
        rgba(0, 196, 255, 0.15) 50%);
    }
    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 22px; height: 22px;
      border-radius: 50%;
      background: var(--accent);
      border: 3px solid #fff;
      box-shadow: 0 0 0 3px var(--accent-glow), 0 3px 10px rgba(0,0,0,0.4);
      cursor: pointer;
      transition: box-shadow 0.12s;
    }
    input[type="range"]::-webkit-slider-thumb:hover {
      box-shadow: 0 0 0 6px var(--accent-glow), 0 3px 14px rgba(0,0,0,0.4);
    }
    input[type="range"]::-moz-range-thumb {
      width: 22px; height: 22px;
      border-radius: 50%;
      background: var(--accent);
      border: 3px solid #fff;
      box-shadow: 0 0 0 3px var(--accent-glow);
      cursor: pointer;
    }
    .range-limits {
      display: flex;
      justify-content: space-between;
      font-size: 0.75rem;
      color: var(--muted);
      margin-top: 4px;
    }

    /* Stop button */
    .stop-btn {
      width: 100%;
      padding: 15px;
      border: 1px solid var(--danger-border);
      border-radius: 14px;
      background: var(--danger-dim);
      color: var(--danger);
      font-size: 0.9rem;
      font-weight: 800;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      cursor: pointer;
      transition: background 0.14s, transform 0.1s;
    }
    .stop-btn:hover  { background: rgba(255, 63, 85, 0.22); transform: translateY(-1px); }
    .stop-btn:active { transform: translateY(0); }

    /* Footer */
    footer {
      margin-top: 40px;
      color: var(--muted);
      font-size: 0.75rem;
      letter-spacing: 0.04em;
      text-align: center;
    }

    @media (max-width: 520px) {
      .card { padding: 28px 20px; }
      #wheel-svg { width: 170px; height: 170px; }
    }
  </style>
</head>
<body>

  <header>
    <img src="{{ logo_url }}" alt="Asterion Robotics" />
    <h1>enwbot Motor Control</h1>
    <p>Velocity interface &rarr; <code>/velocity_controller/commands</code></p>
  </header>

  <div class="card">

    <!-- Spinning wheel -->
    <div class="wheel-wrap">
      <div class="wheel-glow"></div>
      <svg id="wheel-svg" viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
        <!-- Static background tint -->
        <circle cx="50" cy="50" r="47" fill="rgba(0,196,255,0.04)"/>
        <!-- Rotating group -->
        <g id="wheel-group">
          <!-- Outer rim -->
          <circle cx="50" cy="50" r="44" fill="none" stroke="#00c4ff" stroke-width="3.5"/>
          <!-- Inner ring -->
          <circle cx="50" cy="50" r="34" fill="none" stroke="#00c4ff" stroke-width="0.9" opacity="0.28"/>
          <!-- 6 Spokes -->
          <line x1="50" y1="50" x2="50"   y2="6"    stroke="#00c4ff" stroke-width="2.5" stroke-linecap="round"/>
          <line x1="50" y1="50" x2="88.1" y2="28"   stroke="#00c4ff" stroke-width="2.5" stroke-linecap="round"/>
          <line x1="50" y1="50" x2="88.1" y2="72"   stroke="#00c4ff" stroke-width="2.5" stroke-linecap="round"/>
          <line x1="50" y1="50" x2="50"   y2="94"   stroke="#00c4ff" stroke-width="2.5" stroke-linecap="round"/>
          <line x1="50" y1="50" x2="11.9" y2="72"   stroke="#00c4ff" stroke-width="2.5" stroke-linecap="round"/>
          <line x1="50" y1="50" x2="11.9" y2="28"   stroke="#00c4ff" stroke-width="2.5" stroke-linecap="round"/>
          <!-- Hub -->
          <circle cx="50" cy="50" r="8"   fill="#00c4ff"/>
          <circle cx="50" cy="50" r="3.2" fill="#000c1e"/>
        </g>
      </svg>
      <div class="wheel-readout">
        joint position &nbsp;<span id="joint-pos">{{ '%.3f'|format(joint.position) }}</span>&nbsp;rad
      </div>
    </div>

    <div class="divider"></div>

    <!-- Velocity control -->
    <div class="vel-section">
      <div class="vel-header">
        <span class="label">Target Velocity</span>
        <span>
          <span class="readout" id="vel-display">{{ '%.2f'|format(motor.target_velocity) }}</span>
          <span class="unit">rad/s</span>
        </span>
      </div>

      <input
        id="vel-slider"
        type="range"
        min="-50.0"
        max="50.0"
        step="0.05"
        value="{{ '%.2f'|format(motor.target_velocity) }}"
      />
      <div class="range-limits"><span>−50.0</span><span>+50.0</span></div>
    </div>

    <button class="stop-btn" id="stop-btn">&#9632;&nbsp; Stop</button>

  </div>

  <script>
    const slider     = document.getElementById('vel-slider');
    const velDisplay = document.getElementById('vel-display');
    const jointPos   = document.getElementById('joint-pos');
    const stopBtn    = document.getElementById('stop-btn');
    const wheelGroup = document.getElementById('wheel-group');

    // ── Wheel animation (rAF smooth lerp) ──────────────────────────────
    let prevPosRad    = null;
    let targetDeg     = 0;
    let displayDeg    = 0;

    function applyWheelPosition(posRad) {
      if (prevPosRad === null) { prevPosRad = posRad; return; }
      let delta = posRad - prevPosRad;
      if (delta >  Math.PI) delta -= 2 * Math.PI;
      if (delta < -Math.PI) delta += 2 * Math.PI;
      targetDeg  += delta * (180 / Math.PI);
      prevPosRad  = posRad;
    }

    (function rafLoop() {
      const diff  = targetDeg - displayDeg;
      displayDeg += diff * 0.22;
      if (Math.abs(diff) > 0.05) {
        wheelGroup.setAttribute('transform', `rotate(${displayDeg.toFixed(2)}, 50, 50)`);
      }
      requestAnimationFrame(rafLoop);
    })();

    // ── Slider fill ─────────────────────────────────────────────────────
    function updateFill(val) {
      const pct = ((+val - (-50.0)) / 10.0) * 100;
      slider.style.background =
        `linear-gradient(to right, var(--accent) ${pct}%, rgba(0,196,255,0.15) ${pct}%)`;
    }

    // ── Velocity command ─────────────────────────────────────────────────
    let inFlight = false;
    let queued   = null;
    let localActiveUntil = 0;

    function isLocalActive() { return inFlight || Date.now() < localActiveUntil; }

    function setLocalVel(v) {
      const n = +v;
      localActiveUntil = Date.now() + 900;
      slider.value = n.toFixed(2);
      velDisplay.textContent = n.toFixed(2);
      updateFill(n);
    }

    async function sendVelocity(value) {
      const v = Math.max(-50.0, Math.min(50.0, +value));
      if (inFlight) { queued = v; return; }
      inFlight = true; queued = null;
      try {
        const resp = await fetch('/set_velocity', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
            'X-Requested-With': 'XMLHttpRequest',
          },
          body: `velocity=${v.toFixed(2)}`,
        });
        if (resp.ok) {
          const data = await resp.json();
          if (!isLocalActive() && data.target_velocity !== undefined) {
            setLocalVel(data.target_velocity);
          }
        }
      } catch (_) {}
      finally {
        inFlight = false;
        if (queued !== null && Math.abs(queued - v) > 0.005) {
          const next = queued; queued = null;
          void sendVelocity(next);
        }
      }
    }

    slider.addEventListener('input', (e) => {
      setLocalVel(e.target.value);
      void sendVelocity(e.target.value);
    });

    stopBtn.addEventListener('click', () => {
      setLocalVel(0.0);
      void sendVelocity(0.0);
    });

    // ── State poll ────────────────────────────────────────────────────────
    async function poll() {
      try {
        const resp = await fetch('/api/state', { cache: 'no-store' });
        if (!resp.ok) return;
        const data = await resp.json();

        const pos = +(data.joint?.position ?? 0);
        jointPos.textContent = pos.toFixed(3);
        applyWheelPosition(pos);

        if (!isLocalActive()) {
          const tv = +(data.motor?.target_velocity ?? 0);
          setLocalVel(tv);
        }
      } catch (_) {}
    }

    // Init
    updateFill(+slider.value);
    setInterval(poll, 100);
  </script>
</body>
</html>"""


    @app.get('/')
    def index():
        return render_template_string(
            page,
            logo_url=_LOGO_URL,
            motor=node.get_motor_snapshot(),
            joint=node.get_joint_snapshot(),
        )

    @app.get('/api/state')
    def api_state():
        return jsonify(get_dashboard_snapshot())

    @app.post('/set_gpio')
    def set_gpio():
        gpio_name = request.form.get('gpio_name', '')
        value_raw = request.form.get('value', '0')
        enabled = value_raw == '1'

        if gpio_name in node.get_state().keys():
            node.set_gpio(gpio_name, enabled)

        return redirect(url_for('index'))

    @app.post('/set_run')
    def set_run():
        run_enabled = bool(node.get_safety_snapshot()['run'])
        node.set_safety_input_bit(1, not run_enabled)
        return redirect(url_for('index'))

    @app.post('/pulse_safety')
    def pulse_safety():
        bit_raw = request.form.get('bit', '0')
        try:
            bit = int(bit_raw)
        except ValueError:
            bit = 0

        node.pulse_safety_input_bit(bit)
        return redirect(url_for('index'))

    @app.post('/pulse_drive_reset')
    def pulse_drive_reset():
        node.pulse_drive_reset()
        return redirect(url_for('index'))

    @app.post('/set_velocity')
    def set_velocity():
        velocity_raw = request.form.get('velocity', '0.0')
        try:
            velocity = float(velocity_raw)
        except ValueError:
            velocity = 0.0

        velocity = max(-50.0, min(50.0, velocity))
        node.set_target_velocity(velocity)

        if request.headers.get('X-Requested-With') == 'XMLHttpRequest':
            return jsonify(node.get_motor_snapshot())

        return redirect(url_for('index'))

    @app.post('/all_off')
    def all_off():
        node.set_all_off()
        return redirect(url_for('index'))

    return app


def main() -> None:
    rclpy.init()
    node = GpioGuiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = create_app(node)

    try:
        app.run(host='0.0.0.0', port=8080)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()

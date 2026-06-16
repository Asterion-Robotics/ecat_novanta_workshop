import math
import threading
from typing import Dict

from flask import Flask, jsonify, redirect, render_template_string, request, url_for

import rclpy
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class GpioGuiNode(Node):
    _PULSE_SECONDS = 0.2
    _DEFAULT_VELOCITY = 0.0

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
        }

    page = """
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="utf-8" />
      <title>enwbot Control Panel</title>
      <style>
        :root {
          --bg: #f5f6f0;
          --panel: #ffffff;
          --panel-soft: #fff3bf;
          --border: #d8dccc;
          --text: #1f2a1f;
          --muted: #667062;
          --green: #1f8f55;
          --red: #c44736;
          --shadow: 0 18px 50px rgba(31, 42, 31, 0.08);
        }
        * { box-sizing: border-box; }
        body {
          margin: 0;
          font-family: "Segoe UI", Helvetica, Arial, sans-serif;
          background:
            radial-gradient(circle at top left, rgba(242, 201, 76, 0.24), transparent 28%),
            linear-gradient(180deg, #f8f7ef 0%, var(--bg) 100%);
          color: var(--text);
        }
        code {
          background: rgba(31, 42, 31, 0.06);
          border-radius: 6px;
          padding: 2px 6px;
          font-size: 0.92em;
        }
        .page {
          max-width: 1080px;
          margin: 0 auto;
          padding: 32px 20px 48px;
        }
        .hero {
          display: flex;
          justify-content: space-between;
          align-items: end;
          gap: 16px;
          margin-bottom: 24px;
        }
        .hero h1 {
          margin: 0;
          font-size: 2.1rem;
          letter-spacing: -0.03em;
        }
        .hero p {
          margin: 8px 0 0;
          color: var(--muted);
        }
        .grid {
          display: grid;
          grid-template-columns: 1.1fr 0.9fr;
          gap: 20px;
        }
        .panel {
          background: var(--panel);
          border: 1px solid var(--border);
          border-radius: 20px;
          box-shadow: var(--shadow);
          padding: 22px;
        }
        .panel.safety {
          background: linear-gradient(180deg, #fff7cf 0%, var(--panel-soft) 100%);
          border-color: #e1cb6a;
        }
        .panel h2 {
          margin: 0 0 8px;
          font-size: 1.1rem;
        }
        .subtle {
          color: var(--muted);
          margin: 0 0 18px;
        }
        .toolbar {
          display: flex;
          justify-content: flex-end;
          margin-bottom: 18px;
        }
        .gpio-list {
          display: grid;
          gap: 12px;
        }
        .gpio-row {
          border: 1px solid var(--border);
          border-radius: 16px;
          padding: 14px;
          display: flex;
          justify-content: space-between;
          align-items: center;
          gap: 12px;
          background: rgba(255, 255, 255, 0.72);
        }
        .gpio-row form,
        .action-row form,
        .toolbar form {
          margin: 0;
        }
        .name {
          font-weight: 700;
        }
        .meta {
          color: var(--muted);
          font-size: 0.95rem;
          margin-top: 4px;
        }
        .badge {
          display: inline-flex;
          align-items: center;
          gap: 8px;
          padding: 7px 12px;
          border-radius: 999px;
          font-size: 0.92rem;
          font-weight: 700;
          background: #eef2eb;
        }
        .badge.on { color: var(--green); }
        .badge.off { color: var(--red); }
        .badge.run { background: rgba(31, 143, 85, 0.14); color: var(--green); }
        .badge.stop { background: rgba(196, 71, 54, 0.14); color: var(--red); }
        .led {
          width: 14px;
          height: 14px;
          border-radius: 50%;
          display: inline-block;
          box-shadow: inset 0 0 0 1px rgba(0, 0, 0, 0.08), 0 0 12px rgba(0, 0, 0, 0.08);
        }
        .led.green { background: var(--green); }
        .led.red { background: var(--red); }
        .actions,
        .status-grid {
          display: grid;
          gap: 12px;
        }
        .action-row,
        .status-card {
          background: rgba(255, 255, 255, 0.55);
          border: 1px solid rgba(130, 111, 25, 0.18);
          border-radius: 16px;
          padding: 14px;
        }
        .action-row {
          display: flex;
          justify-content: space-between;
          align-items: center;
          gap: 14px;
        }
        .action-copy strong {
          display: block;
          margin-bottom: 4px;
        }
        .button-row {
          display: flex;
          gap: 8px;
          flex-wrap: wrap;
          justify-content: flex-end;
        }
        button {
          border: 0;
          border-radius: 999px;
          padding: 10px 16px;
          font-weight: 700;
          cursor: pointer;
          color: #fff;
          background: #2f4f35;
          transition: transform 140ms ease;
        }
        button:hover { transform: translateY(-1px); }
        .button-secondary { background: #607065; }
        .button-warning { background: #c38a16; }
        .button-danger { background: #b44c3c; }
        .status-grid {
          grid-template-columns: repeat(2, minmax(0, 1fr));
          margin-top: 16px;
        }
        .status-card .value {
          font-size: 1.4rem;
          font-weight: 800;
          margin-top: 10px;
        }
        .footer-note {
          margin-top: 12px;
          color: var(--muted);
          font-size: 0.92rem;
        }
        .motor-box {
          margin-top: 20px;
          border: 1px solid var(--border);
          border-radius: 16px;
          padding: 14px;
          background: rgba(255, 255, 255, 0.72);
        }
        .motor-box h3 {
          margin: 0 0 6px;
          font-size: 1rem;
        }
        .motor-slider-row {
          margin-top: 10px;
          display: grid;
          gap: 10px;
        }
        .motor-slider-row input[type="range"] {
          width: 100%;
        }
        .motor-target {
          font-weight: 700;
          color: var(--text);
        }
        @media (max-width: 900px) {
          .grid { grid-template-columns: 1fr; }
          .hero { flex-direction: column; align-items: flex-start; }
          .action-row,
          .gpio-row { flex-direction: column; align-items: flex-start; }
          .button-row { justify-content: flex-start; }
          .status-grid { grid-template-columns: 1fr; }
        }
      </style>
    </head>
    <body>
      <div class="page">
        <div class="hero">
          <div>
            <h1>enwbot Control Panel</h1>
            <p>GPIO controls on the left, TwinSAFE controls and diagnostics on the right.</p>
          </div>
          <div class="toolbar">
            <form method="post" action="{{ url_for('all_off') }}">
              <button type="submit" class="button-danger">All OFF</button>
            </form>
          </div>
        </div>

        <div class="grid">
          <section class="panel">
            <h2>GPIO Outputs</h2>
            <p class="subtle">Safety tower and general digital outputs.</p>
            <div class="gpio-list">
              {% for gpio_name, value in state.items() %}
              <div class="gpio-row">
                <div>
                  <div class="name">{{ gpio_name }}</div>
                  <div class="meta">Current state</div>
                </div>
                <div class="button-row">
                  <span class="badge {{ 'on' if value > 0.5 else 'off' }}" data-gpio-badge="{{ gpio_name }}">{{ 'ON' if value > 0.5 else 'OFF' }}</span>
                  <form method="post" action="{{ url_for('set_gpio') }}">
                    <input type="hidden" name="gpio_name" value="{{ gpio_name }}"/>
                    <button type="submit" name="value" value="1">ON</button>
                    <button type="submit" name="value" value="0" class="button-secondary">OFF</button>
                  </form>
                </div>
              </div>
              {% endfor %}
            </div>

            <div class="motor-box">
              <h3>Motor Control</h3>
              <p class="subtle">Command velocity for <code>right_wheel_joint</code>.</p>
              <form method="post" action="{{ url_for('set_velocity') }}" class="motor-slider-row" id="velocity-form">
                <label for="velocity">Target velocity (<span class="motor-target" id="velocity-value">{{ '%.2f'|format(motor.target_velocity) }}</span> rad/s)</label>
                <input id="velocity" name="velocity" type="range" min="-5.0" max="5.0" step="0.05" value="{{ '%.2f'|format(motor.target_velocity) }}" />
                <div class="meta">Drag the slider to command velocity immediately.</div>
              </form>
              <div class="button-row" style="margin-top: 8px;">
                <form method="post" action="{{ url_for('set_velocity') }}" id="motor-stop-form">
                  <input type="hidden" name="velocity" value="0.0"/>
                  <button type="submit" class="button-danger">Motor Stop</button>
                </form>
              </div>
            </div>
          </section>

          <section class="panel safety">
            <h2>Safety Controls</h2>
            <p class="subtle">TwinSAFE input actions and live safety diagnostics.</p>

            <div class="actions">
              <div class="action-row">
                <div class="action-copy">
                  <strong>RUN</strong>
                  <div class="meta">Maintained switch on bit 1 of <code>safety_input_vars</code>.</div>
                </div>
                <div class="button-row">
                  <span class="badge {{ 'run' if safety.run else 'stop' }}" id="run-badge">{{ 'ON' if safety.run else 'OFF' }}</span>
                  <form method="post" action="{{ url_for('set_run') }}">
                    <button type="submit" id="run-button-label">Turn {{ 'OFF' if safety.run else 'ON' }}</button>
                  </form>
                </div>
              </div>

              <div class="action-row">
                <div class="action-copy">
                  <strong>Error Ack</strong>
                  <div class="meta">Short positive pulse on bit 0, ending with a falling edge.</div>
                </div>
                <form method="post" action="{{ url_for('pulse_safety') }}">
                  <input type="hidden" name="bit" value="0"/>
                  <button type="submit" class="button-warning">Pulse ACK</button>
                </form>
              </div>

              <div class="action-row">
                <div class="action-copy">
                  <strong>Restart</strong>
                  <div class="meta">Short positive pulse on bit 2, ending with a falling edge.</div>
                </div>
                <form method="post" action="{{ url_for('pulse_safety') }}">
                  <input type="hidden" name="bit" value="2"/>
                  <button type="submit" class="button-warning">Pulse Restart</button>
                </form>
              </div>

              <div class="action-row">
                <div class="action-copy">
                  <strong>Drive Reset</strong>
                  <div class="meta">Pulses <code>right_wheel_joint/reset_fault</code> through <code>drive_reset_controller</code>.</div>
                </div>
                <form method="post" action="{{ url_for('pulse_drive_reset') }}">
                  <button type="submit" class="button-danger">Pulse Drive Reset</button>
                </form>
              </div>
            </div>

            <div class="status-grid">
              <div class="status-card">
                <div class="name">ERROR</div>
                <div class="meta">Bit 0 of <code>safety_output_vars</code></div>
                <div class="value"><span class="led {{ 'red' if safety.error_led else 'green' }}" id="error-led"></span> <span id="error-text">{{ 'ERROR' if safety.error_led else 'OK' }}</span></div>
              </div>
              <div class="status-card">
                <div class="name">ESTOP</div>
                <div class="meta">Bit 1 of <code>safety_output_vars</code></div>
                <div class="value"><span class="led {{ 'green' if safety.estop_ok else 'red' }}" id="estop-led"></span> <span id="estop-text">{{ 'READY' if safety.estop_ok else 'STOPPED' }}</span></div>
              </div>
              <div class="status-card">
                <div class="name">Safety State</div>
                <div class="meta"><code>safety_logic_state</code></div>
                <div class="value" id="logic-state-text">{{ safety.logic_state_text }}</div>
              </div>
              <div class="status-card">
                <div class="name">Input / Output Vars</div>
                <div class="meta">Raw bytes for quick checks</div>
                <div class="value"><span id="input-vars-text">{{ safety.safety_input_vars }}</span> / <span id="output-vars-text">{{ safety.safety_output_vars }}</span></div>
              </div>
            </div>

            <div class="footer-note">RUN is maintained. ACK and RESTART pulse high briefly and return low automatically.</div>
          </section>
        </div>
      </div>
      <script>
        const gpioBadges = document.querySelectorAll('[data-gpio-badge]');
        const runBadge = document.getElementById('run-badge');
        const runButtonLabel = document.getElementById('run-button-label');
        const errorLed = document.getElementById('error-led');
        const errorText = document.getElementById('error-text');
        const estopLed = document.getElementById('estop-led');
        const estopText = document.getElementById('estop-text');
        const logicStateText = document.getElementById('logic-state-text');
        const inputVarsText = document.getElementById('input-vars-text');
        const outputVarsText = document.getElementById('output-vars-text');
        const velocityForm = document.getElementById('velocity-form');
        const motorStopForm = document.getElementById('motor-stop-form');
        const velocitySlider = document.getElementById('velocity');
        const velocityValue = document.getElementById('velocity-value');
        let velocityRequestInFlight = false;
        let queuedVelocity = null;
        let localVelocity = Number(velocitySlider.value || 0.0);
        let localVelocityActiveUntil = 0;

        function updateVelocityLabel(value) {
          const numeric = Number(value);
          velocityValue.textContent = Number.isFinite(numeric) ? numeric.toFixed(2) : '0.00';
        }

        function setLocalVelocity(value) {
          const numeric = Number(value);
          localVelocity = Number.isFinite(numeric) ? numeric : 0.0;
          localVelocityActiveUntil = Date.now() + 1000;
          velocitySlider.value = localVelocity.toFixed(2);
          updateVelocityLabel(localVelocity);
        }

        function isLocalVelocityActive() {
          return velocityRequestInFlight || Date.now() < localVelocityActiveUntil;
        }

        async function sendVelocityCommand(value) {
          const numeric = Number(value);
          const clamped = Math.max(-5.0, Math.min(5.0, Number.isFinite(numeric) ? numeric : 0.0));

          if (velocityRequestInFlight) {
            queuedVelocity = clamped;
            return;
          }

          velocityRequestInFlight = true;
          queuedVelocity = null;

          try {
            const body = new URLSearchParams({ velocity: clamped.toFixed(2) });
            const response = await fetch(velocityForm.action, {
              method: 'POST',
              headers: {
                'Content-Type': 'application/x-www-form-urlencoded;charset=UTF-8',
                'Accept': 'application/json',
                'X-Requested-With': 'XMLHttpRequest',
              },
              body: body.toString(),
            });

            if (!response.ok) {
              return;
            }

            const payload = await response.json();
            if (payload && payload.target_velocity !== undefined) {
              setLocalVelocity(payload.target_velocity);
            }
          } catch (_error) {
          } finally {
            velocityRequestInFlight = false;
            if (queuedVelocity !== null && Math.abs(queuedVelocity - clamped) > 0.001) {
              const nextVelocity = queuedVelocity;
              queuedVelocity = null;
              void sendVelocityCommand(nextVelocity);
            }
          }
        }

        velocitySlider.addEventListener('input', (event) => {
          setLocalVelocity(event.target.value);
          void sendVelocityCommand(event.target.value);
        });

        velocityForm.addEventListener('submit', (event) => {
          event.preventDefault();
          setLocalVelocity(velocitySlider.value);
          void sendVelocityCommand(velocitySlider.value);
        });

        motorStopForm.addEventListener('submit', (event) => {
          event.preventDefault();
          setLocalVelocity(0.0);
          void sendVelocityCommand(0.0);
        });

        function setBadgeState(element, active, activeClass, inactiveClass, activeText, inactiveText) {
          element.classList.remove(activeClass, inactiveClass);
          element.classList.add(active ? activeClass : inactiveClass);
          element.textContent = active ? activeText : inactiveText;
        }

        function setLedState(element, isGreen) {
          element.classList.remove('green', 'red');
          element.classList.add(isGreen ? 'green' : 'red');
        }

        async function refreshDashboard() {
          try {
            const response = await fetch('{{ url_for('api_state') }}', { cache: 'no-store' });
            if (!response.ok) {
              return;
            }
            const payload = await response.json();

            gpioBadges.forEach((badge) => {
              const gpioName = badge.dataset.gpioBadge;
              const value = Number(payload.gpio[gpioName] || 0);
              setBadgeState(badge, value > 0.5, 'on', 'off', 'ON', 'OFF');
            });

            const safety = payload.safety;
            setBadgeState(runBadge, Boolean(safety.run), 'run', 'stop', 'ON', 'OFF');
            runButtonLabel.textContent = `Turn ${safety.run ? 'OFF' : 'ON'}`;

            setLedState(errorLed, !Boolean(safety.error_led));
            errorText.textContent = safety.error_led ? 'ERROR' : 'OK';

            setLedState(estopLed, Boolean(safety.estop_ok));
            estopText.textContent = safety.estop_ok ? 'READY' : 'STOPPED';

            logicStateText.textContent = safety.logic_state_text;
            inputVarsText.textContent = safety.safety_input_vars;
            outputVarsText.textContent = safety.safety_output_vars;

            const motor = payload.motor;
            const targetVelocity = Number(motor.target_velocity || 0.0);
            if (!isLocalVelocityActive()) {
              velocitySlider.value = targetVelocity.toFixed(2);
              updateVelocityLabel(targetVelocity);
            }
          } catch (_error) {
          }
        }

        window.setInterval(refreshDashboard, 500);
      </script>
    </body>
    </html>
    """

    @app.get('/')
    def index():
        return render_template_string(
            page,
            state=node.get_state(),
            safety=node.get_safety_snapshot(),
            motor=node.get_motor_snapshot(),
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

        velocity = max(-5.0, min(5.0, velocity))
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

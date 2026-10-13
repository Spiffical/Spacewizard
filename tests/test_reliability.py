"""Safety regressions; all transports are fake or localhost pseudo-terminals."""
import importlib.util
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bridge import metrics
from bridge.config import load_config
from bridge.serial_input import parse_serial_line
from bridge.reliability import InputFreshness, RecoveringOSCClient, RecoveringSerial
from pythonosc.osc_message import OscMessage

CONFIG = load_config(str(ROOT / 'config/bridge.json')).config


def frame(z=0.5, gx=0):
    return dict(AX=0, AY=0, AZ=z, GX=gx, GY=0, GZ=0, YAW=0, PITCH=0, ROLL=0, IMP=0)


def line(**kwargs):
    return ' '.join(f'{k}={v}' for k, v in frame(**kwargs).items()) + '\n'


class MotionTests(unittest.TestCase):
    def test_health_files_expire_when_runtime_stops(self):
        spec = importlib.util.spec_from_file_location('health', ROOT / 'scripts/check_health.py')
        health = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(health)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'health'
            self.assertFalse(health.healthy(path, 100))
            path.touch()
            os.utime(path, (100, 100))
            self.assertTrue(health.healthy(path, 105))
            self.assertFalse(health.healthy(path, 111))

    def test_auto_detection_waits_safely_for_receiver(self):
        connection = Mock()
        with patch('bridge.reliability.find_arduino_port', side_effect=[None, '/new-receiver']), \
             patch('bridge.reliability.serial.Serial', return_value=connection) as factory, \
             patch('bridge.reliability.time.monotonic', side_effect=[1, 2.1]):
            port = RecoveringSerial(None)
            self.assertEqual(port.read(lambda _: line()), '')
            self.assertEqual(port.read(lambda _: line()), line())
        self.assertEqual(factory.call_args.kwargs['port'], '/new-receiver')

    def test_rest_does_not_reactivate_without_new_movement(self):
        state = metrics.ImpactState()
        pickups = []
        for t in range(1000, 10001, 10):
            with patch.object(metrics, 'get_current_time_ms', return_value=t):
                result = metrics.calculate_motion_metrics(frame(z=.5 if t < 1100 else .53), CONFIG, state)
            if result['pickup_triggered']:
                pickups.append(t)
        self.assertEqual(pickups, [1110])
        self.assertFalse(result['interaction_active'])
        for t in (10010, 10030):
            with patch.object(metrics, 'get_current_time_ms', return_value=t):
                result = metrics.calculate_motion_metrics(frame(z=.58), CONFIG, state)
        self.assertTrue(result['pickup_triggered'])

    def test_song_energy_winds_down_when_held_and_recovers(self):
        from bridge.osc import send_synth_control_messages
        state = metrics.ImpactState()
        def run(start, stop, gx):
            result = None
            for t in range(start, stop, 10):
                g = gx(t) if callable(gx) else gx
                with patch.object(metrics, 'get_current_time_ms', return_value=t):
                    result = metrics.calculate_motion_metrics(frame(gx=g), CONFIG, state)
            return result
        shaky = lambda t: 0.6 if t % 20 else 1.0  # hand-held: the lights stay on, the song winds down
        result = run(1000, 1500, 0)
        self.assertFalse(result['interaction_active'])
        result = run(1500, 1800, 1.0)  # small nudge: lights wake, song does not start
        self.assertTrue(result['interaction_active'])
        self.assertFalse(result['song_gate_active'])
        result = run(1800, 2300, 5.0)  # deliberate turn starts the song at full energy
        self.assertTrue(result['song_gate_active'])
        self.assertEqual(result['song_energy'], 1.0)
        result = run(2300, 3800, shaky)  # 1.5 s held: slowing down, still playing
        self.assertTrue(result['song_gate_active'])
        self.assertGreater(result['song_energy'], 0.4)
        self.assertLess(result['song_energy'], 0.8)
        result = run(3800, 4800, 5.0)  # moving again recovers without a restart
        self.assertTrue(result['song_gate_active'])
        self.assertGreater(result['song_energy'], 0.9)
        result = run(4800, 8500, shaky)  # held long enough: wound down and stopped
        self.assertTrue(result['interaction_active'])
        self.assertFalse(result['song_gate_active'])
        self.assertEqual(result['song_energy'], 0.0)
        result = run(8500, 8800, 5.0)  # the next deliberate move restarts at full energy
        self.assertTrue(result['song_gate_active'])
        self.assertEqual(result['song_energy'], 1.0)
        client = Mock()
        send_synth_control_messages(result, client)
        values = client.send_message.call_args[0][1]
        self.assertEqual((len(values), values[6], values[7], values[8]), (9, 1.0, 1.0, 1.0))
        send_synth_control_messages({'interaction_active': False, 'input_valid': False}, client)
        self.assertEqual(client.send_message.call_args[0][1][6:], [0.0, 0.0, 0.0])

    def test_parser_rejects_corruption(self):
        valid = line().strip()
        for bad in ('GY=0 GZ=0 IMP=0', valid.replace('AZ=0.5', 'AZ=nan'),
                    valid.replace('IMP=0', 'IMP=inf'), 'FMT=7 ' + valid,
                    valid + ' AX=2', valid + ' QW=1',
                    valid + ' QW=0 QX=0 QY=0 QZ=0', 'VALID=0 ' + valid,
                    'SEQ=1.5 ' + valid, valid.replace('AZ=0.5', 'AZ=8192')):
            with self.subTest(bad=bad):
                self.assertIsNone(parse_serial_line(bad))
        self.assertIsNotNone(parse_serial_line(valid))
        self.assertIsNotNone(parse_serial_line('FMT=2 SEQ=65535 VALID=1 ' + valid.replace('AZ=0.5','AZ=8192')))

    def test_duplicates_do_not_refresh_input(self):
        monitor = InputFreshness(.5)
        self.assertTrue(monitor.stale(1))
        self.assertTrue(monitor.accept({'SEQ': 65535}, 1))
        self.assertFalse(monitor.accept({'SEQ': 65535}, 1.49))
        self.assertTrue(monitor.stale(1.5))
        self.assertTrue(monitor.accept({'SEQ': 0}, 1.51))
        self.assertFalse(monitor.stale(1.52))

    def test_osc_failure_is_isolated_and_retried(self):
        bad = Mock()
        bad.send_message.side_effect = OSError('offline')
        good = Mock()
        with patch('bridge.reliability.SimpleUDPClient', side_effect=[bad, good]) as factory, \
             patch('bridge.reliability.time.monotonic', side_effect=[1, 1.1, 2.1]):
            client = RecoveringOSCClient('127.0.0.1', 9999)
            self.assertFalse(client.send_message('/x', 1))
            self.assertFalse(client.send_message('/x', 1))
            self.assertTrue(client.send_message('/x', 0))
        self.assertEqual(factory.call_count, 2)
        good.send_message.assert_called_once_with('/x', 0)

    def test_serial_startup_absence_and_reconnect(self):
        connection = Mock()
        with patch('bridge.reliability.serial.Serial', side_effect=[OSError('absent'), connection]) as factory, \
             patch('bridge.reliability.time.monotonic', side_effect=[1, 2.1, 2.2, 2.2]):
            port = RecoveringSerial('/mock')
            self.assertEqual(port.read(lambda _: line()), '')
            self.assertEqual(port.read(lambda _: line()), line())
            def fail(_):
                raise OSError('removed')
            self.assertEqual(port.read(fail), '')
            self.assertTrue(port.disconnected)
            self.assertIsNone(port.connection)
        self.assertEqual(factory.call_count, 2)


@unittest.skipUnless(hasattr(os, 'openpty'), 'requires pseudo-terminal support')
class BridgeIntegrationTests(unittest.TestCase):
    def test_real_process_loss_reconnect_invalid_frames_and_sigterm(self):
        # Local UDP collectors receive commands without lighting or audio hardware.
        receivers = []
        for _ in range(2):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('127.0.0.1', 0))
            sock.setblocking(False)
            receivers.append(sock)
        maestro, synth = receivers
        process = None
        masters = []
        slaves = []
        packets = []
        with tempfile.TemporaryDirectory() as directory:
            device = Path(directory) / 'receiver'
            log_path = Path(directory) / 'bridge.log'
            health_path = Path(directory) / 'bridge.health'
            def attach():
                master, slave = os.openpty()
                masters.append(master)
                slaves.append(slave)
                device.unlink(missing_ok=True)
                device.symlink_to(os.ttyname(slave))
                return master
            master = attach()
            def collect():
                for sock in receivers:
                    while True:
                        try:
                            data = sock.recv(65535)
                        except BlockingIOError:
                            break
                        msg = OscMessage(data)
                        packets.append((sock is synth, msg.address, msg.params))
            def run_for(seconds, text=None):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    if text:
                        os.write(master, text.encode())
                    collect()
                    time.sleep(.02)
                collect()
            def latest(address):
                return [p[2] for p in packets if p[1] == address][-1]
            try:
                with log_path.open('w') as log:
                    process = subprocess.Popen([
                        sys.executable, str(ROOT / 'scripts/run_bridge.py'),
                        str(device), '127.0.0.1', str(maestro.getsockname()[1]), '--synth',
                        '--synth-port', str(synth.getsockname()[1]), '--health-file', str(health_path),
                        '--config', str(ROOT / 'config/bridge.json')
                    ], stdout=log, stderr=subprocess.STDOUT)
                    run_for(.4)
                    self.assertEqual(latest('/global/brightness'), [0.0])
                    self.assertEqual(latest('/spacewizard/auduino')[6], 0.0)
                    run_for(.3, line())
                    run_for(.25, line(gx=2))
                    self.assertEqual(latest('/global/brightness'), [1.0])
                    run_for(.8)  # receiver stays connected, candle stops transmitting
                    self.assertEqual(latest('/global/brightness'), [0.0])
                    self.assertEqual(latest('/spacewizard/auduino')[6], 0.0)
                    packets.clear()
                    run_for(.6, 'GY=2 IMP=0\n')
                    self.assertTrue(all(p[2] == [0.0] for p in packets if p[1] == '/global/brightness'))
                    run_for(.25, line())
                    run_for(.25, line(gx=2))
                    self.assertEqual(latest('/global/brightness'), [1.0])
                    os.close(master); masters.remove(master)
                    os.close(slaves.pop(0))
                    device.unlink()
                    run_for(.3)
                    self.assertEqual(latest('/global/brightness'), [0.0])
                    master = attach()
                    run_for(1.3, line())
                    run_for(.25, line(gx=2))
                    self.assertEqual(latest('/global/brightness'), [1.0])
                    self.assertIsNone(process.poll())
                    self.assertTrue(health_path.exists())
                    process.send_signal(signal.SIGTERM)
                    process.wait(timeout=3)
                    collect()
                    self.assertEqual(process.returncode, 0, log_path.read_text())
                    self.assertEqual(latest('/global/brightness'), [0.0])
                    self.assertEqual(latest('/spacewizard/auduino')[0], 0.0)
            finally:
                if process is not None and process.poll() is None:
                    process.kill(); process.wait()
                for fd in masters + slaves:
                    os.close(fd)
                for sock in receivers:
                    sock.close()


if __name__ == '__main__':
    unittest.main()

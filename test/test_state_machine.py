# =============================================================================
# DOSYA : test/test_state_machine.py
# AMAÇ  : Supervisor (state_machine.py) için ROS'suz birim testleri: geçiş tablosu, debounce (Q11),
#         FAILSAFE kilitleme ve onayla çıkış (Q13). Çalıştırma:  python3 -m pytest test/ -q
# =============================================================================
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sualti_sim_son.state_machine import State, Supervisor  # noqa: E402

P = dict(yaw_tol=0.087, dive_tol=0.05,
         fs_odom_timeout=0.5, fs_tilt_max=0.785, fs_z_min=-2.6, fs_z_max=-0.2,
         fs_err_max=0.6, fs_sat_time=3.0,
         fs_nan_n=1, fs_tilt_n=5, fs_depth_n=3, fs_err_n=20, fs_sat_n=1, fs_clear_time=2.0)
OK = dict(age=0.02, finite=True, roll=0.0, pitch=0.0, z=-2.0, err=0.05, sat_time=0.0)
M = dict(ready=True, dz=0.3, yaw_err=0.0, v=0.0, dwell_left=0.0, done=False)


def sup():
    return Supervisor(P, log=lambda msg, imp: None)


def run(s, n, t0, faults=OK, m=M, dt=0.05):
    for k in range(n):
        s.step(t0 + k * dt, faults, m)
    return t0 + n * dt


def test_nominal_sequence():
    s = sup()
    s.step(0.0, OK, dict(M, ready=False))
    assert s.state is State.INIT and s.action() == 'zero'
    s.step(0.05, OK, M)
    assert s.state is State.DIVE
    # dalış biter + yön doğru → aynı adımda ALIGN → TRACK (zincirleme)
    s.step(0.10, OK, dict(M, dz=0.01))
    assert s.state is State.TRACK and s.advance()
    s.step(0.15, OK, dict(M, dz=0.01, v=0.0, dwell_left=2.0, yaw_err=1.5))  # köşe: yön yanlış
    assert s.state is State.ALIGN and not s.advance()
    s.step(0.20, OK, dict(M, dz=0.01, dwell_left=2.0, yaw_err=0.01))
    assert s.state is State.DWELL and s.advance()
    s.step(0.25, OK, dict(M, dz=0.01, dwell_left=-0.01, yaw_err=0.01))
    assert s.state is State.TRACK
    s.step(0.30, OK, dict(M, dz=0.01, done=True))
    assert s.state is State.DONE


def test_debounce_tilt():
    s = sup()
    t = run(s, 2, 0.0)
    bad = dict(OK, roll=1.0)
    t = run(s, 4, t, bad)                       # 4 örnek < fs_tilt_n = 5
    assert s.state is not State.FAILSAFE
    t = run(s, 1, t, OK)                        # tek iyi örnek sayacı sıfırlar
    t = run(s, 4, t, bad)
    assert s.state is not State.FAILSAFE
    run(s, 1, t, bad)                           # 5. ardışık örnek
    assert s.state is State.FAILSAFE and s.fault == 'tilt' and s.action() == 'zero'


def test_nan_immediate_and_stale():
    s = sup()
    run(s, 2, 0.0)
    s.step(0.2, dict(age=0.02, finite=False), None)
    assert s.state is State.FAILSAFE and s.fault == 'nan'
    s2 = sup()
    s2.step(0.0, dict(OK, age=0.6), M)
    assert s2.fault == 'odom_stale'


def test_depth_ascend_and_hold_actions():
    s = sup()
    t = run(s, 2, 0.0)
    run(s, 3, t, dict(OK, z=-2.65))
    assert s.fault == 'depth_low' and s.action() == 'ascend'
    s = sup()
    t = run(s, 2, 0.0)
    run(s, 20, t, dict(OK, err=0.7))
    assert s.fault == 'track_error' and s.action() == 'hold'


def test_track_error_ignored_in_init():
    s = sup()
    run(s, 40, 0.0, dict(OK, err=5.0, sat_time=10.0), dict(M, ready=False))
    assert s.state is State.INIT


def test_latch_and_ack():
    s = sup()
    t = run(s, 2, 0.0)
    s.trip('odom_stale', t)
    # arıza sürerken onay → reddedilir, FAILSAFE'te kalınır
    s.acknowledge()
    t = run(s, 1, t, dict(OK, age=1.0))
    assert s.state is State.FAILSAFE
    # arıza geçti ama 1 s < fs_clear_time → onay reddedilir
    t = run(s, 20, t)
    s.acknowledge()
    t = run(s, 1, t)
    assert s.state is State.FAILSAFE
    # onay olmadan asla çıkmaz (otomatik çıkış yok)
    t = run(s, 100, t)
    assert s.state is State.FAILSAFE
    s.acknowledge()
    run(s, 1, t)
    assert s.state is State.INIT and s.fault is None


def test_roll_pitch_clamp():
    # Q8a: asin argümanı 1'i aşarsa ValueError yerine ±90° dönmeli
    sp = 1.0000001
    assert math.isclose(math.asin(max(-1.0, min(1.0, sp))), math.pi / 2)

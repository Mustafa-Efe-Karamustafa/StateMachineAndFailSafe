# =============================================================================
# DOSYA : plot_run.py
# AMAÇ  : motion_node'un yazdığı CSV kaydından (~/auv_logs/auv_<görev>_<tarih>.csv) test grafiği çizer:
#         (1) yol: referans ve araç (başlangıç çerçevesi)  (2) derinlik z(t) ve güvenli aralık [z_min, z_max]
#         (3) durum-zaman grafiği (FAILSAFE kırmızı gölgeli)  (4) takip hatası ve doyma süresi, eşikleriyle.
# KULLANIM: ros2 run sualti_sim_son plot_run <kayıt.csv> -o T1.png --title "T1: arıza yok"
#           --time wall : yatay eksen duvar saati (köprü ölünce /clock durduğu için T2'de gerekli)
# =============================================================================
import argparse
import csv
import math

from .motion_node import DEFAULTS
from .state_machine import State

ORDER = [s.name for s in State]


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def main():
    ap = argparse.ArgumentParser(description='motion_node CSV kaydından test grafiği')
    ap.add_argument('csv')
    ap.add_argument('-o', '--out', default=None)
    ap.add_argument('--title', default='')
    ap.add_argument('--time', choices=('sim', 'wall'), default='sim')
    ap.add_argument('--z-min', type=float, default=DEFAULTS['fs_z_min'])
    ap.add_argument('--z-max', type=float, default=DEFAULTS['fs_z_max'])
    ap.add_argument('--err-max', type=float, default=DEFAULTS['fs_err_max'])
    ap.add_argument('--sat-time', type=float, default=DEFAULTS['fs_sat_time'])
    a = ap.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    with open(a.csv) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit('kayıt boş')
    key = 't' if a.time == 'sim' else 'wall'
    t0 = _f(rows[0][key])
    t = [_f(r[key]) - t0 for r in rows]
    st = [ORDER.index(r['state']) for r in rows]
    col = lambda k: [_f(r[k]) for r in rows]

    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    (a1, a2), (a3, a4) = ax
    a1.plot(col('rx'), col('ry'), 'r--', label='referans')
    a1.plot(col('xs'), col('ys'), 'b-', label='araç')
    a1.axis('equal')
    a1.set_xlabel('x [m] (başlangıç çerçevesi)')
    a1.set_ylabel('y [m]')
    a1.set_title('Yol')

    a2.plot(t, col('z'), 'b-', label='z (dünya)')
    a2.axhline(a.z_min, color='r', ls=':', label=f'z_min = {a.z_min}')
    a2.axhline(a.z_max, color='orange', ls=':', label=f'z_max = {a.z_max}')
    a2.axhline(-3.0, color='k', lw=2, label='zemin')
    a2.set_ylabel('z [m]')
    a2.set_title('Derinlik')

    a3.step(t, st, where='post', color='k')
    a3.set_yticks(range(len(ORDER)))
    a3.set_yticklabels(ORDER)
    a3.set_title('Durum-zaman')

    a4.plot(t, col('err'), 'b-', label='takip hatası ‖p_ref - p‖ [m]')
    a4.axhline(a.err_max, color='b', ls=':', label=f'e_max = {a.err_max}')
    a4.plot(t, col('sat_time'), 'm-', label='doyma süresi [s]')
    a4.axhline(a.sat_time, color='m', ls=':', label=f'T_sat = {a.sat_time}')
    a4.plot(t, [math.degrees(max(abs(_f(r['roll'])), abs(_f(r['pitch'])))) / 10 for r in rows], 'g-',
            lw=0.8, label='max(|φ|,|θ|) / 10 [°]')
    a4.set_title('Arıza girdileri')

    # FAILSAFE aralıklarını gölgele ve ilk girişi işaretle.
    fs = ORDER.index('FAILSAFE')
    spans, start = [], None
    for ti, si in zip(t, st):
        if si == fs and start is None:
            start = ti
        elif si != fs and start is not None:
            spans.append((start, ti))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    for x in (a2, a3, a4):
        for s0, s1 in spans:
            x.axvspan(s0, s1, color='r', alpha=0.12)
        x.set_xlabel(f"zaman [s] ({'simülasyon' if a.time == 'sim' else 'duvar'} saati)")
    for x in (a1, a2, a4):
        x.legend(fontsize=8)
    for x in ax.flat:
        x.grid(True)
    first = next((r for r in rows if r['state'] == 'FAILSAFE'), None)
    sub = f"FAILSAFE: sebep={first['fault']}, t={_f(first[key]) - t0:.2f} s" if first else 'FAILSAFE yok'
    fig.suptitle(f'{a.title}\n{sub}')
    fig.tight_layout()
    out = a.out or a.csv.rsplit('.', 1)[0] + '.png'
    fig.savefig(out, dpi=110)
    print(f'grafik: {out}  ({sub})')


if __name__ == '__main__':
    main()

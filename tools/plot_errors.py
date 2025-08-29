#!/usr/bin/env python3
import argparse, pandas as pd, matplotlib.pyplot as plt
import numpy as np

def first_idx_below(rho, yaw, rho_thr, yaw_thr):
    both = (rho <= rho_thr) & (np.abs(yaw) <= yaw_thr)
    idx = np.argmax(both) if both.any() else None
    if idx == 0 and not both.iloc[0]: return None
    return int(np.where(both)[0][0]) if both.any() else None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', help='CSV from record_odom.py')
    ap.add_argument('--out', default='errors.png')
    ap.add_argument('--rho_thr', type=float, default=0.05)
    ap.add_argument('--yaw_thr', type=float, default=0.10)
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    t = df['t_sec'].astype(float)
    rho = df['rho'].astype(float)
    yaw_err = df['yaw_err'].astype(float).abs()

    # Save separate plots
    plt.figure()
    plt.plot(t, rho, label='Distance error (m)')
    plt.axhline(args.rho_thr, linestyle='--', label=f'Spec {args.rho_thr} m')
    plt.xlabel('Time (s)'); plt.ylabel('Meters'); plt.title('Distance Error vs Time')
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig('distance_error.png')

    plt.figure()
    plt.plot(t, yaw_err, label='Orientation error (rad)')
    plt.axhline(args.yaw_thr, linestyle='--', label=f'Spec {args.yaw_thr} rad')
    plt.xlabel('Time (s)'); plt.ylabel('Radians'); plt.title('Orientation Error vs Time')
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig('orientation_error.png')

    # Combined
    plt.figure()
    plt.plot(t, rho, label='Distance error (m)')
    plt.plot(t, yaw_err, label='Orientation error (rad)')
    plt.axhline(args.rho_thr, linestyle='--', label=f'{args.rho_thr} m spec')
    plt.axhline(args.yaw_thr, linestyle='--', label=f'{args.yaw_thr} rad spec')
    plt.xlabel('Time (s)'); plt.title('Errors vs Time')
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(args.out)

    # Print summary
    final_rho = rho.iloc[-1]
    final_yaw = yaw_err.iloc[-1]
    idx = first_idx_below(rho, yaw_err, args.rho_thr, args.yaw_thr)
    if idx is not None:
        print(f'First time within both thresholds: t = {t.iloc[idx]:.2f} s')
    else:
        print('Never simultaneously within both thresholds in this run.')

    print(f'Final distance error: {final_rho:.4f} m ({final_rho*100:.1f} cm)')
    print(f'Final yaw error:     {final_yaw:.4f} rad')
    print('Wrote: distance_error.png, orientation_error.png, and', args.out)

if __name__ == '__main__':
    main()

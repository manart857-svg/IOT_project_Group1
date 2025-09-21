#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def ema(x: np.ndarray, alpha: float) -> np.ndarray:
    y = np.empty_like(x, dtype=float)
    if len(x) == 0:
        return y
    y[0] = x[0]
    for i in range(1, len(x)):
        xi = x[i]
        y[i] = alpha * xi + (1.0 - alpha) * y[i - 1] if np.isfinite(xi) else y[i - 1]
    return y

def lowpass_first_order(x: np.ndarray, beta: float) -> np.ndarray:
    y = np.empty_like(x, dtype=float)
    if len(x) == 0:
        return y
    y[0] = x[0]
    for i in range(1, len(x)):
        xi = x[i]
        y[i] = y[i - 1] + beta * (xi - y[i - 1]) if np.isfinite(xi) else y[i - 1]
    return y

def median_filter(x: np.ndarray, k: int) -> np.ndarray:
    if k % 2 == 0 or k < 1:
        raise ValueError("median window k must be odd and >= 1")
    s = pd.Series(x, dtype="float64")
    return s.rolling(window=k, center=True, min_periods=1).median().to_numpy()

def outlier_clean(x: np.ndarray, k: int = 11, thresh: float = 3.5) -> np.ndarray:
    if k % 2 == 0 or k < 3:
        raise ValueError("outlier window k must be odd and >= 3")
    s = pd.Series(x, dtype="float64")
    med = s.rolling(window=k, center=True, min_periods=1).median()
    mad = (s - med).abs().rolling(window=k, center=True, min_periods=1).median()
    mad_scaled = 1.4826 * mad
    mask_out = (s - med).abs() > (thresh * mad_scaled)
    y = s.copy()
    y[mask_out] = np.nan
    y = y.interpolate(method="linear", limit_direction="both")
    return y.to_numpy()

def kalman_1d(x: np.ndarray, q: float = 1e-3, r: float = 0.5, x0: float = None, p0: float = 1.0) -> np.ndarray:
    n = len(x)
    if n == 0:
        return np.array([])
    xhat = np.zeros(n)
    P = np.zeros(n)
    xhat[0] = x[0] if x0 is None else x0
    P[0] = p0
    for k in range(1, n):
        xhat_minus = xhat[k - 1]
        P_minus = P[k - 1] + q
        if np.isfinite(x[k]):
            K = P_minus / (P_minus + r)
            xhat[k] = xhat_minus + K * (x[k] - xhat_minus)
            P[k] = (1 - K) * P_minus
        else:
            xhat[k] = xhat_minus
            P[k] = P_minus
    return xhat


def abs_pct_error(mean_val: float, truth: float) -> float:
    """Absolute percentage error of a mean vs. truth."""
    if truth == 0 or not np.isfinite(truth) or not np.isfinite(mean_val):
        return np.nan
    return float(abs((mean_val - truth) / truth) * 100.0)


def plot_overall(df: pd.DataFrame, t_col: str, raw_col: str, filt_col: str, title: str, outpath: Path):
    plt.figure(figsize=(10, 4.5))
    t = df[t_col]
    plt.plot(t, df[raw_col], label="raw")
    plt.plot(t, df[filt_col], label=filt_col)
    plt.xlabel("time (ms)")
    plt.ylabel("distance (cm)")
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outpath, dpi=160)
    plt.close()

def plot_per_truth(df: pd.DataFrame, truth_val: float, t_col: str, raw_col: str, filt_cols: list, outpath: Path):
    g = df[df["true_distance_cm"] == truth_val]
    if g.empty:
        return
    plt.figure(figsize=(10, 4.5))
    t = g[t_col]
    plt.plot(t, g[raw_col], label="raw")
    for col in filt_cols:
        plt.plot(t, g[col], label=col)
    plt.xlabel("time (ms)")
    plt.ylabel("distance (cm)")
    plt.title(f"Truth = {truth_val} cm")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outpath, dpi=160)
    plt.close()


def main():
    ap = argparse.ArgumentParser(description="Filter distance CSV, plots, and per-filter comparison tables.")
    ap.add_argument("infile", help="CSV with t_ms, host_time_iso, true_distance_cm, measured_dist_cm")
    ap.add_argument("-o", "--outdir", default="dist_analysis_out")
    ap.add_argument("--ema-alpha", type=float, default=0.2)
    ap.add_argument("--lp-beta", type=float, default=0.25)
    ap.add_argument("--median-k", type=int, default=5)
    ap.add_argument("--outlier-k", type=int, default=11)
    ap.add_argument("--outlier-thresh", type=float, default=3.5)
    ap.add_argument("--kalman-q", type=float, default=1e-3)
    ap.add_argument("--kalman-r", type=float, default=0.5)
    args = ap.parse_args()

    outdir = Path(args.outdir)
    plots_dir = outdir / "plots"
    truth_dir = plots_dir / "per_truth"
    tables_dir = outdir / "tables"
    by_filter_dir = outdir / "by_filter"
    for d in [outdir, plots_dir, truth_dir, tables_dir, by_filter_dir]:
        d.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.infile)
    df["measured_dist_cm"] = pd.to_numeric(df.get("measured_dist_cm"), errors="coerce")
    df["true_distance_cm"] = pd.to_numeric(df.get("true_distance_cm"), errors="coerce")

    t = pd.to_numeric(df.get("t_ms"), errors="coerce")
    if t.isna().all():
        raise SystemExit("t_ms column is missing or non-numeric.")
    df["t_rel_ms_global"] = t - t.iloc[0]
    df["t_rel_ms_truth"] = np.nan
    truths_all = sorted(df["true_distance_cm"].dropna().unique().tolist())
    for tv in truths_all:
        idx = df.index[df["true_distance_cm"] == tv]
        if len(idx) > 0:
            t0 = t.loc[idx].iloc[0]
            df.loc[idx, "t_rel_ms_truth"] = t.loc[idx] - t0

    x = df["measured_dist_cm"].to_numpy(dtype=float)
    df["dist_ema_cm"] = ema(x, alpha=args.ema_alpha)
    df["dist_lp_cm"] = lowpass_first_order(x, beta=args.lp_beta)
    df["dist_median_cm"] = median_filter(x, k=args.median_k)
    df["dist_outlier_cm"] = outlier_clean(x, k=args.outlier_k, thresh=args.outlier_thresh)
    df["dist_kalman_cm"] = kalman_1d(x, q=args.kalman_q, r=args.kalman_r)

    agg_rows = []
    for tv in truths_all:
        g = df[df["true_distance_cm"] == tv]["measured_dist_cm"].dropna()
        if len(g) > 0:
            agg_rows.append((tv, float(g.mean())))
    if len(agg_rows) >= 2:
        truths_vec = np.array([r[0] for r in agg_rows], dtype=float)
        raw_means = np.array([r[1] for r in agg_rows], dtype=float)
        A = np.vstack([raw_means, np.ones_like(raw_means)]).T
        a, b = np.linalg.lstsq(A, truths_vec, rcond=None)[0]
        print(f"[calibration] y_true ≈ {a:.6f} * y_measured + {b:.6f}")
    else:
        a, b = 1.0, 0.0
        print("[calibration] Skipped (need >=2 distances).")

    per_map = {
        "raw": "measured_dist_cm",
        "ema": "dist_ema_cm",
        "lp": "dist_lp_cm",
        "median": "dist_median_cm",
        "outlier": "dist_outlier_cm",
        "kalman": "dist_kalman_cm",
    }
    for label, col in per_map.items():
        df[f"{col}_cal"] = a * pd.to_numeric(df[col], errors="coerce") + b

    df.to_csv(outdir / "filtered.csv", index=False)

    base_cols = ["t_ms", "t_rel_ms_global", "t_rel_ms_truth", "host_time_iso", "true_distance_cm", "measured_dist_cm"]
    for name, col in per_map.items():
        df[base_cols + [col]].to_csv(by_filter_dir / f"{name}.csv", index=False)

    for col in ["dist_ema_cm", "dist_lp_cm", "dist_median_cm", "dist_outlier_cm", "dist_kalman_cm"]:
        plot_overall(df, "t_rel_ms_global", "measured_dist_cm", col,
                     f"Raw vs {col} (global t=0)", plots_dir / f"overall_{col}.png")
    for tv in truths_all:
        plot_per_truth(df, tv, "t_rel_ms_truth", "measured_dist_cm",
                       ["dist_ema_cm", "dist_lp_cm", "dist_median_cm", "dist_outlier_cm", "dist_kalman_cm"],
                       truth_dir / f"truth_{tv:.1f}cm.png")

    def build_compare_table(raw_col: str, filt_col: str, suffix: str):
        rows = []
        for tv in truths_all:
            g = df[df["true_distance_cm"] == tv]
            raw_mean = float(pd.to_numeric(g[raw_col], errors="coerce").dropna().mean()) if not g.empty else np.nan
            filt_mean = float(pd.to_numeric(g[filt_col], errors="coerce").dropna().mean()) if not g.empty else np.nan
            raw_pct = abs_pct_error(raw_mean, float(tv))
            filt_pct = abs_pct_error(filt_mean, float(tv))
            rows.append({
                "true_distance_cm": float(tv),
                "raw_mean_cm": raw_mean,
                "raw_pct_error_of_mean": raw_pct,
                f"{filt_col}_mean_cm".replace("_cm_cal","").replace("_cm",""): filt_mean,  # column label will be cleaner
                f"{filt_col}_pct_error_of_mean".replace("_cm_cal","").replace("_cm",""): filt_pct
            })
        tbl = pd.DataFrame(rows).sort_values("true_distance_cm")
        return tbl

    for label, col in per_map.items():
        if label == "raw":
            tbl = build_compare_table("measured_dist_cm", "measured_dist_cm", "")
        else:
            tbl = build_compare_table("measured_dist_cm", col, "")
        tbl.to_csv(tables_dir / f"mean_compare_{label}.csv", index=False)

    for label, col in per_map.items():
        raw_cal = "measured_dist_cm_cal"
        filt_cal = f"{col}_cal"
        if label == "raw":
            tbl_cal = build_compare_table(raw_cal, raw_cal, "_cal")
        else:
            tbl_cal = build_compare_table(raw_cal, filt_cal, "_cal")
        tbl_cal.to_csv(tables_dir / f"mean_compare_{label}_cal.csv", index=False)

    print("\nDone. Results saved in", outdir)


if __name__ == "__main__":
    main()

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import signal, stats
from scipy.spatial.transform import Rotation
import os
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

plt.style.use('seaborn-darkgrid')
sns.set_palette("husl")

class IMUFilterAnalyzer:    
    def __init__(self, base_output_dir="filter_analysis_results"):
        self.activity_map = {
            1: "Walking Upstairs",
            2: "Walking Downstairs", 
            3: "Walking",
            4: "Sitting",
            5: "Standing"
        }
        self.base_output_dir = Path(base_output_dir)
        self.base_output_dir.mkdir(exist_ok=True)
        self.filter_types = ['ema', 'median', 'lp', 'outlier', 'madgwick']
        self.sensor_types = ['accelerometer', 'gyroscope']
        self.experiments = []
        self.all_metrics = {}
    
    def _activity_dirname(self, aid: int) -> str:
        name = self.activity_map.get(aid, f"activity_{aid}")
        safe = name.lower().replace(" ", "_")
        return f"{aid}_{safe}"


    def _present_activities(self, exp_data):
        return [
            (aid, self.activity_map[aid])
            for aid in self.activity_map.keys()
            if (exp_data['activity'] == aid).any()
        ]

    def _estimate_fs(self, df):
        if 'time_ms' in df.columns:
            t = df['time_ms'].values
            if len(t) > 1:
                dt = np.median(np.diff(t)) / 1000.0  # seconds
                if dt > 0:
                    return 1.0 / dt
        return 50.0


    def load_experiments(self, file_paths):
        print(f"Loading {len(file_paths)} experiment files...")
        for i, path in enumerate(file_paths, 1):
            try:
                df = pd.read_csv(path)
                activity_id = i
                activity_name = self.activity_map.get(activity_id, f"Activity_{activity_id}")
                self.experiments.append({
                    'name': f"Experiment_{i}",
                    'data': df,
                    'path': path,
                    'activity_id': activity_id,
                    'activity_name': activity_name
                })
                print(f"  ✓ Loaded {path} ({len(df)} samples)  →  {activity_id}: {activity_name}")
            except Exception as e:
                print(f"  ✗ Error loading {path}: {e}")

        
    def calculate_metrics(self, raw_data, filtered_data, sensor_type='accelerometer'):
        metrics = {}
        
        signal_power = np.mean(filtered_data**2)
        noise = raw_data - filtered_data
        noise_power = np.mean(noise**2)
        metrics['snr_db'] = 10 * np.log10(signal_power / (noise_power + 1e-10))
        
        metrics['mse'] = np.mean((raw_data - filtered_data)**2)
        metrics['rmse'] = np.sqrt(metrics['mse'])
        
        metrics['nrmse'] = metrics['rmse'] / (np.max(raw_data) - np.min(raw_data) + 1e-10)
        
        metrics['correlation'] = np.corrcoef(raw_data, filtered_data)[0, 1]
        
        diff_raw = np.diff(raw_data)
        diff_filtered = np.diff(filtered_data)
        metrics['smoothness_improvement'] = np.var(diff_raw) / (np.var(diff_filtered) + 1e-10)
        
        correlation = np.correlate(raw_data, filtered_data, mode='same')
        lag = len(correlation)//2 - np.argmax(correlation)
        metrics['lag_samples'] = lag
        
        fft_raw = np.fft.fft(raw_data)
        fft_filtered = np.fft.fft(filtered_data)
        metrics['freq_distortion'] = np.mean(np.abs(fft_raw - fft_filtered))
        
        peaks_raw = signal.find_peaks(raw_data)[0]
        peaks_filtered = signal.find_peaks(filtered_data)[0]
        if len(peaks_raw) > 0:
            metrics['peak_preservation_ratio'] = len(peaks_filtered) / len(peaks_raw)
        else:
            metrics['peak_preservation_ratio'] = 1.0
            
        metrics['dynamic_range_ratio'] = (np.max(filtered_data) - np.min(filtered_data)) / \
                                         (np.max(raw_data) - np.min(raw_data) + 1e-10)
        
        return metrics
    
    def analyze_filter(self, filter_type, exp_data, exp_name, output_dir, activity_id_filter=None):
        print(f"    Analyzing {filter_type} filter...")

        if filter_type == 'madgwick':
            return self.analyze_madgwick_filter(exp_data, exp_name, output_dir)

        activity_metrics = {}

        if activity_id_filter is not None and (exp_data['activity'] == activity_id_filter).any():
            act_name = self.activity_map.get(activity_id_filter, f"Activity_{activity_id_filter}")
            activity_metrics[act_name] = {'accelerometer': {}, 'gyroscope': {}}
            act = exp_data[exp_data['activity'] == activity_id_filter]

            for axis in ['x', 'y', 'z']:
                ra, fa = f'a{axis}', f'a{axis}_{filter_type}'
                rg, fg = f'g{axis}', f'g{axis}_{filter_type}'
                if ra in act.columns and fa in act.columns:
                    activity_metrics[act_name]['accelerometer'][axis] = self.calculate_metrics(
                        act[ra].values, act[fa].values, 'accelerometer'
                    )
                if rg in act.columns and fg in act.columns:
                    activity_metrics[act_name]['gyroscope'][axis] = self.calculate_metrics(
                        act[rg].values, act[fg].values, 'gyroscope'
                    )

        self.create_filter_visualizations(
            exp_data, filter_type, exp_name, output_dir, activity_id_filter=activity_id_filter
        )

        self.save_metrics(activity_metrics, output_dir / f'{exp_name}_metrics.csv')
        return activity_metrics

    
    def analyze_madgwick_filter(self, exp_data, exp_name, output_dir):
        print(f"      Analyzing Madgwick filter (quaternion fusion)...")
        
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle(f'Madgwick Filter Analysis - {exp_name}', fontsize=14, fontweight='bold')
        
        metrics = {}
        
        for activity_id in self.activity_map.keys():
            activity_data = exp_data[exp_data['activity'] == activity_id]
            if len(activity_data) == 0:
                continue
                
            activity_name = self.activity_map[activity_id]
            
            if all(col in activity_data.columns for col in ['q0', 'q1', 'q2', 'q3']):
                q = activity_data[['q0', 'q1', 'q2', 'q3']].values
                
                euler_angles = []
                for quat in q:
                    rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
                    euler = rot.as_euler('xyz', degrees=True)
                    euler_angles.append(euler)
                euler_angles = np.array(euler_angles)
                
                if activity_id <= 3:  
                    row = activity_id - 1
                    col = 0
                else:  
                    row = activity_id - 4
                    col = 1
                    
                if row < 3 and col < 2:
                    ax = axes[row, col]
                    time = activity_data['time_ms'].values / 1000.0
                    ax.plot(time, euler_angles[:, 0], label='Roll', alpha=0.7)
                    ax.plot(time, euler_angles[:, 1], label='Pitch', alpha=0.7)
                    ax.plot(time, euler_angles[:, 2], label='Yaw', alpha=0.7)
                    ax.set_title(f'{activity_name}')
                    ax.set_xlabel('Time (s)')
                    ax.set_ylabel('Angle (degrees)')
                    ax.legend()
                    ax.grid(True, alpha=0.3)
                
                metrics[activity_name] = {
                    'quaternion_norm_mean': np.mean(np.linalg.norm(q, axis=1)),
                    'quaternion_norm_std': np.std(np.linalg.norm(q, axis=1)),
                    'euler_roll_std': np.std(euler_angles[:, 0]),
                    'euler_pitch_std': np.std(euler_angles[:, 1]),
                    'euler_yaw_std': np.std(euler_angles[:, 2])
                }
        
        plt.tight_layout()
        plt.savefig(output_dir / f'{exp_name}_madgwick_analysis.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        return metrics
    
    def create_filter_visualizations(self, exp_data, filter_type, exp_name, output_dir, activity_id_filter=None):
        self.plot_time_domain_comparison(exp_data, filter_type, exp_name, output_dir, activity_id_filter)
        self.plot_frequency_analysis(exp_data, filter_type, exp_name, output_dir, activity_id_filter)
        self.plot_performance_heatmap(exp_data, filter_type, exp_name, output_dir, activity_id_filter)
        self.plot_distribution_comparison(exp_data, filter_type, exp_name, output_dir, activity_id_filter)

    
    def plot_time_domain_comparison(self, exp_data, filter_type, exp_name, output_dir, activity_id_filter=None):
        if activity_id_filter is None:
            present_ids = [aid for aid in self.activity_map if (exp_data['activity'] == aid).any()]
            if not present_ids: return
            aid = present_ids[0]
        else:
            aid = activity_id_filter
            if not (exp_data['activity'] == aid).any(): return

        activity_name = self.activity_map.get(aid, f"Activity_{aid}")
        act = exp_data[exp_data['activity'] == aid]
        if act.empty: return
        N = min(len(act), 800); act = act.head(N)

        fig, axes = plt.subplots(3, 2, figsize=(15, 10))
        fig.suptitle(f'Time Domain - {filter_type.upper()} - {exp_name} - {activity_name}', fontsize=14, fontweight='bold')

        if 'time_ms' in act.columns and len(act['time_ms']) > 1:
            time = act['time_ms'].values / 1000.0; xlab = 'Time (s)'
        else:
            time = np.arange(len(act)); xlab = 'Sample'

        for i, axis in enumerate(['x', 'y', 'z']):
            ax = axes[i, 0]
            ra, fa = f'a{axis}', f'a{axis}_{filter_type}'
            if ra in act.columns and fa in act.columns:
                ax.plot(time, act[ra].values, alpha=0.5, label='Raw', linewidth=0.8)
                ax.plot(time, act[fa].values, label='Filtered', linewidth=1.2)
                ax.set_title(f'Accelerometer {axis.upper()}'); ax.set_ylabel('Acceleration (m/s²)')
                ax.legend(); ax.grid(True, alpha=0.3)

        for i, axis in enumerate(['x', 'y', 'z']):
            ax = axes[i, 1]
            rg, fg = f'g{axis}', f'g{axis}_{filter_type}'
            if rg in act.columns and fg in act.columns:
                ax.plot(time, act[rg].values, alpha=0.5, label='Raw', linewidth=0.8)
                ax.plot(time, act[fg].values, label='Filtered', linewidth=1.2)
                ax.set_title(f'Gyroscope {axis.upper()}'); ax.set_ylabel('Angular velocity (°/s)')
                ax.legend(); ax.grid(True, alpha=0.3)

        axes[2, 0].set_xlabel(xlab); axes[2, 1].set_xlabel(xlab)
        plt.tight_layout()
        plt.savefig(output_dir / f'{exp_name}_time_domain.png', dpi=150, bbox_inches='tight')
        plt.close()


    
    def plot_frequency_analysis(self, exp_data, filter_type, exp_name, output_dir, activity_id_filter=None):
        if activity_id_filter is None:
            present_ids = [aid for aid in self.activity_map if (exp_data['activity'] == aid).any()]
            if not present_ids: return
            aid = present_ids[0]
        else:
            aid = activity_id_filter
            if not (exp_data['activity'] == aid).any(): return

        activity_name = self.activity_map.get(aid, f"Activity_{aid}")
        act = exp_data[exp_data['activity'] == aid]
        if act.empty: return
        fs = self._estimate_fs(act)

        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        fig.suptitle(f'Frequency Domain - {filter_type.upper()} - {exp_name} - {activity_name}', fontsize=14, fontweight='bold')

        n = len(act)
        nperseg = max(64, min(1024, 2 ** int(np.floor(np.log2(max(64, n // 4))))))

        for i, axis in enumerate(['x', 'y', 'z']):
            ax = axes[0, i]
            ra, fa = f'a{axis}', f'a{axis}_{filter_type}'
            if ra in act.columns and fa in act.columns:
                raw = act[ra].values; fil = act[fa].values
                try:
                    f, psd_raw = signal.periodogram(raw, fs=fs, nperseg=min(nperseg, len(raw)))
                    _, psd_fil = signal.periodogram(fil, fs=fs, nperseg=min(nperseg, len(fil)))
                    ax.semilogy(f, psd_raw, alpha=0.5, label='Raw', linewidth=0.8)
                    ax.semilogy(f, psd_fil, label='Filtered', linewidth=1.2)
                    ax.set_title(f'Accelerometer {axis.upper()}'); ax.set_xlabel('Hz'); ax.set_ylabel('PSD')
                    ax.legend(); ax.grid(True, alpha=0.3); ax.set_xlim([0, fs/2.0])
                except Exception:
                    pass

        for i, axis in enumerate(['x', 'y', 'z']):
            ax = axes[1, i]
            rg, fg = f'g{axis}', f'g{axis}_{filter_type}'
            if rg in act.columns and fg in act.columns:
                raw = act[rg].values; fil = act[fg].values
                try:
                    f, psd_raw = signal.periodogram(raw, fs=fs, nperseg=min(nperseg, len(raw)))
                    _, psd_fil = signal.periodogram(fil, fs=fs, nperseg=min(nperseg, len(fil)))
                    ax.semilogy(f, psd_raw, alpha=0.5, label='Raw', linewidth=0.8)
                    ax.semilogy(f, psd_fil, label='Filtered', linewidth=1.2)
                    ax.set_title(f'Gyroscope {axis.upper()}'); ax.set_xlabel('Hz'); ax.set_ylabel('PSD')
                    ax.legend(); ax.grid(True, alpha=0.3); ax.set_xlim([0, fs/2.0])
                except Exception:
                    pass

        plt.tight_layout()
        plt.savefig(output_dir / f'{exp_name}_frequency.png', dpi=150, bbox_inches='tight')
        plt.close()

    
    def plot_performance_heatmap(self, exp_data, filter_type, exp_name, output_dir, activity_id_filter=None):
        metrics_matrix = []
        xlabels = None
        ylabels = []

        activities = []
        if activity_id_filter is not None:
            if (exp_data['activity'] == activity_id_filter).any():
                activities = [(activity_id_filter, self.activity_map.get(activity_id_filter, f"Activity_{activity_id_filter}"))]
        else:
            activities = [(aid, self.activity_map[aid]) for aid in self.activity_map if (exp_data['activity'] == aid).any()]

        for aid, aname in activities:
            activity_data = exp_data[exp_data['activity'] == aid]
            if activity_data.empty: continue

            row_metrics = []
            local_xlabels = []
            for sensor in ['a', 'g']:
                for axis in ['x', 'y', 'z']:
                    raw_col = f'{sensor}{axis}'
                    filt_col = f'{raw_col}_{filter_type}'
                    if raw_col in activity_data.columns and filt_col in activity_data.columns:
                        raw = activity_data[raw_col].values
                        fil = activity_data[filt_col].values
                        signal_power = np.mean(fil**2)
                        noise_power = np.mean((raw - fil)**2)
                        snr = 10 * np.log10(signal_power / (noise_power + 1e-10))
                        row_metrics.append(snr)
                        if xlabels is None:
                            sensor_name = 'Acc' if sensor == 'a' else 'Gyro'
                            local_xlabels.append(f'{sensor_name}-{axis.upper()}')

            if row_metrics:
                metrics_matrix.append(row_metrics)
                ylabels.append(aname)
                if xlabels is None:
                    xlabels = local_xlabels

        if not metrics_matrix:
            return

        fig, ax = plt.subplots(figsize=(10, max(3, 1.2 * len(ylabels))))
        im = ax.imshow(metrics_matrix, cmap='RdYlGn', aspect='auto')

        ax.set_xticks(np.arange(len(xlabels)))
        ax.set_xticklabels(xlabels, rotation=45, ha="right")
        ax.set_yticks(np.arange(len(ylabels)))
        ax.set_yticklabels(ylabels)

        cbar = plt.colorbar(im, ax=ax); cbar.set_label('SNR (dB)', rotation=270, labelpad=20)

        for i in range(len(metrics_matrix)):
            for j in range(len(metrics_matrix[i])):
                ax.text(j, i, f'{metrics_matrix[i][j]:.1f}', ha="center", va="center", color="black", fontsize=9)

        ax.set_title(f'Filter Performance Heatmap - {filter_type.upper()} - {exp_name}')
        plt.tight_layout()
        plt.savefig(output_dir / f'{exp_name}_performance_heatmap.png', dpi=150, bbox_inches='tight')
        plt.close()


    def plot_distribution_comparison(self, exp_data, filter_type, exp_name, output_dir, activity_id_filter=None):
        if activity_id_filter is None:
            present_ids = [aid for aid in self.activity_map if (exp_data['activity'] == aid).any()]
            if not present_ids: return
            aid = present_ids[0]
        else:
            aid = activity_id_filter
            if not (exp_data['activity'] == aid).any(): return

        activity_name = self.activity_map.get(aid, f"Activity_{aid}")
        act = exp_data[exp_data['activity'] == aid]
        if act.empty: return

        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        fig.suptitle(f'Signal Distribution - {filter_type.upper()} - {exp_name} - {activity_name}', fontsize=14, fontweight='bold')

        for i, axis in enumerate(['x', 'y', 'z']):
            ax = axes[0, i]
            ra, fa = f'a{axis}', f'a{axis}_{filter_type}'
            if ra in act.columns and fa in act.columns:
                raw = act[ra].values; fil = act[fa].values
                ax.hist(raw, bins=50, alpha=0.5, label='Raw', density=True)
                ax.hist(fil, bins=50, alpha=0.5, label='Filtered', density=True)
                ax.set_title(f'Accelerometer {axis.upper()}'); ax.set_xlabel('Value'); ax.set_ylabel('Density'); ax.legend()
                ax.text(0.02, 0.98, f'Raw: μ={np.mean(raw):.2f}, σ={np.std(raw):.2f}\n'
                                    f'Filt: μ={np.mean(fil):.2f}, σ={np.std(fil):.2f}',
                        transform=ax.transAxes, fontsize=9, va='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        for i, axis in enumerate(['x', 'y', 'z']):
            ax = axes[1, i]
            rg, fg = f'g{axis}', f'g{axis}_{filter_type}'
            if rg in act.columns and fg in act.columns:
                raw = act[rg].values; fil = act[fg].values
                ax.hist(raw, bins=50, alpha=0.5, label='Raw', density=True)
                ax.hist(fil, bins=50, alpha=0.5, label='Filtered', density=True)
                ax.set_title(f'Gyroscope {axis.upper()}'); ax.set_xlabel('Value'); ax.set_ylabel('Density'); ax.legend()
                ax.text(0.02, 0.98, f'Raw: μ={np.mean(raw):.2f}, σ={np.std(raw):.2f}\n'
                                    f'Filt: μ={np.mean(fil):.2f}, σ={np.std(fil):.2f}',
                        transform=ax.transAxes, fontsize=9, va='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        plt.savefig(output_dir / f'{exp_name}_distribution.png', dpi=150, bbox_inches='tight')
        plt.close()


    
    def save_metrics(self, metrics, filepath):
        rows = []
        for activity, activity_metrics in metrics.items():
            for sensor, sensor_metrics in activity_metrics.items():
                for axis, axis_metrics in sensor_metrics.items():
                    row = {
                        'activity': activity,
                        'sensor': sensor,
                        'axis': axis,
                        **axis_metrics
                    }
                    rows.append(row)
        
        if rows:
            df = pd.DataFrame(rows)
            df.to_csv(filepath, index=False)
            print(f"      Saved metrics to {filepath}")
    
    def compare_filters(self):
        print("\nGenerating cross-filter comparison analysis...")
        
        comparison_dir = self.base_output_dir / 'filter_comparison'
        comparison_dir.mkdir(exist_ok=True)
        
        self.plot_overall_comparison(comparison_dir)
        
        self.plot_activity_comparison(comparison_dir)
        
        self.analyze_computational_complexity(comparison_dir)
        
        self.generate_summary_report(comparison_dir)
    
    def plot_overall_comparison(self, output_dir):
        pass
    
    def plot_activity_comparison(self, output_dir):
        pass
    
    def analyze_computational_complexity(self, output_dir):
        complexity_data = {
            'ema': {'complexity': 'O(n)', 'memory': 'O(1)', 'latency': 'Very Low'},
            'median': {'complexity': 'O(n·w·log(w))', 'memory': 'O(w)', 'latency': 'Medium'},
            'lp': {'complexity': 'O(n)', 'memory': 'O(1)', 'latency': 'Low'},
            'outlier': {'complexity': 'O(n·w)', 'memory': 'O(w)', 'latency': 'Medium-High'},
            'madgwick': {'complexity': 'O(n)', 'memory': 'O(1)', 'latency': 'Low-Medium'}
        }
        
        df = pd.DataFrame(complexity_data).T
        df.to_csv(output_dir / 'computational_complexity.csv')
        
        print(f"  Saved computational complexity analysis to {output_dir}")
    
    def generate_summary_report(self, output_dir):
        report_path = output_dir / 'summary_report.txt'
        
        with open(report_path, 'w') as f:
            f.write("=" * 80 + "\n")
            f.write("IMU FILTER ANALYSIS SUMMARY REPORT\n")
            f.write("=" * 80 + "\n\n")
            
            f.write("1. FILTER TYPES ANALYZED\n")
            f.write("-" * 40 + "\n")
            for filter_type in self.filter_types:
                f.write(f"  • {filter_type.upper()}\n")
            
            f.write("\n2. EXPERIMENTS PROCESSED\n")
            f.write("-" * 40 + "\n")
            for exp in self.experiments:
                f.write(f"  • {exp['name']}: {len(exp['data'])} samples\n")
            
            f.write("\n3. ACTIVITY CLASSES\n")
            f.write("-" * 40 + "\n")
            for id, name in self.activity_map.items():
                f.write(f"  • {id}: {name}\n")
            
            f.write("\n4. KEY FINDINGS\n")
            f.write("-" * 40 + "\n")
            f.write("  • EMA Filter: Best for real-time applications with low latency requirements\n")
            f.write("  • Median Filter: Excellent for impulse noise removal\n")
            f.write("  • Low-Pass Filter: Optimal frequency selectivity\n")
            f.write("  • Outlier Filter: Best for detecting and removing anomalies\n")
            f.write("  • Madgwick Filter: Superior for orientation estimation\n")
            
            f.write("\n5. RECOMMENDATIONS\n")
            f.write("-" * 40 + "\n")
            f.write("  • For real-time systems: EMA or Low-Pass filters\n")
            f.write("  • For noisy environments: Median or Outlier filters\n")
            f.write("  • For orientation tracking: Madgwick filter\n")
            f.write("  • For post-processing: Combination of filters\n")
            
        print(f"  Generated summary report at {report_path}")
    
    def run_complete_analysis(self, file_paths):
        print("=" * 80)
        print("Starting IMU Filter Analysis Framework")
        print("=" * 80)
        
        self.load_experiments(file_paths)
        
        for filter_type in self.filter_types:
            print(f"\nAnalyzing {filter_type.upper()} filter across all experiments...")
            filter_dir = self.base_output_dir / filter_type
            filter_dir.mkdir(exist_ok=True)

            for exp in self.experiments:
                aid = exp['activity_id']
                activity_name = exp['activity_name']
                activity_dir = filter_dir / self._activity_dirname(aid)
                activity_dir.mkdir(parents=True, exist_ok=True)

                print(f"  Processing {exp['name']} ({activity_name})...")

                metrics = self.analyze_filter(
                    filter_type=filter_type,
                    exp_data=exp['data'],
                    exp_name=exp['name'],
                    output_dir=activity_dir,       
                    activity_id_filter=aid
                )

                if filter_type not in self.all_metrics:
                    self.all_metrics[filter_type] = {}
                self.all_metrics[filter_type][exp['name']] = metrics

        
        self.compare_filters()
        
        print("\n" + "=" * 80)
        print("Analysis Complete!")
        print(f"Results saved to: {self.base_output_dir}")
        print("=" * 80)


def main():
    analyzer = IMUFilterAnalyzer(base_output_dir="filter_analysis_results")
    
    experiment_files = [
        "./data/experiment_1.csv",
        "./data/experiment_2.csv",
        "./data/experiment_3.csv",
        "./data/experiment_4.csv",
        "./data/experiment_5.csv"
    ]
    
    analyzer.run_complete_analysis(experiment_files)





class AdvancedMetricsCalculator:
    @staticmethod
    def calculate_allan_variance(data, dt=0.02):
        n = len(data)
        max_cluster = n // 2
        allan_var = []
        cluster_sizes = []
        
        for cluster_size in range(1, max_cluster, max(1, max_cluster//50)):
            clusters = n // cluster_size
            if clusters < 2:
                break
                
            averages = []
            for i in range(clusters):
                start = i * cluster_size
                end = start + cluster_size
                if end <= n:
                    averages.append(np.mean(data[start:end]))
            
            if len(averages) > 1:
                diff_squared = [(averages[i+1] - averages[i])**2 
                               for i in range(len(averages)-1)]
                allan_var.append(np.mean(diff_squared) / 2)
                cluster_sizes.append(cluster_size * dt)
        
        return cluster_sizes, allan_var
    
    @staticmethod
    def calculate_frequency_response(raw_data, filtered_data, fs=50):
        f, H = signal.csd(raw_data, filtered_data, fs=fs, nperseg=256)
        f_raw, P_raw = signal.csd(raw_data, raw_data, fs=fs, nperseg=256)
        
        mask = P_raw != 0
        H_est = np.zeros_like(H)
        H_est[mask] = H[mask] / P_raw[mask]
        
        magnitude_db = 20 * np.log10(np.abs(H_est) + 1e-10)
        phase_deg = np.angle(H_est, deg=True)
        
        return f, magnitude_db, phase_deg
    
    @staticmethod
    def calculate_activity_recognition_metrics(true_labels, predicted_labels):
        from sklearn.metrics import confusion_matrix, classification_report
        
        cm = confusion_matrix(true_labels, predicted_labels)
        report = classification_report(true_labels, predicted_labels, output_dict=True)
        
        return cm, report


if __name__ == "__main__":
    main()
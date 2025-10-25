#!/usr/bin/env python3
"""
Analyzer that reads pre-computed filter values from CSV files and organizes output by filter type.
Each filter gets its own directory with comparisons across all input files.
Time series are normalized to start from 0 ms.
"""

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from typing import Dict, List
import seaborn as sns
import glob
import warnings

# Suppress matplotlib warnings about empty legends
warnings.filterwarnings('ignore', message='No artists with labels found')

# Set style for better-looking plots
sns.set_style("whitegrid")
plt.rcParams['figure.dpi'] = 100

def normalize_time(df: pd.DataFrame) -> pd.DataFrame:
    """Ensure time starts from 0 ms."""
    df = df.copy()
    if 't_ms' in df.columns:
        t = pd.to_numeric(df['t_ms'], errors='coerce')
        df['t_ms_normalized'] = t - t.iloc[0]
    else:
        # If no t_ms column, create one
        df['t_ms_normalized'] = np.arange(len(df)) * 100  # Assume 100ms intervals
    return df

def compute_metrics(series: pd.Series, truth: float = None) -> Dict:
    """Compute various metrics for a series."""
    s = pd.to_numeric(series, errors='coerce').dropna()
    
    metrics = {
        'mean': float(s.mean()) if len(s) > 0 else np.nan,
        'std': float(s.std()) if len(s) > 0 else np.nan,
        'min': float(s.min()) if len(s) > 0 else np.nan,
        'max': float(s.max()) if len(s) > 0 else np.nan,
        'range': float(s.max() - s.min()) if len(s) > 0 else np.nan,
        'variance': float(s.var()) if len(s) > 0 else np.nan,
    }
    
    if truth is not None and not np.isnan(truth):
        if len(s) > 0:
            errors = s - truth
            metrics['truth'] = truth
            metrics['mean_error'] = float(errors.mean())
            metrics['abs_mean_error'] = float(errors.abs().mean())
            metrics['rmse'] = float(np.sqrt((errors**2).mean()))
            metrics['max_error'] = float(errors.abs().max())
            metrics['percent_error'] = float(abs(s.mean() - truth) / truth * 100) if truth != 0 else np.nan
    
    return metrics

def analyze_by_filter(input_files: List[Path], output_dir: Path, filters_to_analyze: List[str] = None):
    """Main analysis function organized by filter type."""
    
    # Default filters to analyze if not specified
    if filters_to_analyze is None:
        filters_to_analyze = ['ema_cm', 'lp_cm', 'median_cm', 'outlier_cm', 'kalman_cm']
    
    # Create output directory structure
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Create a directory for each filter
    filter_dirs = {}
    for filter_name in filters_to_analyze:
        filter_dir = output_dir / filter_name.replace('_cm', '')
        filter_dir.mkdir(exist_ok=True)
        filter_dirs[filter_name] = filter_dir
    
    # Load all data files
    all_data = {}
    for file_path in input_files:
        if not file_path.exists():
            print(f"Warning: {file_path} does not exist")
            continue
        
        df = pd.read_csv(file_path)
        df = normalize_time(df)
        
        # Extract truth value if present
        truth = None
        if 'true_distance_cm' in df.columns:
            truth_vals = pd.to_numeric(df['true_distance_cm'], errors='coerce').dropna().unique()
            if len(truth_vals) == 1:
                truth = float(truth_vals[0])
        
        all_data[file_path.stem] = {
            'df': df,
            'truth': truth
        }
    
    # Analyze each filter across all files
    summary_metrics = []
    
    for filter_name in filters_to_analyze:
        filter_dir = filter_dirs[filter_name]
        filter_label = filter_name.replace('_cm', '')
        
        print(f"\nAnalyzing filter: {filter_label}")
        
        # Create comparison plots
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle(f'Filter: {filter_label.upper()} - Comparison Across Files', fontsize=16)
        
        # Subplot 1: Time series comparison
        ax1 = axes[0, 0]
        
        # Subplot 2: Raw vs Filtered overlay
        ax2 = axes[0, 1]
        
        # Subplot 3: Error distribution (if truth available)
        ax3 = axes[1, 0]
        
        # Subplot 4: Metrics bar chart
        ax4 = axes[1, 1]
        
        file_metrics = []
        colors = plt.cm.tab10(np.linspace(0, 1, len(all_data)))
        
        for idx, (file_name, data) in enumerate(all_data.items()):
            df = data['df']
            truth = data['truth']
            color = colors[idx]
            
            if filter_name not in df.columns:
                print(f"  Warning: {filter_name} not found in {file_name}")
                continue
            
            # Get time and filter series
            t = df['t_ms_normalized']
            filtered = pd.to_numeric(df[filter_name], errors='coerce')
            raw = pd.to_numeric(df['raw_cm'], errors='coerce')
            
            # Plot time series
            ax1.plot(t, filtered, label=f'{file_name}', color=color, alpha=0.8)
            if truth is not None:
                ax1.axhline(y=truth, color=color, linestyle='--', alpha=0.3)
            
            # Plot raw vs filtered
            ax2.plot(t, raw, color=color, alpha=0.3, linestyle='--')
            ax2.plot(t, filtered, color=color, alpha=0.8, label=f'{file_name}')
            
            # Calculate metrics
            metrics = compute_metrics(filtered, truth)
            metrics['file'] = file_name
            metrics['filter'] = filter_label
            file_metrics.append(metrics)
            
            # Plot error distribution if truth available
            if truth is not None:
                errors = filtered - truth
                ax3.hist(errors.dropna(), alpha=0.5, label=f'{file_name}', color=color, bins=30)
        
        # Configure subplot 1 (time series)
        ax1.set_xlabel('Time (ms)')
        ax1.set_ylabel('Distance (cm)')
        ax1.set_title('Filtered Signals')
        if ax1.get_lines():  # Only add legend if there are lines
            ax1.legend(loc='best', fontsize=8)
        ax1.grid(True, alpha=0.3)
        
        # Configure subplot 2 (raw vs filtered)
        ax2.set_xlabel('Time (ms)')
        ax2.set_ylabel('Distance (cm)')
        ax2.set_title('Raw (dashed) vs Filtered (solid)')
        if ax2.get_lines():  # Only add legend if there are lines
            ax2.legend(loc='best', fontsize=8)
        ax2.grid(True, alpha=0.3)
        
        # Configure subplot 3 (error distribution)
        ax3.set_xlabel('Error (cm)')
        ax3.set_ylabel('Frequency')
        ax3.set_title('Error Distribution (Filtered - Truth)')
        if ax3.patches:  # Only add legend if there are histogram patches
            ax3.legend(loc='best', fontsize=8)
        ax3.grid(True, alpha=0.3)
        
        # Plot metrics comparison
        if file_metrics:
            metrics_df = pd.DataFrame(file_metrics)
            
            # Select key metrics for bar chart
            if 'rmse' in metrics_df.columns:
                metric_cols = ['rmse', 'abs_mean_error', 'std']
            else:
                metric_cols = ['std', 'range']
            
            x_pos = np.arange(len(metrics_df))
            width = 0.25
            
            for i, metric in enumerate(metric_cols):
                if metric in metrics_df.columns:
                    ax4.bar(x_pos + i*width, metrics_df[metric], width, label=metric)
            
            ax4.set_xlabel('Files')
            ax4.set_ylabel('Value (cm)')
            ax4.set_title('Metrics Comparison')
            ax4.set_xticks(x_pos + width)
            ax4.set_xticklabels(metrics_df['file'], rotation=45, ha='right')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
            
            # Save metrics to CSV
            metrics_df.to_csv(filter_dir / f'{filter_label}_metrics.csv', index=False)
            summary_metrics.extend(file_metrics)
        
        plt.tight_layout()
        plt.savefig(filter_dir / f'{filter_label}_comparison.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        # Create individual comparison plots for each file
        for file_name, data in all_data.items():
            df = data['df']
            truth = data['truth']
            
            if filter_name not in df.columns or 'raw_cm' not in df.columns:
                continue
            
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
            fig.suptitle(f'{file_name} - Filter: {filter_label.upper()}', fontsize=14)
            
            t = df['t_ms_normalized']
            filtered = pd.to_numeric(df[filter_name], errors='coerce')
            raw = pd.to_numeric(df['raw_cm'], errors='coerce')
            
            # Plot raw vs filtered
            ax1.plot(t, raw, label='Raw', color='gray', alpha=0.5)
            ax1.plot(t, filtered, label=filter_label, color='blue', linewidth=2)
            if truth is not None:
                ax1.axhline(y=truth, color='red', linestyle='--', label=f'Truth ({truth} cm)')
            ax1.set_xlabel('Time (ms)')
            ax1.set_ylabel('Distance (cm)')
            ax1.set_title('Raw vs Filtered Signal')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Plot difference
            diff = filtered - raw
            ax2.plot(t, diff, color='green', alpha=0.7)
            ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
            ax2.fill_between(t, 0, diff, alpha=0.3, color='green')
            ax2.set_xlabel('Time (ms)')
            ax2.set_ylabel('Difference (cm)')
            ax2.set_title('Filter Effect (Filtered - Raw)')
            ax2.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(filter_dir / f'{file_name}_{filter_label}.png', dpi=150, bbox_inches='tight')
            plt.close()
    
    # Save overall summary
    if summary_metrics:
        summary_df = pd.DataFrame(summary_metrics)
        summary_df.to_csv(output_dir / 'all_filters_summary.csv', index=False)
        
        # Create pivot tables for easy comparison
        if 'rmse' in summary_df.columns:
            pivot_rmse = summary_df.pivot_table(index='file', columns='filter', values='rmse')
            pivot_rmse.to_csv(output_dir / 'rmse_comparison.csv')
            
            # Plot RMSE comparison
            fig, ax = plt.subplots(figsize=(10, 6))
            pivot_rmse.plot(kind='bar', ax=ax)
            ax.set_ylabel('RMSE (cm)')
            ax.set_title('RMSE Comparison Across Filters and Files')
            ax.set_xlabel('File')
            plt.xticks(rotation=45, ha='right')
            plt.legend(title='Filter')
            plt.tight_layout()
            plt.savefig(output_dir / 'rmse_comparison.png', dpi=150, bbox_inches='tight')
            plt.close()
        
        # Mean comparison
        pivot_mean = summary_df.pivot_table(index='file', columns='filter', values='mean')
        pivot_mean.to_csv(output_dir / 'mean_comparison.csv')
        
        # Std comparison
        pivot_std = summary_df.pivot_table(index='file', columns='filter', values='std')
        pivot_std.to_csv(output_dir / 'std_comparison.csv')
        
        print(f"\nAnalysis complete. Results saved to {output_dir}")
        print("\nSummary Statistics:")
        if 'rmse' in summary_df.columns:
            print(summary_df.groupby('filter')[['mean', 'std', 'rmse']].mean())
        else:
            print(summary_df.groupby('filter')[['mean', 'std']].mean())

def main():
    parser = argparse.ArgumentParser(
        description='Analyze pre-filtered data organized by filter type'
    )
    parser.add_argument(
        'inputs', 
        nargs='+', 
        help='CSV files with pre-computed filter values (supports wildcards)'
    )
    parser.add_argument(
        '-o', '--output', 
        default='filter_analysis_output',
        help='Output directory (default: filter_analysis_output)'
    )
    parser.add_argument(
        '--filters',
        nargs='+',
        default=['ema_cm', 'lp_cm', 'median_cm', 'outlier_cm', 'kalman_cm'],
        help='Filter columns to analyze'
    )
    
    args = parser.parse_args()
    
    # Expand glob patterns and convert to Path objects
    input_files = []
    for pattern in args.inputs:
        if '*' in pattern or '?' in pattern:
            # It's a glob pattern
            files = glob.glob(pattern)
            if not files:
                print(f"Warning: No files found matching pattern '{pattern}'")
            input_files.extend([Path(f) for f in files])
        else:
            # It's a regular file path
            input_files.append(Path(pattern))
    
    if not input_files:
        print("Error: No input files found")
        return
    
    output_dir = Path(args.output)
    
    # Run analysis
    analyze_by_filter(input_files, output_dir, args.filters)

if __name__ == '__main__':
    main()
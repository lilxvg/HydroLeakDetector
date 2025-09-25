import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks, butter, filtfilt, correlate, stft, welch
import glob
import os
import warnings
warnings.filterwarnings('ignore')

class EnhancedLeakDetector:
    def __init__(self, sampling_rate=50000000, pipe_length=1.0):
        self.sampling_rate = sampling_rate
        self.pipe_length = pipe_length
        self.dt = 1.0 / sampling_rate
        self.downsample_factor = 100
        self.effective_sampling_rate = sampling_rate / self.downsample_factor
        self.effective_dt = 1 / self.effective_sampling_rate
        self.speed_of_sound = 1480
        
    def load_and_preprocess(self, filepath, time_col=0, signal_col=1):
        try:
            print(f"Loading {filepath}...")
            data = pd.read_csv(filepath)
            if len(data.columns) <= max(time_col, signal_col):
                time_col, signal_col = 0, 1
            time = data.iloc[:, time_col].values
            signal = data.iloc[:, signal_col].values
            print(f"Original: {len(signal)} samples at {self.sampling_rate/1e6:.1f} MS/s")
            if len(signal) > 100000:
                signal = self.downsample(signal)
                time = time[::self.downsample_factor]
                print(f"Downsampled: {len(signal)} samples at {self.effective_sampling_rate/1e3:.1f} kS/s")
            return time, signal
        except Exception as e:
            print(f"Error loading {filepath}: {e}")
            return None, None
            
    def downsample(self, signal):
        return signal[::self.downsample_factor]
        
    def apply_adaptive_filter(self, signal, pinger_freq=10000):
        signal = signal - np.mean(signal)
        f, Pxx = welch(signal, self.effective_sampling_rate, nperseg=1024)
        noise_floor = np.percentile(Pxx, 10)
        signal_power = np.max(Pxx)
        if signal_power / noise_floor > 100:
            lowcut = max(1000, pinger_freq * 0.5)
            highcut = min(20000, pinger_freq * 2)
        else:
            lowcut = max(500, pinger_freq * 0.8)
            highcut = min(15000, pinger_freq * 1.2)
        print(f"Applying bandpass filter: {lowcut/1000:.1f} - {highcut/1000:.1f} kHz")
        nyquist = 0.5 * self.effective_sampling_rate
        low = lowcut / nyquist
        high = highcut / nyquist
        if low >= 1 or high >= 1:
            b, a = butter(4, high, btype='low')
        else:
            b, a = butter(4, [low, high], btype='band')
        return filtfilt(b, a, signal)
        
    def find_direct_pulse(self, signal, method='peak'):
        # First, let's find the approximate location of the direct pulse
        # Look for the first major peak in the signal
        if method == 'energy':
            window_size = int(0.0001 / self.effective_dt)
            energy = np.convolve(signal**2, np.ones(window_size)/window_size, mode='same')
            pulse_start = np.argmax(energy)
            return pulse_start
        elif method == 'peak':
            # Find the first significant peak in the signal
            peaks, _ = find_peaks(np.abs(signal), height=np.max(np.abs(signal))*0.5, distance=int(0.0001/self.effective_dt))
            if len(peaks) > 0:
                return peaks[0]
            else:
                return np.argmax(np.abs(signal))
        elif method == 'matched_filter':
            template = np.exp(-np.linspace(-3, 3, 100)**2) * np.sin(2 * np.pi * 10000 * np.linspace(0, 0.0001, 100))
            correlation = np.correlate(np.abs(signal), np.abs(template), mode='same')
            return np.argmax(correlation)
            
    def calibrate_with_end_cap(self, time, signal):
        direct_idx = self.find_direct_pulse(signal, method='peak')
        expected_delay = (2 * self.pipe_length) / 1480
        search_start = direct_idx + int(0.8 * expected_delay / self.effective_dt)
        search_end = direct_idx + int(1.2 * expected_delay / self.effective_dt)
        search_end = min(search_end, len(signal) - 1)
        if search_end <= search_start:
            print("Warning: Cannot find end cap reflection, using default speed of sound")
            return 1480, direct_idx, direct_idx
        search_signal = np.abs(signal[search_start:search_end])
        end_cap_idx = search_start + np.argmax(search_signal)
        time_diff = time[end_cap_idx] - time[direct_idx]
        if time_diff <= 0:
            print("Warning: Invalid time difference, using default speed of sound")
            return 1480, direct_idx, end_cap_idx
        calculated_speed = (2 * self.pipe_length) / time_diff
        if not (1000 <= calculated_speed <= 2000):
            print(f"Warning: Unrealistic speed of sound {calculated_speed:.1f} m/s, using default")
            return 1480, direct_idx, end_cap_idx
        print(f"Calibrated speed of sound: {calculated_speed:.1f} m/s")
        return calculated_speed, direct_idx, end_cap_idx
        
    def align_signals(self, baseline_signal, scenario_signal):
        min_length = min(len(baseline_signal), len(scenario_signal))
        baseline_trunc = baseline_signal[:min_length]
        scenario_trunc = scenario_signal[:min_length]
        correlation = correlate(baseline_trunc, scenario_trunc, mode='full')
        lags = np.arange(-len(scenario_trunc) + 1, len(baseline_trunc))
        best_lag = lags[np.argmax(correlation)]
        return best_lag, min_length
    def detect_leak_echoes(self, baseline_signal, scenario_signal, time, direct_idx, top_n=10):
        min_length = min(len(baseline_signal), len(scenario_signal))
        baseline_signal = baseline_signal[:min_length]
        scenario_signal = scenario_signal[:min_length]
        time = time[:min_length]
        
        # Find the time range where we expect to see the echo (1.05s to 1.1s)
        time_start_idx = np.argmin(np.abs(time - 0.02))
        time_end_idx = np.argmin(np.abs(time - 2.5))
        
        # Ensure we have a valid search window
        if time_end_idx <= time_start_idx:
            print("Warning: Invalid time window (1.05s-1.1s), using default search range")
            max_time = (2 * self.pipe_length) / self.speed_of_sound
            search_start = direct_idx + int(0.0001 / self.effective_dt)
            search_end = direct_idx + int(max_time / self.effective_dt)
            search_end = min(search_end, len(time) - 1)
        else:
            search_start = time_start_idx
            search_end = time_end_idx
        
        # Ensure we're searching in the correct time range
        search_start = max(search_start, 0)
        search_end = min(search_end, len(time) - 1)
        
        if search_end <= search_start:
            print("Warning: Search window is invalid, using default search range")
            max_time = (2 * self.pipe_length) / self.speed_of_sound
            search_start = direct_idx + int(0.0001 / self.effective_dt)
            search_end = direct_idx + int(max_time / self.effective_dt)
            search_end = min(search_end, len(time) - 1)
        
        # Check if the search window is still empty
        if search_end <= search_start:
            print("Error: Search window is still empty after adjustments")
            return [], [], None
        
        diff_signal = np.abs(scenario_signal) - np.abs(baseline_signal)
        baseline_abs = np.abs(baseline_signal[search_start:search_end])
        scenario_abs = np.abs(scenario_signal[search_start:search_end])
        
        # Check if the window is too small for convolution
        window_size = int(0.00005 / self.effective_dt)
        if window_size < 5:
            window_size = 5
        
        if len(baseline_abs) < window_size:
            print("Warning: Search window too small for convolution, adjusting window size")
            window_size = max(3, len(baseline_abs) // 2)
        
        energy_baseline = np.convolve(baseline_abs**2, np.ones(window_size)/window_size, mode='same')
        energy_scenario = np.convolve(scenario_abs**2, np.ones(window_size)/window_size, mode='same')
        energy_diff = energy_scenario - energy_baseline
        
        norm_diff = (scenario_abs - baseline_abs) / (baseline_abs + 1e-10)
        
        combined_evidence = np.zeros_like(diff_signal[search_start:search_end])
        combined_evidence += diff_signal[search_start:search_end] / np.max(np.abs(diff_signal[search_start:search_end]) + 1e-10)
        combined_evidence += norm_diff / np.max(np.abs(norm_diff) + 1e-10)
        combined_evidence += energy_diff / np.max(np.abs(energy_diff) + 1e-10)
        
        min_height = 1.0
        min_distance = int(0.0001 / self.effective_dt)
        
        # Ensure we have valid peaks
        if len(combined_evidence) == 0:
            print("Warning: No evidence data for peak detection")
            return [], [], None
        
        peaks, properties = find_peaks(
            combined_evidence, 
            height=min_height,
            distance=min_distance,
            prominence=0.3
        )
        
        if top_n is not None and len(peaks) > 0:
            # sort peaks by their combined_evidence value (descending) and pick top_n
            order = np.argsort(combined_evidence[peaks])[::-1]
            selected = peaks[order[:top_n]]
            peaks = np.sort(selected)  # optional: keep indices in ascending order

        echo_indices = peaks + search_start
        echo_metrics = []
        
        for idx in echo_indices:
            if idx >= len(time):
                continue
            time_diff = time[idx] - time[direct_idx]
            distance = (self.speed_of_sound * time_diff) / 2
            
            # Only consider echoes within the pipe (0 to pipe_length)
            if 0 < distance <= self.pipe_length:
                echo_metrics.append({
                    'time': time[idx],
                    'time_diff': time_diff,
                    'distance': distance,
                    'amplitude': scenario_signal[idx],
                    'evidence': combined_evidence[idx - search_start]
                })
            else:
                print(f"  Ignoring echo at {distance:.3f}m (beyond pipe length of {self.pipe_length}m)")
        
        echo_metrics.sort(key=lambda x: x['evidence'], reverse=True)
        primary_leak = echo_metrics[0] if echo_metrics else None
        return echo_indices, echo_metrics, primary_leak            
    def plot_detailed_analysis(self, baseline_time, baseline_signal, scenario_signal,
                                    direct_idx, echo_indices, echo_metrics, primary_leak, scenario_name, save_path=None):
            min_length = min(len(baseline_time), len(scenario_signal))
            baseline_time = baseline_time[:min_length]
            baseline_signal = baseline_signal[:min_length]
            scenario_signal = scenario_signal[:min_length]

            # Time relative to direct pulse
            time_since_direct = baseline_time - baseline_time[direct_idx]

            # Keep only valid echo indices
            echo_indices = [idx for idx in echo_indices if idx < min_length]

            fig, axes = plt.subplots(3, 1, figsize=(12, 15))

            baseline_legend = "Baseline"
            scenario_legend = f"Leak at {primary_leak['distance']:.3f}m" if primary_leak else scenario_name

            flipped_time = time_since_direct[::-1]
            baseline_signal_flipped = baseline_signal
            scenario_signal_flipped = scenario_signal

            # Raw signals
            axes[0].plot(flipped_time, baseline_signal_flipped, color='blue', linestyle='-', label=baseline_legend, alpha=0.7)
            axes[0].plot(flipped_time, scenario_signal_flipped, color='red', linestyle='-', label=scenario_legend, alpha=0.7)
            axes[0].set_xlabel('Time Before Direct Pulse (s)')
            axes[0].set_ylabel('Amplitude')
            axes[0].set_title('Raw Signals')
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)
            if len(flipped_time) > 0:
                axes[0].set_xlim(0, flipped_time[0])

            # Difference signal (no point markers)
            diff_signal = np.abs(scenario_signal_flipped) - np.abs(baseline_signal_flipped)
            diff_legend = 'Difference (|Scenario| - |Baseline|)'
            axes[1].plot(flipped_time, diff_signal, color='purple', label=diff_legend, alpha=0.7)
            axes[1].axhline(y=0, color='black', linestyle='-', alpha=0.5)

            # Annotate echoes without plotting decorative markers
            for i, idx in enumerate(echo_indices):
                flipped_idx = len(flipped_time) - 1 - idx
                if 0 <= flipped_idx < len(flipped_time):
                    echo_distance = None
                    is_primary = False

                    # Match echo index to metrics by time_diff
                    for echo in echo_metrics:
                        if abs(time_since_direct[idx] - echo['time_diff']) < 1e-6:
                            echo_distance = echo['distance']
                            is_primary = bool(echo.get('primary', False))
                            break

                    if echo_distance is None:
                        continue

                    # Use colored annotation text (green for primary, red for others)
                    # annot_color = 'green' if is_primary else 'red'
                    # label = f"Primary Leak: {echo_distance:.3f} m" if is_primary else f"Echo {i+1}: {echo_distance:.3f} m"
                    # axes[1].annotate(label,
                                    # xy=(flipped_time[flipped_idx], diff_signal[flipped_idx]),
                                    # xytext=(10, 10),
                                    # textcoords='offset points',
                                    # color=annot_color,
                                    # arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0', color=annot_color))

            axes[1].set_xlabel('Time Before Direct Pulse (s)')
            axes[1].set_ylabel('Amplitude Difference')
            axes[1].set_title('Difference Signal')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)
            if len(flipped_time) > 0:
                axes[1].set_xlim(0, flipped_time[0])

            # Distance bar chart
            if echo_metrics:
                distances = [echo['distance'] for echo in echo_metrics]
                evidence = [echo['evidence'] for echo in echo_metrics]
                is_primary = [echo.get('primary', False) for echo in echo_metrics]
                colors = ['green' if primary else 'orange' for primary in is_primary]
                bars = axes[2].bar(range(len(distances)), distances, color=colors, alpha=0.7)
                axes[2].set_xlabel('Echo Number')
                axes[2].set_ylabel('Distance (m)')
                axes[2].set_title('Estimated Leak Distances (Green = Primary)')

                for i, (d, e, primary) in enumerate(zip(distances, evidence, is_primary)):
                    text_color = 'green' if primary else 'black'
                    weight = 'bold' if primary else 'normal'
                    axes[2].text(i, d, f'{d:.3f} m\n(conf: {e:.2f})', ha='center', va='bottom', color=text_color, weight=weight)
            else:
                axes[2].text(0.5, 0.5, 'No echoes detected', ha='center', va='center', transform=axes[2].transAxes)
                axes[2].set_title('Estimated Leak Distances')

            plt.tight_layout()
            if save_path:
                plt.savefig(save_path, dpi=300, bbox_inches='tight')
            plt.show()
    def analyze_scenario(self, baseline_path, scenario_path, output_dir):
        baseline_time, baseline_raw = self.load_and_preprocess(baseline_path)
        scenario_time, scenario_raw = self.load_and_preprocess(scenario_path)
        
        if baseline_time is None or scenario_time is None:
            print("Error: Could not load data files")
            return None, None, None
            
        baseline_filtered = self.apply_adaptive_filter(baseline_raw)
        scenario_filtered = self.apply_adaptive_filter(scenario_raw)
        
        min_length = min(len(baseline_filtered), len(scenario_filtered))
        baseline_filtered = baseline_filtered[:min_length]
        scenario_filtered = scenario_filtered[:min_length]
        baseline_time = baseline_time[:min_length]
        
        self.speed_of_sound, direct_idx, end_cap_idx = self.calibrate_with_end_cap(
            baseline_time, baseline_filtered)
        
        echo_indices, echo_metrics, primary_leak = self.detect_leak_echoes(
            baseline_filtered, scenario_filtered, baseline_time, direct_idx)
        
        if primary_leak:
            for echo in echo_metrics:
                echo['primary'] = (abs(echo['time'] - primary_leak['time']) < 1e-6)
        
        os.makedirs(output_dir, exist_ok=True)
        
        scenario_name = os.path.splitext(os.path.basename(scenario_path))[0]
        plot_path = os.path.join(output_dir, f"{scenario_name}_analysis.png")
        
        self.plot_detailed_analysis(
            baseline_time, baseline_filtered, scenario_filtered,
            direct_idx, echo_indices, echo_metrics, primary_leak, scenario_name, plot_path)
        
        print(f"\n=== RESULTS FOR {scenario_name} ===")
        
        if echo_metrics:
            print(f"Detected {len(echo_metrics)} potential echoes:")
            print("-" * 60)
            
            for i, echo in enumerate(echo_metrics):
                is_primary = echo.get('primary', False)
                marker = "★" if is_primary else "○"
                
                print(f"{marker} Echo {i+1}:")
                print(f"   Distance: {echo['distance']:.3f} m from sensor (confidence: {echo['evidence']:.3f})")
                print(f"   Time delay: {echo['time_diff']*1000:.2f} ms")
                
                if is_primary:
                    print("   STATUS: PRIMARY LEAK CANDIDATE")
                
                print()
            
            if primary_leak:
                print("=== VALIDATION ===")
                print(f"Primary leak detected at: {primary_leak['distance']:.3f} m from hydrophone")
                print(f"Time delay: {primary_leak['time_diff']*1000:.2f} ms")
                print(f"Confidence: {primary_leak['evidence']:.3f}")
                
                if 0 < primary_leak['distance'] <= self.pipe_length:
                    print("✓ Distance is within pipe bounds (0 - {self.pipe_length} m)")
                else:
                    print("✗ Warning: Distance is outside pipe bounds")
                
                if primary_leak['evidence'] > 1.0:
                    print("✓ Strong evidence for leak detection")
                elif primary_leak['evidence'] > 0.5:
                    print("~ Moderate evidence for leak detection")
                else:
                    print("✗ Weak evidence for leak detection")
        else:
            print("No leak echoes detected")
            
            print("\n=== DIAGNOSTICS ===")
            print(f"Direct pulse found at: {baseline_time[direct_idx]:.6f} s")
            print(f"End cap reflection at: {baseline_time[end_cap_idx]:.6f} s")
            print(f"Speed of sound: {self.speed_of_sound:.1f} m/s")
            
            correlation = np.corrcoef(baseline_filtered, scenario_filtered)[0, 1]
            print(f"Correlation between signals: {correlation:.3f}")
            
            if correlation > 0.99:
                print("✗ Signals are very similar - possible issues with leak simulation")
            else:
                print("✓ Signals are different - leak should be detectable")
                
            baseline_energy = np.sum(baseline_filtered**2)
            scenario_energy = np.sum(scenario_filtered**2)
            energy_ratio = scenario_energy / baseline_energy
            
            print(f"Energy ratio (scenario/baseline): {energy_ratio:.3f}")
            
            if energy_ratio < 1.1:
                print("✗ Little additional energy in scenario - leak might be too weak")
            else:
                print("✓ Additional energy detected in scenario - leak should be present")
        
        return echo_metrics, baseline_time, scenario_filtered

def main():
    detector = EnhancedLeakDetector(
        sampling_rate=50000000,
        pipe_length=0.8  
    )
    
    baseline_path = "C:\\Users\\abdul\\Desktop\\Hydrophone_Project\\Response_Folder\\analogbaseline.csv"
    output_dir = "C:\\Users\\abdul\\Desktop\\Hydrophone_Project\\enhanced_analysis"
    
    scenario_files = glob.glob("C:\\Users\\abdul\\Desktop\\Hydrophone_Project\\Response_Folder\\analog2.csv")
    scenario_files = [f for f in scenario_files if "baseline" not in f.lower()]
    
    print(f"Found {len(scenario_files)} scenario files")
    
    all_results = []
    
    for scenario_path in scenario_files:
        print(f"\n{'='*60}")
        print(f"Processing: {os.path.basename(scenario_path)}")
        print(f"{'='*60}")
        
        results, time, signal = detector.analyze_scenario(
            baseline_path, scenario_path, output_dir)
        
        if results is not None:
            primary_leak = None
            for leak in results:
                if leak.get('primary', False):
                    primary_leak = leak
                    break
                    
            all_results.append({
                'file': os.path.basename(scenario_path),
                'leaks': results,
                'primary_leak': primary_leak
            })
    
    summary_path = os.path.join(output_dir, "leak_detection_summary.txt")
    with open(summary_path, 'w') as f:
        f.write("ENHANCED LEAK DETECTION SUMMARY\n")
        f.write("=" * 50 + "\n\n")
        
        f.write(f"Sampling rate: {detector.sampling_rate/1e6:.1f} MS/s\n")
        f.write(f"Effective sampling rate: {detector.effective_sampling_rate/1e3:.1f} kS/s\n")
        f.write(f"Pipe length: {detector.pipe_length} m\n")
        f.write(f"Speed of sound: {detector.speed_of_sound:.1f} m/s\n\n")
        
        f.write("DETECTION RESULTS:\n")
        f.write("=" * 50 + "\n")
        
        for result in all_results:
            f.write(f"\n{result['file']}:\n")
            
            if result['leaks']:
                for i, leak in enumerate(result['leaks']):
                    primary_marker = "★ " if leak.get('primary', False) else "  "
                    f.write(f"{primary_marker}Leak {i+1}: {leak['distance']:.3f}m from hydrophone "
                           f"(Δt = {leak['time_diff']*1000:.2f}ms, confidence = {leak['evidence']:.3f})\n")
            else:
                f.write("  No leaks detected\n")
                
            if result['primary_leak']:
                f.write(f"  → PRIMARY LEAK: {result['primary_leak']['distance']:.3f}m from hydrophone\n")
    
    print(f"\nAnalysis complete! Results saved to '{output_dir}'")
    print(f"Summary: {summary_path}")

if __name__ == "__main__":
    main()
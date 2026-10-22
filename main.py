"""
tamper_gui_settle.py

Complete script:
- Serial reading (ESP32 -> HX711)
- Filters + spike/step detectors
- Deflection detectors (slope + CUSUM)
- Startup suppression, weight-placement settle window, cooldown & single-alert
- Snapshot save, auto-zip, acknowledge
"""

import serial
import threading
import time
from collections import deque, namedtuple
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.signal import savgol_filter, butter, filtfilt
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path
from datetime import datetime, timedelta
import os
import zipfile
import sys
import traceback

# ---------------- USER SETTINGS ----------------
SERIAL_PORT = "COM13"       # change to your COM port
BAUD = 115200
TIMEOUT = 1.0

BASE_DIR = Path(__file__).resolve().parent
OUT_DIR = BASE_DIR / "scale_logs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# filters & parameters
SAMPLE_RATE_HZ = 50
READAVG_N = 5
SMA_WINDOW = 10
EXP_ALPHA = 0.08
MEDIAN_WINDOW = 5
SG_WINDOW = 11
SG_POLY = 2
BUTTERORDER = 3
BUTTER_CUTOFF_HZ = 2.0

# tamper thresholds (defaults; compute with Learn Baseline)
TAMPER_SPIKE_THRESHOLD = 1000
TAMPER_STEP_THRESHOLD = 2000
BLOCK_SIZE = 50

# Deflection detectors (defaults — Learn Baseline will update)
SLOPE_WINDOW = 60
SLOPE_THRESH = 0.8
SLOPE_DEBOUNCE = 3

CUSUM_K = 10.0
CUSUM_H = 500.0

# settle logic for weight placement
SETTLE_AFTER_WEIGHT_SEC = 5.0   # seconds to suppress alarms after a placement
WEIGHT_PLACEMENT_DELTA = 2000.0  # counts threshold to call it a placement

# auto-zip
ZIP_AFTER_DAYS = 7
KEEP_RECENT_FILES = 30

# buzzer (if your ESP32 supports commands)
ENABLE_SERIAL_BUZZER = False

# cooldown & startup suppression
STARTUP_SUPPRESS_SEC = 2.0
TAMPER_COOLDOWN_SEC = 5.0

TamperResult = namedtuple("TamperResult", ["score", "flags", "reason"])

# helper: butter
def butter_lowpass(cutoff, fs, order=3):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

# Serial thread: reads lines and calls callback
class SerialLoggerThread(threading.Thread):
    def __init__(self, port, baud, timeout, data_callback, error_callback):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.data_callback = data_callback
        self.error_callback = error_callback
        self._stop_event = threading.Event()
        self.ser = None

    def open(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except Exception as e:
            self.error_callback(f"Cannot open serial port {self.port}: {e}")
            return False
        return True

    def stop(self):
        self._stop_event.set()

    def write(self, s):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write((s + "\n").encode())
        except Exception as e:
            self.error_callback(f"Serial write error: {e}")

    def run(self):
        try:
            if not self.open():
                return
            while not self._stop_event.is_set():
                try:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    parts = line.split(",")
                    if len(parts) < 2:
                        continue
                    try:
                        millis = float(parts[0].strip())
                        raw = float(parts[1].strip())
                    except:
                        continue
                    self.data_callback(millis, raw)
                except Exception as e_inner:
                    self.error_callback(f"Serial read error: {e_inner}")
                    time.sleep(0.05)
        finally:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

# Main GUI app
class TamperApp:
    def __init__(self, root):
        self.root = root
        root.title("Scale Logger - Settle + Tamper Detection")

        self.build_ui()

        # timing & state
        self.start_time = time.time()
        self.last_tamper_time = 0.0
        self.tamper_cooldown = TAMPER_COOLDOWN_SEC
        self.tamper_active = False

        # buffers & filters
        self.readavg_buf = deque(maxlen=READAVG_N)
        self.sma_buf = deque(maxlen=SMA_WINDOW)
        self.median_buf = deque(maxlen=MEDIAN_WINDOW)
        self.exp_prev = None
        self.butter_b, self.butter_a = butter_lowpass(BUTTER_CUTOFF_HZ, SAMPLE_RATE_HZ, BUTTERORDER)

        self.millis_list = []
        self.raw_list = []
        self.readavg_list = []
        self.sma_list = []
        self.exp_list = []
        self.median_list = []
        self.sg_list = []
        self.butter_list = []

        self.prev_raw = None
        self.prev_block_mean = None
        self.block_buffer = []

        self.thread = None
        self.is_logging = False

        # settle (weight placement) state
        self.settling = False
        self.settle_end_time = 0.0
        self.last_weight_mean = None

        # baseline and deflection state
        self.baseline_var = 800.0
        self.slope_exceed_count = 0
        self.cusum_pos = 0.0
        self.cusum_neg = 0.0
        self.cusum_ref = None

        self.serial_thread = None

        # plot
        self.fig, self.ax = plt.subplots(figsize=(7,3.5))
        self.ax.set_title("Raw ADC (blue) vs Filtered (orange)")
        self.ax.set_xlabel("samples")
        self.ax.set_ylabel("ADC counts")
        self.raw_line, = self.ax.plot([], [], label="raw")
        self.filter_line, = self.ax.plot([], [], label="exp-smoothed")
        self.ax.legend()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.set_indicator_color("grey")
        self.root.after(200, self.gui_update)

    def build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill=tk.X)

        self.start_btn = ttk.Button(top, text="Start Logging", command=self.start_logging)
        self.start_btn.pack(side=tk.LEFT, padx=4)
        self.stop_btn = ttk.Button(top, text="Stop Logging", command=self.stop_logging, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=4)

        self.learn_btn = ttk.Button(top, text="Learn Baseline", command=self.learn_baseline_gui)
        self.learn_btn.pack(side=tk.LEFT, padx=4)

        self.ack_btn = ttk.Button(top, text="Acknowledge", command=self.acknowledge_alarm, state=tk.DISABLED)
        self.ack_btn.pack(side=tk.LEFT, padx=4)

        self.status_label = ttk.Label(top, text="Status: Idle")
        self.status_label.pack(side=tk.LEFT, padx=8)

        self.indicator_canvas = tk.Canvas(top, width=22, height=22, highlightthickness=0)
        self.indicator_canvas.pack(side=tk.LEFT, padx=6)
        self.indicator_circle = self.indicator_canvas.create_oval(2, 2, 20, 20, fill="grey", outline="black")

        self.zip_var = tk.BooleanVar(value=True)
        self.zip_check = ttk.Checkbutton(top, text=f"Auto-zip > {ZIP_AFTER_DAYS} days", variable=self.zip_var)
        self.zip_check.pack(side=tk.LEFT, padx=8)

        self.delete_after_var = tk.BooleanVar(value=False)
        self.delete_after_check = ttk.Checkbutton(top, text="Delete originals after zip", variable=self.delete_after_var)
        self.delete_after_check.pack(side=tk.LEFT, padx=8)

        self.port_label = ttk.Label(top, text=f"Port: {SERIAL_PORT}")
        self.port_label.pack(side=tk.RIGHT, padx=8)

        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        bottom = ttk.Frame(self.root, padding=6)
        bottom.pack(fill=tk.X)
        self.log_text = tk.Text(bottom, height=8, state=tk.DISABLED)
        self.log_text.pack(fill=tk.X)

    def log(self, *args):
        msg = " ".join(str(a) for a in args)
        print(msg)
        try:
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.insert("end", msg + "\n")
            self.log_text.see("end")
            self.log_text.configure(state=tk.DISABLED)
        except Exception:
            pass

    def set_indicator_color(self, color):
        try:
            self.indicator_canvas.itemconfig(self.indicator_circle, fill=color)
        except Exception:
            pass

    def _blink_toggle(self):
        if not self.blinking:
            return
        self.indicator_on = not self.indicator_on
        color = "red" if self.indicator_on else "grey"
        self.set_indicator_color(color)
        self.blink_job = self.root.after(500, self._blink_toggle)

    def start_blink(self, duration_s=None):
        if getattr(self, "blinking", False):
            return
        self.blinking = True
        self.ack_btn["state"] = tk.NORMAL
        self.indicator_on = False
        self.blink_job = self.root.after(0, self._blink_toggle)
        if duration_s is not None:
            self.root.after(int(duration_s * 1000), lambda: self.stop_blink())

    def stop_blink(self):
        if getattr(self, "blink_job", None):
            try:
                self.root.after_cancel(self.blink_job)
            except Exception:
                pass
        self.blink_job = None
        self.blinking = False
        self.indicator_on = False
        self.ack_btn["state"] = tk.DISABLED
        if self.is_logging:
            self.set_indicator_color("yellow")
        else:
            self.set_indicator_color("grey")

    def acknowledge_alarm(self):
        self.log("Alarm acknowledged by operator.")
        self.tamper_active = False
        self.cusum_pos = 0.0
        self.cusum_neg = 0.0
        self.stop_blink()

    def serial_write(self, msg):
        try:
            if self.serial_thread and isinstance(self.serial_thread, SerialLoggerThread):
                self.serial_thread.write(msg)
        except Exception as e:
            self.log("Serial write failed:", e)

    def save_tamper_snapshot(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        snap_csv = OUT_DIR / f"tamper_snapshot_{ts}.csv"
        snap_xlsx = OUT_DIR / f"tamper_snapshot_{ts}.xlsx"

        window = 500
        data = {
            "millis": self.millis_list[-window:],
            "raw": self.raw_list[-window:],
            "exp": self.exp_list[-window:]
        }
        df = pd.DataFrame(data)
        try:
            df.to_csv(snap_csv, index=False)
            df.to_excel(snap_xlsx, index=False)
            self.log("Tamper snapshot saved:", snap_csv.name, snap_xlsx.name)
        except Exception as e:
            self.log("Error saving tamper snapshot:", e)

    def show_tamper_popup_once(self, title, message):
        def popup():
            try:
                messagebox.showerror(title, message)
            except Exception:
                pass
        self.root.after(0, popup)

    def data_callback(self, millis, raw):
        # fast path in background thread
        try:
            # startup suppression: ignore noisy initial samples but warm buffers
            if time.time() - self.start_time < STARTUP_SUPPRESS_SEC:
                self.millis_list.append(millis)
                self.raw_list.append(raw)
                self.prev_raw = raw
                # warm filters
                self.readavg_buf.append(raw)
                return

            # append raw data & update base lists
            self.millis_list.append(millis)
            self.raw_list.append(raw)

            # read-average
            self.readavg_buf.append(raw)
            avg_read = sum(self.readavg_buf) / len(self.readavg_buf)
            self.readavg_list.append(avg_read)

            # SMA
            self.sma_buf.append(avg_read)
            sma_v = sum(self.sma_buf) / len(self.sma_buf)
            self.sma_list.append(sma_v)

            # exponential smoothing
            if self.exp_prev is None:
                exp_val = sma_v
            else:
                exp_val = EXP_ALPHA * sma_v + (1.0 - EXP_ALPHA) * self.exp_prev
            self.exp_prev = exp_val
            self.exp_list.append(exp_val)

            # median
            self.median_buf.append(avg_read)
            med_v = np.median(np.array(self.median_buf))
            self.median_list.append(med_v)

            # savgol
            if len(self.raw_list) >= SG_WINDOW:
                try:
                    sg_val = savgol_filter(np.array(self.raw_list[-SG_WINDOW:]), SG_WINDOW, SG_POLY)[-1]
                except Exception:
                    sg_val = avg_read
            else:
                sg_val = avg_read
            self.sg_list.append(sg_val)

            # butterworth block
            butter_window = 128
            if len(self.raw_list) >= butter_window:
                try:
                    block = np.array(self.raw_list[-butter_window:])
                    filt_block = filtfilt(self.butter_b, self.butter_a, block, padlen=15)
                    butter_val = filt_block[-1]
                except Exception:
                    butter_val = exp_val
            else:
                butter_val = exp_val
            self.butter_list.append(butter_val)

            # if currently settling due to weight placement, check end
            if getattr(self, "settling", False):
                if time.time() >= self.settle_end_time:
                    # end settling: set new cusum_ref and re-arm
                    self.settling = False
                    if self.last_weight_mean is not None:
                        self.cusum_ref = float(self.last_weight_mean)
                    else:
                        if len(self.raw_list) >= 100:
                            self.cusum_ref = float(np.mean(self.raw_list[-100:]))
                    self.cusum_pos = 0.0
                    self.cusum_neg = 0.0
                    self.slope_exceed_count = 0
                    self.log("[SETTLE-END] Settling ended; detectors re-armed.")
                else:
                    # still settling: suppress tamper detectors
                    # we already updated filters and lists above, so return early
                    return

            # spike detection (instant)
            try:
                if self.prev_raw is not None and abs(raw - self.prev_raw) > TAMPER_SPIKE_THRESHOLD:
                    self.log(f"[TAMPER-SPIKE] {self.prev_raw:.1f} -> {raw:.1f}")
                    self._trigger_tamper_event(reason="spike")
            except Exception as e:
                self.log("Spike detect error:", e)

            self.prev_raw = raw

            # block step detection (improved - detect weight placement vs tamper)
            self.block_buffer.append(raw)
            if len(self.block_buffer) >= BLOCK_SIZE:
                try:
                    cur_block_mean = float(np.mean(self.block_buffer))
                    if self.prev_block_mean is not None:
                        delta = cur_block_mean - self.prev_block_mean
                        # large positive step -> weight placement
                        if delta > WEIGHT_PLACEMENT_DELTA:
                            self.last_weight_mean = cur_block_mean
                            self.settling = True
                            self.settle_end_time = time.time() + SETTLE_AFTER_WEIGHT_SEC
                            self.log(f"[WEIGHT-PLACED] mean {self.prev_block_mean:.1f} -> {cur_block_mean:.1f}; settling {SETTLE_AFTER_WEIGHT_SEC}s")
                            # clear transient state
                            self.cusum_pos = 0.0
                            self.cusum_neg = 0.0
                            self.slope_exceed_count = 0
                            # don't trigger tamper here
                        else:
                            # if currently settling, ignore small fluctuations
                            if self.settling:
                                self.log(f"[SETTLING] suppressed step delta={delta:.1f}")
                            else:
                                if abs(delta) > TAMPER_STEP_THRESHOLD:
                                    self.log(f"[TAMPER-STEP] block mean changed {self.prev_block_mean:.1f} -> {cur_block_mean:.1f}")
                                    self._trigger_tamper_event(reason="step")
                    self.prev_block_mean = cur_block_mean
                except Exception as e:
                    self.log("Block step error:", e)
                finally:
                    self.block_buffer = []

            # --------------------------
            # DEFLECTION DETECTION CODE (only runs when not settling)
            # --------------------------
            # 1) Rolling slope detector
            try:
                if len(self.raw_list) >= SLOPE_WINDOW:
                    window = np.array(self.raw_list[-SLOPE_WINDOW:])
                    x = np.arange(len(window))
                    wx = x - x.mean()
                    denom = (wx * wx).sum()
                    if denom != 0:
                        slope = (wx * (window - window.mean())).sum() / denom
                    else:
                        slope = 0.0
                    slope_counts_per_sec = slope * SAMPLE_RATE_HZ

                    if abs(slope) > SLOPE_THRESH:
                        self.slope_exceed_count += 1
                    else:
                        self.slope_exceed_count = max(0, self.slope_exceed_count - 1)

                    if self.slope_exceed_count >= SLOPE_DEBOUNCE:
                        msg = f"[DEFLECTION-SLOPE] slope={slope:.3f} counts/sample ({slope_counts_per_sec:.1f} counts/s)"
                        self.log(msg)
                        self._trigger_tamper_event(reason="slope", message=msg)
                        self.slope_exceed_count = 0
            except Exception as e:
                self.log("Slope detection error:", e)

            # 2) CUSUM detector
            try:
                if self.cusum_ref is None and len(self.raw_list) >= 100:
                    self.cusum_ref = float(np.mean(self.raw_list[-100:]))

                if self.cusum_ref is not None:
                    latest = float(self.raw_list[-1])
                    s_pos = latest - self.cusum_ref - CUSUM_K
                    self.cusum_pos = max(0.0, self.cusum_pos + s_pos)
                    s_neg = -(latest - self.cusum_ref) - CUSUM_K
                    self.cusum_neg = max(0.0, self.cusum_neg + s_neg)

                    if self.cusum_pos > CUSUM_H:
                        msg = f"[CUSUM POS] cusum_pos={self.cusum_pos:.1f} ref={self.cusum_ref:.1f}"
                        self.log(msg)
                        self._trigger_tamper_event(reason="cusum_pos", message=msg)
                        self.cusum_pos = 0.0
                        self.cusum_neg = 0.0

                    if self.cusum_neg > CUSUM_H:
                        msg = f"[CUSUM NEG] cusum_neg={self.cusum_neg:.1f} ref={self.cusum_ref:.1f}"
                        self.log(msg)
                        self._trigger_tamper_event(reason="cusum_neg", message=msg)
                        self.cusum_pos = 0.0
                        self.cusum_neg = 0.0
            except Exception as e:
                self.log("CUSUM detection error:", e)

        except Exception as e:
            self.log("Data callback error:", e)
            traceback.print_exc()

    def _trigger_tamper_event(self, reason="unknown", message=None):
        now = time.time()
        # cooldown: prevent multiple snapshots/popups in short time
        if now - self.last_tamper_time < self.tamper_cooldown:
            self.log(f"Tamper event '{reason}' suppressed by cooldown.")
            return
        if self.tamper_active:
            self.log(f"Tamper event '{reason}' ignored because tamper is already active.")
            return

        self.tamper_active = True
        self.last_tamper_time = now

        self.root.after(0, lambda: self.start_blink(duration_s=None))

        if ENABLE_SERIAL_BUZZER:
            try:
                self.serial_write("BUZZER_ON")
            except Exception:
                pass

        try:
            self.save_tamper_snapshot()
        except Exception as e:
            self.log("Snapshot saving error:", e)

        popup_msg = message if message else f"Tamper detected: {reason}"
        self.show_tamper_popup_once("TAMPER ALERT", popup_msg)

    def error_callback(self, msg):
        self.log("ERROR:", msg)
        self.root.after(10, lambda: messagebox.showerror("Serial Error", msg))

    def start_logging(self):
        if self.is_logging:
            return
        # clear
        self.millis_list.clear()
        self.raw_list.clear()
        self.readavg_list.clear()
        self.sma_list.clear()
        self.exp_list.clear()
        self.median_list.clear()
        self.sg_list.clear()
        self.butter_list.clear()
        self.readavg_buf.clear()
        self.sma_buf.clear()
        self.median_buf.clear()
        self.exp_prev = None
        self.prev_raw = None
        self.prev_block_mean = None
        self.block_buffer = []
        self.slope_exceed_count = 0
        self.cusum_pos = 0.0
        self.cusum_neg = 0.0
        self.cusum_ref = None
        self.start_time = time.time()
        self.last_tamper_time = 0.0
        self.tamper_active = False
        self.settling = False
        self.settle_end_time = 0.0
        self.last_weight_mean = None

        self.serial_thread = SerialLoggerThread(SERIAL_PORT, BAUD, TIMEOUT, self.data_callback, self.error_callback)
        self.serial_thread.start()

        self.is_logging = True
        self.start_btn["state"] = tk.DISABLED
        self.stop_btn["state"] = tk.NORMAL
        self.status_label["text"] = "Status: Logging"
        self.set_indicator_color("yellow")
        self.log("Started logging on", SERIAL_PORT)

    def stop_logging(self):
        if not self.is_logging:
            return
        try:
            if self.serial_thread:
                self.serial_thread.stop()
                self.serial_thread.join(timeout=2.0)
        except Exception:
            pass
        self.is_logging = False
        self.start_btn["state"] = tk.NORMAL
        self.stop_btn["state"] = tk.DISABLED
        self.status_label["text"] = "Status: Stopped"
        self.tamper_active = False
        self.stop_blink()
        self.set_indicator_color("grey")
        self.log("Stopped logging. Saving files...")
        self.save_and_finalize()

    def save_and_finalize(self):
        self.stop_blink()

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_csv = OUT_DIR / f"scale_log_{ts}.csv"
        out_xlsx = OUT_DIR / f"scale_log_{ts}.xlsx"

        df = pd.DataFrame({
            "millis": self.millis_list,
            "raw": self.raw_list,
            "readavg": self.readavg_list,
            "sma": self.sma_list,
            "exp": self.exp_list,
            "median": self.median_list,
            "savgol": self.sg_list,
            "butter": self.butter_list
        })

        try:
            df.to_csv(out_csv, index=False)
            df.to_excel(out_xlsx, index=False)
            self.log("Saved:", out_csv.name, out_xlsx.name)
            try:
                if sys.platform.startswith("win"):
                    os.startfile(str(out_xlsx))
                elif sys.platform.startswith("darwin"):
                    os.system(f"open \"{out_xlsx}\"")
            except Exception as e:
                self.log("Could not auto-open file:", e)
        except Exception as e:
            self.log("Error saving files:", e)
            traceback.print_exc()

        if self.zip_var.get():
            try:
                self.auto_zip_old_logs(days=ZIP_AFTER_DAYS, keep_recent=KEEP_RECENT_FILES, delete_after=self.delete_after_var.get())
            except Exception as ex:
                self.log("Auto-zip error:", ex)
        self.log("Done finalizing.")

    def auto_zip_old_logs(self, days=7, keep_recent=30, delete_after=False):
        now = datetime.now()
        cutoff = now - timedelta(days=days)

        csv_files = sorted(OUT_DIR.glob("scale_log_*.csv"), key=lambda p: p.stat().st_mtime)
        xlsx_files = sorted(OUT_DIR.glob("scale_log_*.xlsx"), key=lambda p: p.stat().st_mtime)

        by_stem = {}
        for p in csv_files + xlsx_files:
            stem = p.stem
            if stem not in by_stem:
                by_stem[stem] = []
            by_stem[stem].append(p)

        stem_items = []
        for stem, files in by_stem.items():
            mtime = min([f.stat().st_mtime for f in files])
            stem_items.append((stem, files, mtime))
        stem_items.sort(key=lambda x: x[2])

        all_files_sorted = []
        for stem, files, _ in stem_items:
            files_sorted = sorted(files, key=lambda p: p.suffix)
            all_files_sorted.extend(files_sorted)

        if keep_recent and len(all_files_sorted) > keep_recent:
            recent_stems = set([p.stem for p in all_files_sorted[-keep_recent:]])
            candidates = [(stem, files) for (stem, files, _) in stem_items if stem not in recent_stems]
        else:
            candidates = [(stem, files) for (stem, files, _) in stem_items]

        to_zip_stems = []
        for stem, files in candidates:
            oldest_mtime = min([p.stat().st_mtime for p in files])
            if datetime.fromtimestamp(oldest_mtime) < cutoff:
                to_zip_stems.append((stem, files))

        if not to_zip_stems:
            self.log("No old CSV/XLSX pairs to zip.")
            return

        archive_dir = OUT_DIR / "archives"
        archive_dir.mkdir(exist_ok=True)
        archive_name = archive_dir / f"logs_{now.strftime('%Y%m%d_%H%M%S')}.zip"

        zipped_count = 0
        with zipfile.ZipFile(archive_name, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            for stem, files in to_zip_stems:
                files_sorted = sorted(files, key=lambda p: p.suffix)
                for p in files_sorted:
                    try:
                        zf.write(p, arcname=p.name)
                        zipped_count += 1
                    except Exception as e:
                        self.log(f"Failed to add {p.name} to zip: {e}")

        self.log(f"Zipped {len(to_zip_stems)} stem(s) ({zipped_count} files) into {archive_name.name}")

        if delete_after:
            deleted = 0
            for stem, files in to_zip_stems:
                for p in files:
                    try:
                        p.unlink()
                        deleted += 1
                    except Exception as e:
                        self.log(f"Could not delete {p.name}: {e}")
            self.log(f"Deleted {deleted} original files after zipping.")

    def compute_baseline(self, sample_window=3000):
        if len(self.raw_list) < sample_window:
            self.log(f"Not enough data to compute baseline. Need {sample_window} samples, have {len(self.raw_list)}")
            return

        data = np.array(self.raw_list[-sample_window:])
        baseline_var = float(np.var(data))

        global TAMPER_SPIKE_THRESHOLD, TAMPER_STEP_THRESHOLD, SLOPE_THRESH, CUSUM_K, CUSUM_H
        TAMPER_SPIKE_THRESHOLD = int(max(100, np.max(np.abs(np.diff(data))) * 4))
        TAMPER_STEP_THRESHOLD = int(max(100, np.std(data) * 10))

        mad = np.median(np.abs(np.diff(data)))
        SLOPE_THRESH = max(0.2, mad * 0.5)
        CUSUM_K = max(1.0, mad * 1.0)
        CUSUM_H = max(200.0, np.std(data) * 20.0)

        self.baseline_var = baseline_var
        self.cusum_ref = float(np.mean(data))

        self.log("Baseline variance learned:", baseline_var)
        self.log("Updated TAMPER_SPIKE_THRESHOLD:", TAMPER_SPIKE_THRESHOLD)
        self.log("Updated TAMPER_STEP_THRESHOLD:", TAMPER_STEP_THRESHOLD)
        self.log(f"Suggested SLOPE_THRESH={SLOPE_THRESH:.3f}, CUSUM_K={CUSUM_K:.1f}, CUSUM_H={CUSUM_H:.1f}")

    def learn_baseline_gui(self):
        self.compute_baseline(sample_window=3000)
        messagebox.showinfo("Baseline Updated", "Baseline thresholds updated successfully.")

    def gui_update(self):
        try:
            NPOINTS = 300
            xs = np.arange(max(0, len(self.raw_list[-NPOINTS:])))
            raw_y = self.raw_list[-NPOINTS:]
            filt_y = self.exp_list[-NPOINTS:]
            if len(raw_y) > 0:
                self.raw_line.set_data(xs, raw_y)
                self.filter_line.set_data(xs, filt_y)
                ymin = min(min(raw_y), min(filt_y)) - 50
                ymax = max(max(raw_y), max(filt_y)) + 50
                self.ax.set_xlim(0, max(100, len(xs)))
                self.ax.set_ylim(ymin, ymax)
                self.canvas.draw_idle()
        except Exception as e:
            print("Plot update error:", e)
        finally:
            self.root.after(200, self.gui_update)

def main():
    root = tk.Tk()
    app = TamperApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: on_closing(root, app))
    root.mainloop()

def on_closing(root, app):
    if app.is_logging:
        if messagebox.askyesno("Exit", "Logging is active. Stop and save before exit?"):
            app.stop_logging()
            root.destroy()
        else:
            return
    else:
        root.destroy()

if __name__ == "__main__":
    main()

# File: GUI.py
import PySimpleGUI as sg
import serial
import serial.tools.list_ports
import threading
import queue
import sys
import math
import time
import binascii
import time
from typing import List, Tuple, Dict, Optional

# ------- File Mode helpers (HEX + constants) -------
def _hex_from_bytes(b: bytes) -> str:
    # encode to uppercase hex without spaces
    return b.hex().upper()

def _bytes_from_hex(s: str) -> bytes:
    # robust hex decode; raises ValueError if bad
    return bytes.fromhex(s.strip())

FILE_TX_CHUNK = 64   # bytes per FILE,PUT,CHUNK (<= firmware tmp[96])
FILE_RX_LIMIT = 2048  # Phase-1 FW buffer limit; enforce in GUI

# Matplotlib (for the half-circle “radar” view)
try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

BAUD = 9600
READ_TIMEOUT_S = 1.0   # short, reader thread loops frequently
LDR_VCC_MV = 3300
INTER_CHUNK_DELAY_S = 0.05      # small pacing so MCU can breathe
ACK_TIMEOUT_S = 4.0             # a bit more headroom for ACKs
SPINNER = ["⠋","⠙","⠹","⠸","⠼","⠴","⠦","⠧","⠇","⠏"]

CHUNK_BYTES = 64             # ≤ 96 (device LINE_MAX). 64 keeps lines short.
READ_TIMEOUT = 2.0           # seconds per line

class FileClient:
    def __init__(self, ser, log_fn=print, progress_cb=None):
        """
        ser: a pyserial.Serial instance already opened at 9600 8N1
        log_fn: callable(str) for GUI log area
        """
        self.ser = ser
        self.log = log_fn
        self.progress_cb = progress_cb

    # ---------------- low-level helpers ----------------
    def _writeln(self, s: str) -> None:
        self.ser.write((s + "\r\n").encode("ascii"))

    def _readline(self, timeout: float = READ_TIMEOUT) -> Optional[str]:
        deadline = time.time() + timeout
        buf = bytearray()
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b == b'\n':
                line = buf.decode("ascii", errors="ignore").rstrip("\r")
                # skip heartbeats / boot-ready / echo / stray boot byte(s)
                if (not line) or line in ("HB", "READY", "B") or line.startswith("ECHO,"):
                    buf.clear()
                    continue
                return line
            elif b != b'\r':
                buf += b
        return None

    def _expect_ok(self, expect_prefix: str, timeout: float = ACK_TIMEOUT_S) -> None:
        """Wait until a line starting with expect_prefix arrives, ignoring HB/READY/ECHO noise."""
        line = self._read_until_prefix([expect_prefix], overall_timeout=timeout)
        if line is None:
            raise RuntimeError(f"Expected '{expect_prefix}…', got: None")

    # ---------------- simple commands ----------------
    def ls(self) -> List[Tuple[str, int]]:
        """Returns [(name, size), …]"""
        self._writeln("FILE,LS")
        line = self._readline()
        if line is None or not line.startswith("FILE,LS,"):
            raise RuntimeError(f"Bad LS response: {line!r}")
        # format: FILE,LS,<count>,name:size;name2:size2
        parts = line.split(",", 3)
        if len(parts) < 4:
            return []
        names = parts[3]
        entries = []
        if names:
            for item in names.split(";"):
                if not item:
                    continue
                nm, sz = item.split(":")
                entries.append((nm, int(sz)))
        return entries

    def get(self, name: str) -> bytes:
        """Download a file's raw bytes."""
        self._writeln(f"FILE,GET,{name}")
        line = self._readline()
        if line is None or not line.startswith("FILE,GET,BEGIN,"):
            raise RuntimeError(f"GET begin failed: {line!r}")

        # Collect DATA lines until END
        out = bytearray()
        while True:
            line = self._readline()
            if line is None:
                raise RuntimeError("GET timed out")
            if line == "FILE,GET,END,OK":
                break
            if line.startswith("FILE,GET,DATA,"):
                hexpart = line[len("FILE,GET,DATA,"):]
                out += binascii.unhexlify(hexpart.encode("ascii"))
            else:
                raise RuntimeError(f"Unexpected GET line: {line}")
        return bytes(out)

    def format(self) -> None:
        # Clear any pending input, fire FORMAT
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._writeln("FILE,FORMAT")

        # Let the board reboot / remount FS and stop chattering (HB/READY/etc).
        t0 = time.time()
        last_rx = t0
        while time.time() - t0 < 5.0:
            b = self.ser.read(1)
            if b:
                last_rx = time.time()
                continue
            if time.time() - last_rx > 0.6:
                break

        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        # Poll for empty directory. Trust ONLY the count field.
        deadline = time.time() + 25.0
        last = None
        while time.time() < deadline:
            time.sleep(0.25)
            self._writeln("FILE,LS")
            line = self._readline(timeout=1.5)  # ignores HB/READY/ECHO already
            if not line:
                continue
            last = line
            if line.startswith("FILE,LS,"):
                # format: FILE,LS,<count>,<maybe-long-names…(can be truncated)>
                parts = line.split(",", 3)
                if len(parts) >= 3:
                    try:
                        cnt = int(parts[2])
                    except ValueError:
                        cnt = None
                    if cnt == 0:
                        return  # formatted and empty — don't care about names tail
        raise RuntimeError(f"FORMAT did not complete; last reply: {last!r}")

    # ---------------- PUT with chunking ----------------
    def _put_begin(self, name: str, ftype: int, total: int) -> str:
        self._writeln(f"FILE,PUT,BEGIN,{name},{ftype},{total}")
        line = self._read_until_prefix(
            ["FILE,PUT,BEGIN,OK", "FILE,PUT,ERR,NEEDS_REBUILD", "FILE,PUT,ERR,"],
            overall_timeout=ACK_TIMEOUT_S
        )
        if line is None:
            raise RuntimeError("PUT,BEGIN no response")
            # NEW: recover from a stale session
        if line.startswith("FILE,PUT,ERR,4"):  # FM_ERR_BUSY
            try:
                self._writeln("FILE,PUT,ABORT")
                _ = self._read_until_prefix(
                    ["FILE,PUT,ABORT,OK", "FILE,PUT,ERR"],
                    overall_timeout=1.0
                )
            except Exception:
                pass
            # retry once
            self._writeln(f"FILE,PUT,BEGIN,{name},{ftype},{total}")
            line = self._read_until_prefix(
                ["FILE,PUT,BEGIN,OK", "FILE,PUT,ERR,NEEDS_REBUILD", "FILE,PUT,ERR,"],
                overall_timeout=ACK_TIMEOUT_S
            )

        if line.startswith("FILE,PUT,BEGIN,OK"):
            return line
        if line.startswith("FILE,PUT,ERR,NEEDS_REBUILD"):
            return "NEEDS_REBUILD"
        raise RuntimeError(f"PUT,BEGIN error: {line}")

    def _put_chunk(self, chunk: bytes) -> None:
        self._writeln("FILE,PUT,CHUNK," + binascii.hexlify(chunk).decode("ascii").upper())
        self._expect_ok("FILE,PUT,CHUNK,OK")

    def _put_end(self) -> None:
        self._writeln("FILE,PUT,END")
        self._expect_ok("FILE,PUT,END,OK")

    def put(self, name: str, ftype: int, data: bytes) -> None:
        """Single upload attempt (no rebuild)."""
        st = self._put_begin(name, ftype, len(data))
        if st == "NEEDS_REBUILD":
            raise RuntimeError("NEEDS_REBUILD")

        # stream chunks with progress callbacks
        total = len(data)
        total_chunks = (total + CHUNK_BYTES - 1) // CHUNK_BYTES

        # initial progress (0/N)
        if getattr(self, "progress_cb", None):
            try:
                self.progress_cb(0, total_chunks)
            except Exception:
                pass

        pos = 0
        chunk_idx = 0
        while pos < total:
            self._put_chunk(data[pos:pos + CHUNK_BYTES])
            pos += CHUNK_BYTES
            chunk_idx += 1
            if getattr(self, "progress_cb", None):
                try:
                    done = chunk_idx if chunk_idx <= total_chunks else total_chunks
                    self.progress_cb(done, total_chunks)
                except Exception:
                    pass

        self._put_end()

    def _read_until_prefix(self, prefixes, overall_timeout: float) -> Optional[str]:
        deadline = time.time() + overall_timeout
        while time.time() < deadline:
            line = self._readline(timeout=0.5)
            if not line:
                continue
            for p in prefixes:
                if line.startswith(p):
                    return line
            # otherwise ignore and keep waiting (echo/heartbeats/etc are already skipped)
        return None

    # ---------------- public: upload with auto-rebuild ----------------
    def upload_with_rebuild(self, name: str, ftype: int, data: bytes,
                            backup_to_disk_dir: Optional[str] = None) -> None:
        """
        Try to PUT; if device says NEEDS_REBUILD, do:
          LS → GET all → (optional: save to disk) → FORMAT → re-PUT all → retry
        """
        try:
            self.log(f"PUT {name} ({len(data)} bytes)…")
            self.put(name, ftype, data)
            self.log("PUT done.")
            return
        except RuntimeError as e:
            if "NEEDS_REBUILD" not in str(e):
                raise

        self.log("Device requested rebuild; backing up existing files…")
        # 1) List and fetch all other files
        entries = self.ls()
        snapshots: Dict[str, Tuple[int, bytes]] = {}
        for (nm, sz) in entries:
            if nm == name:
                # back up even same name, in case we’re retrying after a previous failure
                pass
            self.log(f"GET {nm} ({sz} bytes)…")
            payload = self.get(nm)
            if len(payload) != sz:
                raise RuntimeError(f"Size mismatch reading {nm}")
            ftype_guess = 0 if nm.lower().endswith(".txt") else 1
            snapshots[nm] = (ftype_guess, payload)

        # Let caller override file types per their UI if needed.
        # For now: text=0, script=1. If you keep types in the GUI, map them back here.

        # Optional: write a quick disk backup
        if backup_to_disk_dir:
            import os
            os.makedirs(backup_to_disk_dir, exist_ok=True)
            for nm, (_, blob) in snapshots.items():
                with open(os.path.join(backup_to_disk_dir, nm), "wb") as f:
                    f.write(blob)
            self.log(f"Backed up {len(snapshots)} file(s) to {backup_to_disk_dir}")

        # 2) Format
        self.log("Formatting device…")
        try:
            self.format()  # your count-only version
        except Exception as e:
            self.log(f"FORMAT failed or didn’t complete ({e}).")
            # ← Fallback path that doesn’t depend on long FILE,LS lines:
            self._wipe_all_files_slow()

        # Verify we really got an empty FS; retry once if not.
        try:
            after = self.ls()
        except Exception:
            after = []  # if MCU just rebooted, ls() may fail once

        if after:
            self.log(f"Format didn’t clear FS ({len(after)} file(s) remain). Retrying once…")
            self.format()
            after = self.ls()
            if after:
                raise RuntimeError("FORMAT did not take; aborting to avoid rebuild loops.")

        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        # 3) Re-upload backups (smallest→largest helps reduce chance of new rebuilds)
        restore_order = sorted(snapshots.items(), key=lambda kv: len(kv[1][1]))
        for nm, (ftype_rest, blob) in restore_order:
            # heuristic: infer type from extension; adjust if your GUI tracks it elsewhere
            if ftype_rest not in (0, 1):
                ftype_rest = 1 if nm.lower().endswith(".txt") is False else 0
            self.log(f"RESTORE {nm} ({len(blob)} bytes)…")
            self.put(nm, ftype_rest, blob)

        # 4) Retry the requested upload last
        self.log(f"Retrying PUT {name}…")
        self.put(name, ftype, data)
        self.log("Rebuild + upload complete.")

    def _ls_count_and_first(self) -> Tuple[Optional[int], Optional[str]]:
        """Return (count, first_name) from FILE,LS. If no names or parse issues, first_name may be None."""
        self._writeln("FILE,LS")
        line = self._readline(timeout=1.5)
        if not line or not line.startswith("FILE,LS,"):
            return (None, None)
        parts = line.split(",", 3)  # FILE,LS,<count>,<names...>
        cnt = None
        try:
            cnt = int(parts[2])
        except Exception:
            pass
        first = None
        if len(parts) >= 4 and parts[3]:
            first_item = parts[3].split(";", 1)[0]  # only the first entry
            if first_item:
                first = first_item.split(":", 1)[0]  # strip size if present
        return (cnt, first)

    def _wipe_all_files_slow(self, max_iters: int = 64) -> None:
        """
        FORMAT fallback: repeatedly delete the first file shown by FILE,LS
        until the count returns 0. Robust to truncated long name lists.
        """
        self.log("Falling back: deleting files one-by-one…")
        for _ in range(max_iters):
            cnt, first = self._ls_count_and_first()
            if cnt == 0:
                self.log("Device is empty.")
                return
            if cnt is None:
                time.sleep(0.25)
                continue
            if not first:
                # no first name yet; give the MCU a moment
                time.sleep(0.25)
                continue
            self._writeln(f"FILE,DEL,{first}")
            resp = self._read_until_prefix(["FILE,DEL,OK", "FILE,DEL,ERR"], overall_timeout=3.0)
            if not resp:
                continue
            if resp.startswith("FILE,DEL,OK"):
                self.log(f"Deleted {first}")
            else:
                self.log(f"Delete failed for {first}: {resp}")
                time.sleep(0.25)
        raise RuntimeError("Fallback delete-all did not finish (max iterations).")

# --- end: file manager for MSP430 protocol ---

# ----------------------- Serial helpers -----------------------
def find_msp430_port():
    """Return first COM port that looks like MSP430/TI; else None."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        manu = (p.manufacturer or "").lower()
        if "msp" in desc or "texas instruments" in manu or "launchpad" in desc:
            return p.device
    return None

def list_all_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

class MCU:
    def __init__(self, rx_queue: queue.Queue):
        self.ser = None
        self.lock = threading.Lock()
        self.rx_q = rx_queue
        self.running = False
        self.reader_th = None

    def open(self, port):
        self.close()
        self.ser = serial.Serial(
            port=port, baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=READ_TIMEOUT_S,
            write_timeout=2.0,
            rtscts=False, dsrdtr=False
        )
        # Toggle DTR to reset board on many LaunchPads (safe on others)
        try:
            self.ser.setDTR(False)
            time.sleep(0.05)
            self.ser.setDTR(True)
        except Exception:
            pass
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # Start background reader
        self.running = True
        self.reader_th = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_th.start()

    def _reader_loop(self):
        buf = b""
        while self.running and self.ser and self.ser.is_open:
            try:
                # read up to 256 bytes per tick (respects timeout)
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk

                # process complete lines
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    s = line.decode("ascii", errors="ignore").rstrip('\r')
                    if s:
                        self.rx_q.put(("rx", s))
            except Exception:
                break

        # flush any leftover if port closes mid-line
        if buf:
            s = buf.decode("ascii", errors="ignore").strip()
            if s:
                self.rx_q.put(("rx", s))

    # --- allow temporarily taking over the serial port (for FileClient) ---
    def pause_reader(self):
        """Stop background reader thread but keep port open."""
        if not self.running:
            return
        self.running = False
        if self.reader_th:
            try:
                self.reader_th.join(timeout=1.0)
            except Exception:
                pass
            self.reader_th = None

    def resume_reader(self):
        """Restart background reader thread if port is open."""
        if self.running or not (self.ser and self.ser.is_open):
            return
        self.running = True
        self.reader_th = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_th.start()

    def close(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send_line(self, text):
        if not self.is_open():
            raise RuntimeError("Serial not open")
        if not text.endswith("\n"):
            text += "\n"
        data = text.encode("ascii", errors="ignore")
        with self.lock:
            self.ser.write(data)
            self.ser.flush()
        # also mirror to log (TX)
        self.rx_q.put(("tx", text.strip()))

    def _read_line(self, timeout=1.0):
        """Read one CR/LF-terminated line, ASCII-decoded, without the trailing CR/LF."""
        old = self.ser.timeout
        try:
            if timeout is not None:
                self.ser.timeout = timeout
            raw = self.ser.readline()  # reads until \n or timeout
        finally:
            self.ser.timeout = old
        if not raw:
            return ""
        try:
            s = raw.decode("ascii", errors="ignore").strip()
        except Exception:
            s = raw.decode("latin1", errors="ignore").strip()
        return s

    def _read_until(self, want_prefixes, overall_timeout=10.0):
        """
        Read lines until we see a line that starts with any prefix in want_prefixes.
        Ignores HB/READY/ECHO noise.
        Returns that line.
        """
        deadline = time.time() + overall_timeout
        while time.time() < deadline:
            line = self._read_line(timeout=0.5)
            if not line:
                continue
            if line in ("HB", "READY"):
                continue
            if line.startswith("ECHO,"):
                continue
            # For FILE,GET flow we may also see FILE,PUT... messages; ignore them here.
            for p in want_prefixes:
                if line.startswith(p):
                    return line
        raise TimeoutError("Timed out waiting for device response.")

    def list_files(self, timeout=5.0):
        """Send FILE,LS and parse: FILE,LS,<count>,name1;name2;...  -> returns [names]."""
        self.send_line("FILE,LS")
        line = self._read_until(["FILE,LS,"], overall_timeout=timeout)
        # Expected: FILE,LS,<count>,<semicolon-separated-names-or-empty>
        # Split only the first 3 fields; the remainder string are the names (possibly empty)
        parts = line.split(",", 3)
        if len(parts) < 3:
            return []
        count_str = parts[2]
        names_str = parts[3] if len(parts) >= 4 else ""
        names = [n for n in names_str.split(";") if n]
        # (count_str may disagree with actual names list if dir changed; we trust the names list)
        return names

    def get_file(self, name, overall_timeout=20.0):
        """
        Fetch a file using FILE,GET,<name>. Returns bytes.
        Accepts multiple FILE,GET,DATA,<hex> lines until FILE,GET,END,OK.
        """
        self.send_line(f"FILE,GET,{name}")
        # Wait for BEGIN
        line = self._read_until(["FILE,GET,BEGIN,"], overall_timeout=overall_timeout)
        # Parse size if present
        # Format: FILE,GET,BEGIN,<name>,<size>
        try:
            parts = line.split(",", 4)
            size_decl = int(parts[-1])
        except Exception:
            size_decl = None

        hex_chunks = []
        deadline = time.time() + overall_timeout
        while time.time() < deadline:
            line = self._read_line(timeout=0.5)
            if not line:
                continue
            if line in ("HB", "READY") or line.startswith("ECHO,"):
                continue
            if line.startswith("FILE,GET,DATA,"):
                h = line[len("FILE,GET,DATA,"):].strip().replace(" ", "")
                # defensive: even length hex only
                if len(h) % 2 != 0:
                    # device emits only full bytes; if odd, stop and error
                    raise ValueError("Odd number of hex characters in DATA line.")
                hex_chunks.append(h)
                continue
            if line.startswith("FILE,GET,END,OK"):
                break
            if line.startswith("FILE,GET,ERR"):
                raise RuntimeError(line)

        blob = bytes.fromhex("".join(hex_chunks))
        if (size_decl is not None) and (len(blob) != size_decl):
            # Not fatal, but surface it
            print(f"[WARN] Declared size {size_decl}, received {len(blob)} bytes.")
        return blob

def _fmt_cm(cm: int) -> str:
    if cm < 0:
        return "no echo"
    if cm > 400:
        return "out of range"
    return f"{cm} cm"

def _cluster_objects(angle_cm_rows, max_gap_cm=8, max_angle_step=5):
    rows = [(a, c) for (a, c) in angle_cm_rows if isinstance(c, (int, float)) and 0 <= c <= 400]
    if not rows:
        return []

    rows.sort(key=lambda t: t[0])
    clusters, cur = [], [rows[0]]

    for a, c in rows[1:]:
        la, lc = cur[-1]
        if abs(c - lc) <= max_gap_cm and (a - la) <= max_angle_step:
            cur.append((a, c))
        else:
            if len(cur) >= 2:
                clusters.append(cur)
            cur = [(a, c)]
    if len(cur) >= 2:
        clusters.append(cur)

    return clusters   # <-- return full clusters of raw points

def _cluster_spread(cl):
    angles = [a for (a, _) in cl]
    dists  = [c for (_, c) in cl]
    ang_span = (max(angles) - min(angles)) if len(angles) > 1 else 0
    dist_span = (max(dists) - min(dists)) if len(dists) > 1 else 0
    # Sort "tightest" first: smaller angle span, smaller distance span, then prefer larger clusters
    return (ang_span, dist_span, -len(cl))

def _cap_clusters(clusters, limit):
    return sorted(clusters, key=_cluster_spread)[:limit]

def _show_radar_popup(parent_window, table_rows):
    """
    Show a 0..180° polar plot with 4m max range.
    Dots = averaged object locations; faint dots = raw samples.
    """
    if not HAVE_MPL:
        import PySimpleGUI as sg
        sg.popup("Matplotlib not installed, cannot render radar view.\n\n"
                 "Tip: pip install matplotlib", title="Radar view")
        return

    import PySimpleGUI as sg

    # Build the polar plot (0..pi, r in meters up to 4m)
    fig = Figure(figsize=(10, 5), dpi=120)
    ax = fig.add_subplot(111, polar=True)
    ax.set_theta_zero_location("E")  # 0° to the right
    ax.set_theta_direction(1)
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    ax.set_rlim(0.0, 4.0)
    #ax.set_rticks([])  # no radial ticks
    ax.set_yticklabels([])  # no radial labels
    ax.grid(True)
    ax.set_title("Detected objects  •  range = 4 m", va='bottom')

    # Semicircle boundary
    thetas = [math.radians(a) for a in range(0, 181)]
    ax.plot(thetas, [4.0]*len(thetas), linewidth=1)

    # Raw points (faint)
    raw_t = [math.radians(a) for (a, c) in table_rows if 0 <= (c if c is not None else -1) <= 400]
    raw_r = [min(c/100.0, 4.0)         for (a, c) in table_rows if 0 <= (c if c is not None else -1) <= 400]
    if raw_t:
        ax.scatter(raw_t, raw_r, s=8, alpha=0.3)

    # Popup window with the canvas
    layout = [[sg.Canvas(key="-RADAR-CANVAS-")], [sg.Button("Close")]]
    win = sg.Window("Sweep Map (0–180°)", layout, modal=True, finalize=True, keep_on_top=True, resizable=True)
    canvas = win["-RADAR-CANVAS-"].TKCanvas
    fig_canvas = FigureCanvasTkAgg(fig, master=canvas)
    fig_canvas.draw()
    fig_canvas.get_tk_widget().pack(fill="both", expand=1)

    # --- Cluster points overlay ---
    clusters = _cluster_objects(table_rows, max_gap_cm=6, max_angle_step=4)
    clusters = _cap_clusters(clusters, 5)  # ≤5 objects

    for cl in clusters:
        angles = [a for (a, _) in cl]
        dists  = [c for (_, c) in cl]
        mid_angle = sum(angles) / len(angles)
        mid_dist  = sum(dists) / len(dists)

        # Angular width (deg → rad)
        ang_span_deg = max(angles) - min(angles)
        ang_span_rad = math.radians(ang_span_deg)

        # Approx. object length at that distance
        obj_length = 2 * mid_dist * math.sin(ang_span_rad / 2.0)

        # Plot a bold orange dot at the cluster center
        ax.scatter([math.radians(mid_angle)], [mid_dist/100.0],
                   s=40, color="blue", edgecolors="black", zorder=3)

        # Annotate with angle, distance, length
        ax.annotate(f"({int(mid_angle)}°,{int(mid_dist)},{int(obj_length)})",
                    xy=(math.radians(mid_angle), mid_dist / 100.0),
                    xytext=(math.radians(mid_angle), 3.7),  # near outer arc
                    ha="center", va="center", fontsize=6, color="blue",
                    arrowprops=dict(arrowstyle="-", color="lightblue"))

    while True:
        ev, _ = win.read(timeout=50)
        if ev in (sg.WIN_CLOSED, "Close"):
            break
    win.close()

def _show_ldr_popup(parent_window, ldr_rows, fit1=None, fit2=None):
    """
    LDR sweep:
      • Readings = small, faint yellow dots (no borders), same size.
      • Cluster centers = gold dots (with thin black edge + annotation).
      • If calibration fit exists → radius = estimated distance (cm→m).
        Otherwise fallback to a fixed 0.5 m ring.
      • VCC readings (raw==1023 or mv>=LDR_VCC_MV) are skipped.
      • Accepts rows: (deg, raw1, mv1, raw2, mv2, b1, b2, delta)
    """
    if not HAVE_MPL:
        sg.popup("Matplotlib not installed, cannot render LDR map.\n\n"
                 "Tip: pip install matplotlib", title="LDR view")
        return

    fig = Figure(figsize=(10, 5), dpi=120)
    ax = fig.add_subplot(111, polar=True)
    ax.set_theta_zero_location("E")
    ax.set_theta_direction(1)
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    ax.set_yticklabels([])
    ax.grid(True)

    have_fit = bool(fit1 or fit2)
    ax.set_title(f"Light sources ({'via calibration' if have_fit else 'no calibration (fixed ring)'}) • clustering enabled",
                 va='bottom')

    max_r_m = 0.6
    ax.set_rlim(0.0, max_r_m)
    thetas = [math.radians(a) for a in range(0, 181)]
    ax.plot(thetas, [max_r_m]*len(thetas), linewidth=1)

    fallback_r_m = 0.5
    if not have_fit:
        ax.plot(thetas, [fallback_r_m]*len(thetas), linewidth=0.7, alpha=0.4)

    # visuals
    READING_SIZE = 8          # smaller, like US
    CLUSTER_SIZE = 60
    READING_COLOR = "yellow"  # faint yellow; no edge
    CLUSTER_COLOR = "gold"

    t_list, r_list = [], []
    skipped_vcc = 0
    skipped_unfit = 0
    cluster_pts = []  # (deg, dist_cm)

    for (deg, raw1, mv1, raw2, mv2, _b1, _b2, _delta) in ldr_rows:
        s1_valid = (raw1 < 1023 and mv1 < LDR_VCC_MV)
        s2_valid = (raw2 < 1023 and mv2 < LDR_VCC_MV)
        if not s1_valid and not s2_valid:
            skipped_vcc += 1
            continue

        use1 = s1_valid and (not s2_valid or mv1 <= mv2)
        raw = raw1 if use1 else raw2
        mv  = mv1  if use1 else mv2
        fit = fit1 if use1 else fit2

        if have_fit and fit is not None:
            d_cm = _estimate_dist_cm(raw, mv, fit)
            if d_cm is None:
                skipped_unfit += 1
                continue
            r_m = min(max(d_cm / 100.0, 0.0), max_r_m)
        else:
            d_cm = 50
            r_m = fallback_r_m

        t_list.append(math.radians(deg))
        r_list.append(r_m)
        cluster_pts.append((deg, d_cm))

    # small, faint yellow readings (no borders)
    if t_list:
        ax.scatter(t_list, r_list, s=READING_SIZE, color=READING_COLOR, alpha=0.3, zorder=2)

    # LDR clusters (gold) with annotation
    if cluster_pts:
        clusters = _cap_clusters(
            _cluster_objects(cluster_pts, max_gap_cm=5, max_angle_step=3),
            2  # ≤2 light sources
        )
        for cl in clusters:
            angles = [a for (a, _) in cl]
            dists  = [c for (_, c) in cl]
            mid_angle = sum(angles) / len(angles)
            mid_dist  = sum(dists) / len(dists)
            # center dot (gold)
            ax.scatter([math.radians(mid_angle)],
                       [mid_dist / 100.0],
                       s=CLUSTER_SIZE, color=CLUSTER_COLOR, edgecolors="black", zorder=3)

            # annotation: (angle°, distance_cm)  ← no length
            ax.annotate(f"({int(mid_angle)}°,{int(mid_dist)})",
                        xy=(math.radians(mid_angle), mid_dist / 100.0),
                        xytext=(math.radians(mid_angle), max_r_m * 0.95),
                        ha="center", va="center", fontsize=6, color=CLUSTER_COLOR,
                        arrowprops=dict(arrowstyle="-", color=CLUSTER_COLOR, alpha=0.8))

    footer = (f"Points plotted: {len(t_list)}   "
              f"skipped VCC: {skipped_vcc}   "
              f"skipped (no fit/out-of-range): {skipped_unfit}")
    layout = [[sg.Canvas(key="-LDR-CANVAS-")],
              [sg.Text(footer)],
              [sg.Button("Close")]]
    win = sg.Window("LDR Sweep Map", layout, modal=True, finalize=True, keep_on_top=True, resizable=True)
    canvas = win["-LDR-CANVAS-"].TKCanvas
    fig_canvas = FigureCanvasTkAgg(fig, master=canvas)
    fig_canvas.draw()
    fig_canvas.get_tk_widget().pack(fill="both", expand=1)
    while True:
        ev, _ = win.read(timeout=50)
        if ev in (sg.WIN_CLOSED, "Close"):
            break
    win.close()

def _show_combo_popup(parent_window, combo_rows, fit1=None, fit2=None):
    """
    Combo view:
      • Ultrasonic (US): faint raw points + cluster overlay
      • LDR: small, borderless light-yellow points at calibrated distance + cluster overlay
        - If both sensors at VCC for an angle, skip it.
        - If no calibration loaded, fall back to a fixed 50 cm ring but still cluster by angle.
    """
    if not HAVE_MPL:
        sg.popup("Matplotlib not installed, cannot render Combo map.\n\n"
                 "Tip: pip install matplotlib", title="Combo view")
        return

    fig = Figure(figsize=(10, 5), dpi=120)
    ax = fig.add_subplot(111, polar=True)
    ax.set_theta_zero_location("E")
    ax.set_theta_direction(1)
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    ax.set_rlim(0.0, 4.0)  # 4 m outer radius for US layer
    ax.set_yticklabels([])
    ax.grid(True)
    ax.set_title("Combo: US clusters + LDR clusters (calibrated when available)", va='bottom')

    thetas = [math.radians(a) for a in range(0, 181)]
    ax.plot(thetas, [4.0]*len(thetas), linewidth=1)

    # ---------------- US layer ----------------
    us_rows = [(deg, dist) for (deg, dist, *_rest) in combo_rows
               if isinstance(dist, int) and 0 <= dist <= 400]
    if us_rows:
        us_t = [math.radians(deg) for (deg, dist) in us_rows]
        us_r = [dist/100.0 for (deg, dist) in us_rows]
        ax.scatter(us_t, us_r, s=8, alpha=0.3)  # faint raw dots

        # US clusters (blue centers)
        us_clusters = _cap_clusters(
            _cluster_objects(us_rows, max_gap_cm=6, max_angle_step=4),
            5  # ≤5 objects
        )

        for cl in us_clusters:
            angles = [a for (a, _) in cl]
            dists  = [c for (_, c) in cl]
            mid_angle = sum(angles) / len(angles)
            mid_dist  = sum(dists) / len(dists)

            ang_span_deg = max(angles) - min(angles)
            ang_span_rad = math.radians(ang_span_deg)
            obj_length = 2 * mid_dist * math.sin(ang_span_rad / 2.0)

            ax.scatter([math.radians(mid_angle)],
                       [mid_dist/100.0],
                       s=40, color="blue", edgecolors="black", zorder=3)
            ax.annotate(f"({int(mid_angle)}°,{int(mid_dist)},{int(obj_length)})",
                        xy=(math.radians(mid_angle), mid_dist/100.0),
                        xytext=(math.radians(mid_angle), 3.7),
                        ha="center", va="center", fontsize=6, color="blue",
                        arrowprops=dict(arrowstyle="-", color="lightblue"))

    # ---------------- LDR layer ----------------
    use_cal = (fit1 is not None) or (fit2 is not None)
    READING_SIZE = 8             # small, like US raw dots
    READING_COLOR = "lightyellow" # borderless
    CLUSTER_SIZE = 60
    CLUSTER_COLOR = "gold"

    t_list, r_list = [], []
    ldr_cluster_pts = []  # (angle_deg, dist_cm)
    skipped_oor = 0

    for (deg, _dist, raw1, mv1, raw2, mv2, _delta) in combo_rows:
        # Drop if both sensors at VCC
        vcc1 = (raw1 >= 1023) or (mv1 >= LDR_VCC_MV)
        vcc2 = (raw2 >= 1023) or (mv2 >= LDR_VCC_MV)
        if vcc1 and vcc2:
            skipped_oor += 1
            continue

        # Pick brighter sensor (lower mV) for distance mapping
        use1 = (mv2 is None) or (mv1 is not None and mv1 <= mv2)
        fit  = fit1 if use1 else fit2
        raw  = raw1 if use1 else raw2
        mv   = mv1  if use1 else mv2

        # Map to distance (cm); fallback = 50 cm
        if use_cal and fit is not None:
            d_cm = _estimate_dist_cm(raw, mv, fit)
            if d_cm is None:
                skipped_oor += 1
                continue
        else:
            d_cm = 50

        # Plot reading (meters)
        t_list.append(math.radians(deg))
        r_list.append(min(max(d_cm/100.0, 0.0), 4.0))
        ldr_cluster_pts.append((deg, d_cm))

    # Draw LDR readings: small, borderless, light yellow
    if t_list:
        ax.scatter(t_list, r_list, s=READING_SIZE, color=READING_COLOR, alpha=0.9)

    # LDR clusters (gold centers)
    if ldr_cluster_pts:
        l_clusters = _cap_clusters(
            _cluster_objects(ldr_cluster_pts, max_gap_cm=5, max_angle_step=3),
            2  # ≤2 light sources
        )

        for cl in l_clusters:
            angles = [a for (a, _) in cl]
            dists  = [c for (_, c) in cl]
            mid_angle = sum(angles) / len(angles)
            mid_dist  = sum(dists) / len(dists)

            ax.scatter([math.radians(mid_angle)], [mid_dist / 100.0],
                       s=CLUSTER_SIZE, color=CLUSTER_COLOR, edgecolors="black", zorder=3)
            # annotation: (angle°, distance_cm)  ← no length
            ax.annotate(f"({int(mid_angle)}°,{int(mid_dist)})",
                        xy=(math.radians(mid_angle), mid_dist / 100.0),
                        xytext=(math.radians(mid_angle), 3.7),
                        ha="center", va="center", fontsize=6, color=CLUSTER_COLOR,
                        arrowprops=dict(arrowstyle="-", color=CLUSTER_COLOR, alpha=0.9))

    # Footer + window
    footer = ("Calibrated LDR" if use_cal else "LDR at fixed 50 cm") \
             + f" • plotted: {len(t_list)} • skipped: {skipped_oor}"
    layout = [[sg.Canvas(key="-COMBO-CANVAS-")],
              [sg.Text(footer if combo_rows else "No data")],
              [sg.Button("Close")]]
    win = sg.Window("Combo Sweep Map", layout, modal=True, finalize=True, keep_on_top=True, resizable=True)
    canvas = win["-COMBO-CANVAS-"].TKCanvas
    fig_canvas = FigureCanvasTkAgg(fig, master=canvas)
    fig_canvas.draw()
    fig_canvas.get_tk_widget().pack(fill="both", expand=1)

    while True:
        ev, _ = win.read(timeout=50)
        if ev in (sg.WIN_CLOSED, "Close"):
            break
    win.close()

def _show_calib_popup(parent_window, rows):
    """
    Animated plot of calibration table:
    x = distance (cm), y = raw ADC (0..1023) for LDR1 & LDR2.
    Points are revealed over time with Play/Pause/Restart.
    """
    if not HAVE_MPL:
        sg.popup("Matplotlib not installed, cannot render calibration view.\n\n"
                 "Tip: pip install matplotlib", title="Calibration view")
        return

    # ensure sorted by distance
    rows = sorted(rows, key=lambda r: r[0])  # (dist_cm, ldr1_raw, ldr2_raw)
    if not rows:
        sg.popup("No calibration rows to display.", title="Calibration view")
        return

    dists = [r[0] for r in rows]
    y1_all = [r[1] for r in rows]
    y2_all = [r[2] for r in rows]

    fig = Figure(figsize=(8, 4), dpi=120)
    ax = fig.add_subplot(111)
    ax.set_title("LDR Calibration (raw ADC vs distance)")
    ax.set_xlabel("Distance (cm)")
    ax.set_ylabel("Raw ADC (0..1023)")
    ax.set_xlim(min(dists) - 2, max(dists) + 2)
    ax.set_ylim(0, 1023)

    line1, = ax.plot([], [], marker='o', linestyle='-', label='LDR1')
    line2, = ax.plot([], [], marker='o', linestyle='-', label='LDR2')
    ax.legend(loc="upper right")

    layout = [
        [sg.Canvas(key="-CAL-CANVAS-")],
        [sg.Button("Play/Pause", key="-CAL-PP-"),
         sg.Button("Restart", key="-CAL-RE-"),
         sg.Button("Close")]
    ]
    win = sg.Window("LDR Calibration", layout, modal=True, finalize=True, keep_on_top=True, resizable=True)
    canvas = win["-CAL-CANVAS-"].TKCanvas
    fig_canvas = FigureCanvasTkAgg(fig, master=canvas)
    fig_canvas.draw()
    fig_canvas.get_tk_widget().pack(fill="both", expand=1)

    idx = 0
    playing = True
    last_tick = time.time()
    frame_dt = 0.10  # seconds per point (~10 FPS)

    def _draw(upto):
        # reveal data up to 'upto' (inclusive)
        xs = dists[:upto]
        y1 = y1_all[:upto]
        y2 = y2_all[:upto]
        line1.set_data(xs, y1)
        line2.set_data(xs, y2)
        fig_canvas.draw()

    # initial draw (nothing)
    _draw(0)

    while True:
        ev, _ = win.read(timeout=40)
        if ev in (sg.WIN_CLOSED, "Close"):
            break
        if ev == "-CAL-PP-":
            playing = not playing
        if ev == "-CAL-RE-":
            idx = 0
            playing = True
            _draw(0)

        if playing and (time.time() - last_tick) >= frame_dt:
            if idx < len(rows):
                idx += 1
                _draw(idx)
                last_tick = time.time()
            else:
                # reached end; pause
                playing = False

    win.close()

def _linfit(xs, ys):
    """Least-squares line fit: returns (a, b) for y = a*x + b, or None."""
    n = len(xs)
    if n < 2:
        return None
    sx = sum(xs); sy = sum(ys)
    sxx = sum(x*x for x in xs); sxy = sum(x*y for x, y in zip(xs, ys))
    denom = n * sxx - sx * sx
    if denom == 0:
        return None
    a = (n * sxy - sx * sy) / denom
    b = (sy - a * sx) / n
    return (a, b)

def _build_calibration_fits(calib_rows, calib_dump_rows):
    """
    Returns (fit1, fit2) where each is (a, b) mapping raw -> distance_cm.
    Prefers dump rows if present; falls back to live calib rows.
    """
    rows = []
    if calib_dump_rows:
        # calib_dump_rows: (dist_cm, ldr1_raw, ldr2_raw)
        rows = [(d, r1, r2) for (d, r1, r2) in calib_dump_rows]
    elif calib_rows:
        # calib_rows: (idx, dist_cm, ldr1_raw, ldr2_raw)
        rows = [(d, r1, r2) for (_i, d, r1, r2) in calib_rows]

    if not rows:
        return (None, None)

    # Sensor 1 fit
    x1 = [r1 for (d, r1, _r2) in rows if r1 < 1023]
    y1 = [d  for (d, r1, _r2) in rows if r1 < 1023]
    fit1 = _linfit(x1, y1) if len(x1) >= 2 else None

    # Sensor 2 fit
    x2 = [r2 for (d, _r1, r2) in rows if r2 < 1023]
    y2 = [d  for (d, _r1, r2) in rows if r2 < 1023]
    fit2 = _linfit(x2, y2) if len(x2) >= 2 else None

    return (fit1, fit2)

def _estimate_dist_cm(raw, mv, fit):
    """
    Use (a,b) to map raw->dist. Treat VCC readings (raw=1023 or mv>=VCC) as out-of-range.
    Returns None if not estimable/out-of-range.
    """
    if fit is None:
        return None
    if raw >= 1023 or mv >= LDR_VCC_MV:
        return None  # out of range by rule
    a, b = fit
    d = a * raw + b
    # Keep within a sensible LDR range (0..60 cm), otherwise treat as OOR
    if d <= 0 or d > 60:
        return None
    return d

# ----------------------- GUI -----------------------
def main():
    sg.theme("BlueMono")

    autodetected = find_msp430_port()
    ports = list_all_ports()
    if autodetected and autodetected not in ports:
        ports.insert(0, autodetected)

    # --- Top bar ---
    top_row = [
        sg.Text("Port:"), sg.Combo(ports, default_value=autodetected, key="-PORT-", size=(20,1)),
        sg.Button("Refresh", key="-REFRESH-"),
        sg.Push(),
        sg.Button("Connect", key="-CONNECT-", button_color=("white","green")),
        sg.Button("Disconnect", key="-DISCONNECT-", button_color=("white","firebrick4"), disabled=True),
    ]

    # --- Tabs ---
    # Smoke Test tab: PING / ECHO + live log
    tab_test = sg.Tab("Smoke Test", [[
        sg.Column([
            [sg.Button("PING", key="-PING-", size=(10,1), disabled=True),
             sg.Text("Echo:"), sg.Input("", key="-ECHO-IN-", size=(25,1)),
             sg.Button("Send Echo", key="-ECHO-", disabled=True)],
            [sg.Multiline("", key="-LOG-", size=(90,16), autoscroll=True, disabled=True, background_color="#f6f8fa")],
            [sg.Text("Tip: If firmware = state-based, use the Control tab. If firmware = smoke test, try PING/Echo.")]
        ])
    ]])

    # Control tab: STATE-based features
    tab_ctrl = sg.Tab("Control", [[
        sg.Column([
            [sg.Button("US 180° Sweep", key="-BTN-US-SWEEP-", size=(16,2), disabled=True),
             sg.Button('LDR 180° Sweep', key='-BTN-LDR-SWEEP-', size=(16, 2), disabled=True),
             sg.Button("Combo 180° Sweep", key="-BTN-COMBO-SWEEP-", size=(16, 2), disabled=True),
             sg.Text("Angle:"), sg.Spin(values=list(range(0,181)), initial_value=90, key="-ANGLE-", size=(4,1)),
             sg.Button("Measure at angle", key="-MEASURE-", size=(16,2), disabled=True),
             sg.Button("Calibrate LDRs", key="-BTN-LDR-CAL-", size=(16, 2), disabled=True),
             sg.Button("Read LDR", key="-LDR-", size=(10,2), disabled=True),
             sg.Button("Show LDR Calibration", key="-BTN-LDR-CAL-DUMP-", size=(18, 2), disabled=True)],
            [sg.Table(
                values=[],
                headings=["Button", "Angle (°)", "Reading"],
                auto_size_columns=True,
                justification="center",
                num_rows=14,
                key="-TABLE-",
                expand_x=True,
                expand_y=True,
            )]
            ,
            [sg.Button("Clear Table", key="-CLEAR-", disabled=True)]
        ])
    ]])

    # Files tab: MCU flash mini-FS over UART
    tab_files = sg.Tab("Files", [[
        sg.Column([
            [sg.Text("Device files:"), sg.Push(),
             sg.Button("List", key="-FILE-LS-", size=(10, 1), disabled=True)],
            [sg.Listbox(values=[], size=(40, 10), key="-FILE-LIST-", enable_events=True, expand_y=True)],
            [sg.Text("Name on device:"), sg.Input("", key="-FILE-NAME-", size=(24,1))],
            [sg.Text("PC file:"), sg.Input("", key="-FILE-PATH-", size=(36,1), enable_events=True),
             sg.FileBrowse("Browse…", key="-FILE-BROWSE-")],
            [sg.Button("Upload → MCU", key="-FILE-UPLOAD-", size=(16,1), disabled=True),
             sg.Button("View", key="-FILE-VIEW-", size=(10, 1), disabled=True),
             sg.Button("Show on LCD", key="-FILE-SHOW-LCD-", size=(14, 1), disabled=True),
             sg.Button("Exit LCD", key="-FILE-EXIT-LCD-", disabled=True),
             sg.Button("Delete", key="-FILE-DEL-", size=(10,1), disabled=True)],
            [sg.ProgressBar(100, orientation='h', size=(36, 20), key='-FILE-PROG-', visible=False),
             sg.Text('', key='-FILE-PROG-TXT-', visible=False)]
        ])
    ]])

    layout = [
        [sg.Text("MSP430 UART GUI", font=("Verdana", 16), expand_x=True, justification="center")],
        top_row,
        [sg.TabGroup([[tab_test, tab_ctrl, tab_files]])],
        [sg.Text("Status:"), sg.Text("Disconnected", key="-STATUS-", size=(80,1))]
    ]

    window = sg.Window("Control System – UART GUI", layout, finalize=True, resizable=True)

    rx_q = queue.Queue()
    mcu = MCU(rx_q)
    table_rows = []  # (button, angle, reading)
    us_points = []  # (angle, cm) from US 180° sweep only
    sweep_active = False
    need_radar_popup = False   # trigger radar animation after sweep ends
    need_ldr_popup = False
    need_combo_popup = False
    need_calib_popup = False

    # For simple pattern waits without blocking serial:
    pending = {}  # key -> (deadline, match_fn, on_success)

    # --- LDR sweep capture state ---
    ldr_collect_active = False
    ldr_rows = []  # parsed tuples: (deg, brightness1, brightness2, delta)

    combo_collect_active = False
    combo_rows = []  # (deg, dist_cm, raw1, mv1, raw2, mv2, delta)

    # --- LDR calibration capture state ---
    calib_active = False
    calib_rows = []  # parsed tuples: (idx, dist_cm, ldr1_raw, ldr2_raw)

    # --- LDR calibration dump state (reads back from FLASH) ---
    calib_dump_active = False
    calib_dump_rows = []  # list of tuples: (dist_cm, ldr1_raw, ldr2_raw)

    calib_fit1 = None  # (a,b) for sensor 1 raw -> distance_cm
    calib_fit2 = None  # (a,b) for sensor 2 raw -> distance_cm

    # --- File mode state ---
    files_list = []  # latest list from MCU
    # Download session
    file_dl_active = False
    file_dl_name = ""
    file_dl_expected = 0
    file_dl_buf = bytearray()
    file_dl_save_path = None
    # Upload session plan (drives pending callbacks)
    upload_plan = None  # dict: {"name":str, "type":0, "chunks":[bytes], "idx":int}

    view_after_download = False  # when True, open viewer on successful GET

    spinner_i = 0
    uploading = False
    upload_name = ""

    lcd_exit_until = 0.0
    lcd_exit_next = 0.0

    lcd_show_active = False

    def _set_lcd_show_ui(active: bool):
        """Disable everything except 'Exit LCD' while FILE-SHOW is active."""

        def _safe_update(key, **kw):
            elem = window.find_element(key)
            if elem is not None:
                try:
                    elem.update(**kw)
                except Exception:
                    pass

        lcd_keys_disable = (
            # Smoke Test / Control
            "-PING-", "-ECHO-",
            "-BTN-US-SWEEP-", "-BTN-LDR-SWEEP-", "-BTN-COMBO-SWEEP-",
            "-MEASURE-", "-LDR-", "-BTN-LDR-CAL-", "-BTN-LDR-CAL-DUMP-",
            "-CLEAR-",
            # Files tab (only keys that actually exist in your layout)
            "-FILE-LS-", "-FILE-UPLOAD-", "-FILE-VIEW-", "-FILE-SHOW-LCD-", "-FILE-DEL-",
            "-FILE-NAME-", "-FILE-PATH-", "-FILE-BROWSE-", "-FILE-LIST-",
            # Top bar
            "-CONNECT-", "-DISCONNECT-",
        )
        for k in lcd_keys_disable:
            _safe_update(k, disabled=active)

        _safe_update("-FILE-EXIT-LCD-", disabled=not active)

    def _open_viewer_window(name, data: bytes):
        # Try to decode as text; else show hex preview
        text = None
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            try:
                text = data.decode("latin-1")
            except UnicodeDecodeError:
                text = None

        if text is None:
            # Show first 256 bytes as hex if not text
            preview = " ".join(f"{b:02X}" for b in data[:256])
            body = f"(Binary file — first 256 bytes as hex)\n\n{preview}"
        else:
            body = text

        layout = [
            [sg.Text(f"{name}  —  {len(data)} bytes"), sg.Push(),
             sg.Button("Save As…", key="-VIEW-SAVE-"), sg.Button("Close")],
            [sg.Multiline(body, size=(100, 35), disabled=True, key="-VIEW-TXT-")]
        ]
        vw = sg.Window("View File", layout, modal=True, finalize=True, keep_on_top=True, resizable=True)
        while True:
            ev, _ = vw.read()
            if ev in (sg.WIN_CLOSED, "Close"):
                break
            if ev == "-VIEW-SAVE-":
                path = sg.popup_get_file("Save file as…", save_as=True, default_extension="", no_window=True)
                if path:
                    try:
                        with open(path, "wb") as f:
                            f.write(data)
                        sg.popup_ok(f"Saved {len(data)} bytes to:\n{path}", keep_on_top=True)
                    except Exception as e:
                        sg.popup_error(f"Save failed:\n{e}", keep_on_top=True)
        vw.close()

    def _upload_ui(active: bool, *, name: str = "", total_chunks: int = 0, done_chunks: int = 0):
        if active:
            pct = int(100 * (done_chunks / max(1, total_chunks)))
            window['-FILE-PROG-'].update(pct, visible=True)
            try:
                spin = SPINNER[spinner_i]
            except NameError:
                spin = SPINNER[0]
            window['-FILE-PROG-TXT-'].update(
                f"{spin} Uploading {name}… {done_chunks}/{total_chunks} chunk(s) ({pct}%)",
                visible=True
            )
        else:
            window['-FILE-PROG-'].update(0, visible=False)
            window['-FILE-PROG-TXT-'].update('', visible=False)

    def log_line(s, kind="rx"):
        window["-LOG-"].update(disabled=False)
        if kind == "tx":
            window["-LOG-"].print(f">> {s}")
        else:
            window["-LOG-"].print(s)
        window["-LOG-"].update(disabled=True)

    # --------------- Event loop ---------------
    while True:
        event, values = window.read(timeout=100)
        if event == sg.WIN_CLOSED:
            break

        # --- log messages from FileClient worker ---
        if event == "-GUI-LOG-":
            msg = values.get("-GUI-LOG-", "")
            if msg:
                log_line(msg, "rx")

        # --- coarse upload progress from worker ---
        if event == "-GUI-UP-PROG-":
            mode, *rest = values.get("-GUI-UP-PROG-", ("",))
            if mode == "start":
                up_name, total_bytes = rest
                uploading = True
                upload_name = up_name
                _upload_ui(True,
                           name=up_name,
                           total_chunks=(total_bytes + CHUNK_BYTES - 1) // CHUNK_BYTES,
                           done_chunks=0)
                window["-STATUS-"].update(f"Uploading {up_name}… {total_bytes} bytes")

            elif mode == "tick":
                done, total = rest if len(rest) == 2 else (0, 0)
                if uploading and upload_name:
                    _upload_ui(True, name=upload_name, total_chunks=total, done_chunks=done)

            elif mode == "done":
                up_name = rest[0] if rest else ""
                uploading = False
                upload_name = ""
                _upload_ui(False)
                window["-STATUS-"].update(f"Upload complete: {up_name}")
                window["-FILE-NAME-"].update("")
                window.write_event_value("-FILE-LS-", None)

            elif mode == "err":
                errmsg = rest[0] if rest else "Unknown error"
                uploading = False
                upload_name = ""
                _upload_ui(False)
                window["-STATUS-"].update(f"Upload failed: {errmsg}")

        # Drain serial queue
        try:
            while True:
                typ, payload = rx_q.get_nowait()
                if typ == "rx":
                    line = payload
                    log_line(line, "rx")
                    # LCD viewer finished (PB1 exit or PC abort) → bounce an internal event
                    if line.startswith("FILE,SHOW,OK"):
                        window.write_event_value("-LCD-SHOW-DONE-", True)
                        continue

                    # If sweep is active, parse lines angle,cm and FIN
                    if sweep_active:
                        if line == "FIN":
                            window["-STATUS-"].update("Sweep finished.")
                            sweep_active = False
                            need_radar_popup = True
                        else:
                            parts = [p.strip() for p in line.split(",")]
                            if len(parts) == 2 and parts[0].lstrip("-").isdigit() and parts[1].lstrip("-").isdigit():
                                angle = int(parts[0])
                                cm = int(parts[1])

                                # feed the radar with raw points
                                us_points.append((angle, cm))

                                # feed the GUI table with a friendly string
                                table_rows.append(("US-SWEEP", angle, _fmt_cm(cm)))
                                window["-TABLE-"].update(values=table_rows)

                                window["-CLEAR-"].update(disabled=False)

                    # If an LDR sweep is active, collect 'LDRS,...' lines until FIN
                    if ldr_collect_active:
                        if line == "FIN":
                            ldr_collect_active = False
                            window["-STATUS-"].update(f"LDR sweep finished. {len(ldr_rows)} samples.")
                            need_ldr_popup = True


                        elif line.startswith("LDRS,"):
                            parts = line.split(',')
                            if len(parts) == 6:
                                try:
                                    deg = int(parts[1])
                                    raw1 = int(parts[2])
                                    mv1 = int(parts[3])
                                    raw2 = int(parts[4])
                                    mv2 = int(parts[5])
                                    b1 = max(0, LDR_VCC_MV - mv1)
                                    b2 = max(0, LDR_VCC_MV - mv2)
                                    delta = b1 - b2
                                    ldr_rows.append((deg, raw1, mv1, raw2, mv2, b1, b2, delta))
                                    table_rows.append(("LDR-SWEEP", deg, f"{raw1}/{mv1} mV , {raw2}/{mv2} mV"))
                                    window["-TABLE-"].update(values=table_rows)

                                except ValueError:
                                    pass

                            window["-STATUS-"].update(f"LDR sweep running… {len(ldr_rows)} rows")

                    if combo_collect_active:
                        if line == "FIN":
                            combo_collect_active = False
                            window["-STATUS-"].update(f"Combo sweep finished. {len(combo_rows)} samples.")
                            need_combo_popup = True

                        elif line.startswith("COMBO,"):
                            parts = line.split(',')

                            if len(parts) == 7:
                                try:
                                    deg = int(parts[1])
                                    dist = int(parts[2])
                                    raw1 = int(parts[3])
                                    mv1 = int(parts[4])
                                    raw2 = int(parts[5])
                                    mv2 = int(parts[6])
                                    b1 = max(0, LDR_VCC_MV - mv1)
                                    b2 = max(0, LDR_VCC_MV - mv2)
                                    delta = b1 - b2
                                    combo_rows.append((deg, dist, raw1, mv1, raw2, mv2, delta))

                                    # --- also log into the GUI table ---
                                    table_rows.append(("COMBO", deg, f"{_fmt_cm(dist)} , {raw1}/{mv1} , {raw2}/{mv2}"))
                                    window["-TABLE-"].update(values=table_rows)

                                except ValueError:
                                    pass

                    # -------- LDR Calibration flow (PB0-driven on the MCU) --------
                    if line.startswith("CAL,"):
                        parts = line.split(',')
                        tag = parts[1] if len(parts) > 1 else ""

                        if tag == "BEGIN":
                            calib_active = True
                            calib_rows.clear()
                            window["-STATUS-"].update(
                                "Calibration started: follow prompts; position at 5..50 cm and press PB0 each step.")
                            sg.popup_quick_message(
                                "Calibration started.\nPlace at 5 cm when prompted, then press PB0 on the board.\nRepeats every 5 cm up to 50 cm.",
                                auto_close=True, auto_close_duration=2, location=window.current_location()
                            )

                        elif tag == "INSTR" and len(parts) >= 3:
                            # Instruction message from firmware
                            window["-STATUS-"].update(f"CAL: {','.join(parts[2:])}")

                        elif tag == "PLACE" and len(parts) >= 3:
                            try:
                                dist = int(parts[2])
                                window["-STATUS-"].update(f"Place light at {dist} cm and press PB0 to sample…")
                                sg.popup_quick_message(
                                    f"Place at {dist} cm, then press PB0.",
                                    auto_close=True, auto_close_duration=1.5, location=window.current_location()
                                )
                            except ValueError:
                                pass

                        elif tag == "SAVED" and len(parts) >= 6:
                            # CAL,SAVED,idx,dist,ldr1,ldr2
                            try:
                                idx = int(parts[2])
                                dist = int(parts[3])
                                l1 = int(parts[4])
                                l2 = int(parts[5])
                                calib_rows.append((idx, dist, l1, l2))
                                window["-STATUS-"].update(f"Saved sample {idx} at {dist} cm → LDR1={l1}, LDR2={l2}")
                            except ValueError:
                                pass

                        elif tag == "END":
                            ok = (len(parts) >= 3 and parts[2] == "OK")
                            calib_active = False
                            calib_fit1, calib_fit2 = _build_calibration_fits(calib_rows, [])
                            if calib_fit1 or calib_fit2:
                                window["-STATUS-"].update("Calibration finished. Distance mapping ready.")
                            else:
                                window["-STATUS-"].update("Calibration finished, but not enough data to fit.")

                            if ok:
                                window["-STATUS-"].update("Calibration finished (OK).")
                            else:
                                window["-STATUS-"].update("Calibration finished (with ERR).")

                            sg.popup_quick_message(
                                "Calibration done.",
                                auto_close=True, auto_close_duration=2, location=window.current_location()
                            )


                        elif tag == "TBL" and len(parts) >= 5:
                            # From api_ldr_calib_dump_uart(): CAL,TBL,<dist_cm>,<ldr1>,<ldr2>
                            try:
                                dist = int(parts[2])
                                l1 = int(parts[3])
                                l2 = int(parts[4])
                                # If a dump session wasn't explicitly started by button, start now.
                                if not calib_dump_active:
                                    calib_dump_active = True
                                    calib_dump_rows.clear()
                                calib_dump_rows.append((dist, l1, l2))
                                window["-STATUS-"].update(f"CAL TBL row: {dist} cm → LDR1={l1}, LDR2={l2}")
                            except ValueError:
                                pass

                        elif tag == "DUMP":
                            ok = (len(parts) >= 3 and parts[2] == "OK")
                            if calib_dump_active:
                                calib_fit1, calib_fit2 = _build_calibration_fits([], calib_dump_rows)
                                if calib_fit1 or calib_fit2:
                                    window["-STATUS-"].update(
                                        f"Calibration dump OK. {len(calib_dump_rows)} rows. Distance mapping ready.")
                                else:
                                    window["-STATUS-"].update("Calibration dump OK, but not enough data to fit.")
                                need_calib_popup = True
                            calib_dump_active = False
                    # -------- FILE MODE: stream parsers --------
                    if line.startswith("FILE,GET,BEGIN,"):
                        # FORMAT: FILE,GET,BEGIN,<name>,<size>
                        parts = line.split(",", 4)  # ["FILE","GET","BEGIN","<name>","<size>"]
                        if len(parts) >= 5:
                            file_dl_active = True
                            file_dl_name = parts[3]

                            try:
                                file_dl_expected = int(parts[4])
                            except ValueError:
                                file_dl_expected = 0
                            file_dl_buf.clear()
                            window["-STATUS-"].update(f"Downloading {file_dl_name} ({file_dl_expected} bytes)…")

                    elif line.startswith("FILE,GET,DATA,") and file_dl_active:
                        # Append decoded payload to buffer
                        try:
                            hex_payload = line.split(",", 3)[3]
                            file_dl_buf.extend(_bytes_from_hex(hex_payload))
                            window["-STATUS-"].update(f"Downloading {file_dl_name}… {len(file_dl_buf)}/{file_dl_expected} bytes")
                        except Exception:
                            window["-STATUS-"].update("Bad DATA hex from device.")

                    elif line.startswith("FILE,GET,END") and file_dl_active:
                        # Save to disk if a path was chosen
                        ok = ("OK" in line)
                        if ok and file_dl_save_path:
                            try:
                                with open(file_dl_save_path, "wb") as f:
                                    f.write(file_dl_buf)
                                window["-STATUS-"].update(f"Saved {file_dl_name} → {file_dl_save_path}")
                            except Exception as e:
                                window["-STATUS-"].update(f"Save failed: {e}")
                        elif ok and view_after_download:
                            # show a viewer window
                            _open_viewer_window(file_dl_name, bytes(file_dl_buf))
                            spinner_i = 0
                            window["-STATUS-"].update("GET completed (viewed).")
                        elif ok and not file_dl_save_path:
                            window["-STATUS-"].update("GET completed (no save path chosen).")
                        else:
                            window["-STATUS-"].update("GET ended with ERR.")

                            # reset common state
                        file_dl_active = False
                        file_dl_name = ""
                        file_dl_expected = 0
                        file_dl_buf.clear()
                        file_dl_save_path = None
                        view_after_download = False

                    # -------- FILE,PUT FSM (no pending[] for uploads) --------
                    if upload_plan:
                        # BEGIN,OK -> send 1st chunk
                        if line.startswith("FILE,PUT,BEGIN,OK") and upload_plan.get("state") == "begin_sent":
                            try:
                                payload = _hex_from_bytes(upload_plan["chunks"][upload_plan["idx"]])
                                window["-STATUS-"].update(
                                    f"PUT sending chunk {upload_plan['idx'] + 1}/{upload_plan['total']}…")
                                time.sleep(INTER_CHUNK_DELAY_S)
                                mcu.send_line(f"FILE,PUT,CHUNK,{payload}")
                                upload_plan["state"] = "chunk_sent"
                            except Exception as e:
                                window["-STATUS-"].update(f"PUT CHUNK send error: {e}")
                                _upload_ui(False)
                                upload_plan = None

                        # CHUNK,OK -> advance; send next or END
                        elif line.startswith("FILE,PUT,CHUNK,OK") and upload_plan.get("state") == "chunk_sent":
                            upload_plan["idx"] += 1
                            done = upload_plan["idx"]
                            total = upload_plan["total"]
                            _upload_ui(True, name=upload_plan["name"], total_chunks=total, done_chunks=done)
                            if done < total:
                                # send next chunk
                                try:
                                    payload = _hex_from_bytes(upload_plan["chunks"][upload_plan["idx"]])
                                    window["-STATUS-"].update(f"PUT sending chunk {upload_plan['idx'] + 1}/{total}…")
                                    time.sleep(INTER_CHUNK_DELAY_S)
                                    mcu.send_line(f"FILE,PUT,CHUNK,{payload}")
                                    upload_plan["state"] = "chunk_sent"
                                except Exception as e:
                                    window["-STATUS-"].update(f"PUT CHUNK send error: {e}")
                                    _upload_ui(False)
                                    upload_plan = None
                            else:
                                # all chunks acked -> END
                                try:
                                    window["-STATUS-"].update("PUT sending END…")
                                    time.sleep(INTER_CHUNK_DELAY_S)
                                    mcu.send_line("FILE,PUT,END")
                                    upload_plan["state"] = "end_sent"
                                except Exception as e:
                                    window["-STATUS-"].update(f"PUT END send error: {e}")
                                    _upload_ui(False)
                                    upload_plan = None

                        # END result
                        elif line.startswith("FILE,PUT,END"):
                            ok = line.startswith("FILE,PUT,END,OK")
                            total = upload_plan["total"]
                            if ok:
                                _upload_ui(True, name=upload_plan["name"], total_chunks=total, done_chunks=total)
                                window["-STATUS-"].update(
                                    f"Uploaded {upload_plan['name']} ({sum(len(c) for c in upload_plan['chunks'])} B)."
                                )

                                # ⬇️ Clear the device-name (and optionally the PC-path) so the next selection auto-fills
                                window["-FILE-NAME-"].update("")  # <-- place this line here
                                # window["-FILE-PATH-"].update("")    # (optional) also clear chosen file path

                                # refresh list after successful upload
                                window.write_event_value("-FILE-LS-", None)
                            else:
                                window["-STATUS-"].update(f"PUT END failed: {line}")
                            _upload_ui(False)
                            upload_plan = None

                        # Any explicit device-side error
                        elif line.startswith("FILE,PUT,ERR"):
                            window["-STATUS-"].update(line)
                            _upload_ui(False)
                            upload_plan = None
                    # -------- end FILE,PUT FSM --------

                    # Check pending matchers
                    now = time.time()
                    for key in list(pending.keys()):
                        deadline, match_fn, on_ok = pending[key]
                        if match_fn(line):
                            on_ok(line)
                            del pending[key]

                    # If a sweep just finished, show the radar animation
                    if need_radar_popup:
                        need_radar_popup = False
                        _show_radar_popup(window, us_points)

                    if need_ldr_popup:
                        need_ldr_popup = False
                        _show_ldr_popup(window, ldr_rows, calib_fit1, calib_fit2)
                        ldr_rows.clear()

                    if need_combo_popup:
                        need_combo_popup = False
                        _show_combo_popup(window, combo_rows, calib_fit1, calib_fit2)
                        combo_rows.clear()

                    if need_calib_popup:
                        need_calib_popup = False
                        _show_calib_popup(window, calib_dump_rows)
                        calib_dump_rows.clear()

                else:
                    log_line(payload, "tx")
        except queue.Empty:
            pass

        if event == "-REFRESH-":
            window["-PORT-"].update(values=list_all_ports())

        if event == "-CONNECT-":
            port = values.get("-PORT-")
            if not port:
                window["-STATUS-"].update("Select a COM port.")
                continue
            try:
                mcu.open(port)
                window["-DISCONNECT-"].update(disabled=False)
                window["-CONNECT-"].update(disabled=True)
                for k in ("-PING-","-ECHO-","-BTN-US-SWEEP-","-BTN-LDR-SWEEP-","-BTN-COMBO-SWEEP-","-MEASURE-","-LDR-","-BTN-LDR-CAL-","-BTN-LDR-CAL-DUMP-"):
                    window[k].update(disabled=False)
                # Enable Files tab actions
                for k in ("-FILE-LS-", "-FILE-UPLOAD-", "-FILE-VIEW-", "-FILE-SHOW-LCD-", "-FILE-DEL-"):
                    window[k].update(disabled=False)

                window["-STATUS-"].update(f"Connected to {port}. Listening…")

                # Proactively poke both firmwares to elicit output
                try:
                    mcu.send_line("PING")        # smoke test
                    mcu.send_line("STATE,3")     # state-based (LDR one-shot)
                except Exception:
                    pass

                # Proactively fetch calibration from FLASH so distance mapping is ready
                try:
                    calib_dump_active = True
                    calib_dump_rows.clear()
                    mcu.send_line("CAL,DUMP")
                    window["-STATUS-"].update("Connected. Fetching calibration from FLASH for LDR distance mapping…")
                except Exception:
                    pass


            except Exception as e:
                window["-STATUS-"].update(f"Connect error: {e}")

        if event == "-DISCONNECT-":
            sweep_active = False
            mcu.close()
            window["-DISCONNECT-"].update(disabled=True)
            window["-CONNECT-"].update(disabled=False)
            for k in ("-PING-","-ECHO-","-BTN-US-SWEEP-","-BTN-LDR-SWEEP-","-BTN-COMBO-SWEEP-","-MEASURE-","-LDR-","-BTN-LDR-CAL-","-BTN-LDR-CAL-DUMP-"):
                window[k].update(disabled=True)
            # Disable Files tab actions
            for k in ("-FILE-LS-", "-FILE-UPLOAD-", "-FILE-VIEW-", "-FILE-SHOW-LCD-", "-FILE-DEL-",
                      "-PING-", "-ECHO-",
                      "-BTN-US-SWEEP-", "-BTN-LDR-SWEEP-", "-BTN-COMBO-SWEEP-",
                      "-MEASURE-", "-LDR-", "-BTN-LDR-CAL-", "-BTN-LDR-CAL-DUMP-"):
                window[k].update(disabled=True)

            window["-FILE-LIST-"].update(values=[])
            # cancel in-flight waits & file ops
            pending.clear()

            file_dl_active = False
            file_dl_name = ""
            file_dl_expected = 0
            file_dl_buf.clear()
            file_dl_save_path = None

            upload_plan = None

            window["-STATUS-"].update("Disconnected")

        # ---------- Smoke Test ----------
        if event == "-PING-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            try:
                mcu.send_line("PING")
            except Exception as e:
                window["-STATUS-"].update(f"PING send error: {e}")
                continue
            deadline = time.time() + 2.0
            def ok(line):
                if line is None:
                    window["-STATUS-"].update("No PONG (timeout).")
                elif line.strip() == "PONG":
                    window["-STATUS-"].update("PONG received.")
            pending["PING"] = (deadline, lambda s: s == "PONG", ok)
            window["-STATUS-"].update("PING sent…")

        if event == "-ECHO-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            text = (values.get("-ECHO-IN-") or "").strip()
            if not text:
                window["-STATUS-"].update("Type something to echo.")
                continue
            try:
                mcu.send_line(text)
            except Exception as e:
                window["-STATUS-"].update(f"ECHO send error: {e}")
                continue

        # ---------- Control ----------
        if event == "-CLEAR-":
            table_rows.clear()
            window["-TABLE-"].update(values=[])
            window["-CLEAR-"].update(disabled=True)

        if event == "-BTN-US-SWEEP-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            if sweep_active:
                window["-STATUS-"].update("Sweep already running…")
            else:
                table_rows.clear()
                us_points.clear()
                window["-TABLE-"].update(values=[])
                window["-CLEAR-"].update(disabled=True)
                sweep_active = True

                try:
                    mcu.send_line("STATE,1")
                    window["-STATUS-"].update("Starting sweep…")
                except Exception as e:
                    window["-STATUS-"].update(f"Sweep TX error: {e}")
                    sweep_active = False

        if event == '-BTN-LDR-SWEEP-':
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            if ldr_collect_active:
                window["-STATUS-"].update("LDR sweep already running…")
                continue
            if sweep_active:
                window["-STATUS-"].update("Ultrasonic sweep running; wait for it to finish.")
                continue
            # Start fresh capture
            ldr_collect_active = True
            try:
                mcu.send_line("STATE,4")
                window["-STATUS-"].update("Starting LDR sweep…")
            except Exception as e:
                ldr_collect_active = False
                window["-STATUS-"].update(f"LDR sweep TX error: {e}")

        if event == "-BTN-COMBO-SWEEP-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            if combo_collect_active:
                window["-STATUS-"].update("Combo sweep already running…")
                continue
            if sweep_active or ldr_collect_active:
                window["-STATUS-"].update("Another sweep running; wait for it to finish.")
                continue

            combo_collect_active = True
            try:
                mcu.send_line("STATE,5")
                window["-STATUS-"].update("Starting Combo sweep…")
            except Exception as e:
                combo_collect_active = False
                window["-STATUS-"].update(f"Combo sweep TX error: {e}")

        if event == "-MEASURE-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            try:
                deg = int(values["-ANGLE-"])
                if deg < 0 or deg > 180:
                    window["-STATUS-"].update("Angle must be 0..180.")
                    continue
                mcu.send_line(f"STATE,2,{deg}")
                deadline = time.time() + 2.0
                def ok(line):
                    if line is None:
                        window["-STATUS-"].update("No DIST reply (timeout).")
                        return
                    if line.startswith("DIST,"):
                        parts = [p.strip() for p in line.split(",")]
                        if len(parts) == 3:
                            try:
                                rdeg = int(parts[1])
                                rcm = int(parts[2])
                                window["-STATUS-"].update(f"Angle {rdeg}° → {_fmt_cm(rcm)}")
                                table_rows.append(("MEASURE", rdeg, _fmt_cm(rcm)))
                                window["-TABLE-"].update(values=table_rows)
                                window["-CLEAR-"].update(disabled=False)

                            except ValueError:
                                window["-STATUS-"].update(f"Bad reply: {line}")
                pending["MEASURE"] = (deadline, lambda s: s.startswith("DIST,"), ok)
                window["-STATUS-"].update("Measure sent…")
            except Exception as e:
                window["-STATUS-"].update(f"Measure error: {e}")

        if event == "-LDR-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            mcu.send_line("STATE,3")
            deadline = time.time() + 2.0
            def ok(line):
                if line is None:
                    window["-STATUS-"].update("No LDR reply (timeout).")
                elif line.startswith("LDR,"):
                    parts = [p.strip() for p in line.split(",")]
                    if len(parts) == 3:
                        window["-STATUS-"].update(f"LDR raw={parts[1]}  mv={parts[2]}")
                        table_rows.append(("LDR", "-", f"{parts[1]}/{parts[2]} mV"))
                        window["-TABLE-"].update(values=table_rows)

            pending["LDR"] = (deadline, lambda s: s.startswith("LDR,"), ok)
            window["-STATUS-"].update("LDR requested…")

        if event == "-BTN-LDR-CAL-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            if sweep_active or ldr_collect_active or combo_collect_active:
                window["-STATUS-"].update("Another operation is running; wait for it to finish.")
                continue
            try:
                mcu.send_line("STATE,6")
                window["-STATUS-"].update(
                    "Starting LDR calibration… follow on-screen prompts; press PB0 for each sample.")
            except Exception as e:
                window["-STATUS-"].update(f"Calibration TX error: {e}")

        if event == "-BTN-LDR-CAL-DUMP-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            if sweep_active or ldr_collect_active or combo_collect_active or calib_active:
                window["-STATUS-"].update("Another operation is running; wait for it to finish.")
                continue
            try:
                calib_dump_active = True
                calib_dump_rows.clear()
                mcu.send_line("CAL,DUMP")
                window["-STATUS-"].update("Fetching calibration from FLASH…")
            except Exception as e:
                calib_dump_active = False
                window["-STATUS-"].update(f"Calibration dump TX error: {e}")

        if event == "-FILE-LS-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
            else:
                try:
                    mcu.send_line("FILE,LS")
                    # Use pending one-shot to catch the line and update list
                    deadline = time.time() + 1.0

                    def on_ls(line):
                        nonlocal files_list
                        if line is None:
                            window["-STATUS-"].update("FILE,LS timeout.")
                            return
                        if not line.startswith("FILE,LS,"):
                            window["-STATUS-"].update(f"Unexpected LS reply: {line}")
                            return

                        parts = line.split(",", 3)
                        display = []
                        if len(parts) == 4 and parts[3]:
                            items = [i for i in parts[3].split(";") if i]
                            for it in items:
                                if ":" in it:
                                    nm, sz = it.split(":", 1)
                                    try:
                                        szi = int(sz)
                                        display.append(f"{nm} ({szi} B)")
                                    except ValueError:
                                        display.append(nm)  # fallback if size isn't an int
                                else:
                                    display.append(it)  # backwards-compat (old FW)
                        files_list = display
                        window["-FILE-LIST-"].update(values=files_list)
                        window["-STATUS-"].update(f"{len(files_list)} file(s) on device.")

                    pending["FILELS"] = (deadline, lambda s: s.startswith("FILE,LS,"), on_ls)
                except Exception as e:
                    window["-STATUS-"].update(f"FILE,LS error: {e}")

        if event == "-FILE-DEL-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            sel = values.get("-FILE-LIST-", [])
            name = sel[0] if sel else values.get("-FILE-NAME-", "").strip()
            if " (" in name:  # strip " (123 B)"
                name = name.split(" (", 1)[0]
            if not name:
                window["-STATUS-"].update("Select a file or enter name to delete.")
                continue
            try:
                mcu.send_line(f"FILE,DEL,{name}")
                deadline = time.time() + 1.0
                def on_del(line):
                    if line is None:
                        window["-STATUS-"].update("FILE,DEL timeout.")
                        return
                    if line.startswith("FILE,DEL,OK"):
                        window["-STATUS-"].update(f"Deleted {name}.")
                        # auto-refresh list
                        window.write_event_value("-FILE-LS-", None)
                    else:
                        window["-STATUS-"].update(f"Delete failed: {line}")
                pending["FILEDEL"] = (deadline, lambda s: s.startswith("FILE,DEL,"), on_del)
            except Exception as e:
                window["-STATUS-"].update(f"FILE,DEL error: {e}")

        if event == "-FILE-UPLOAD-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue

            path = (values.get("-FILE-PATH-", "") or "").strip()
            name = (values.get("-FILE-NAME-", "") or "").strip()

            if not path:
                window["-STATUS-"].update("Choose a PC file to upload.")
                continue

            if not name:
                import os
                name = os.path.basename(path)
                window["-FILE-NAME-"].update(name)

            if len(name) > 12:
                window["-STATUS-"].update("Device filename must be ≤12 chars.")
                continue
            if "," in name:
                window["-STATUS-"].update("Device filename cannot contain commas (,).")
                continue

            # Read file
            try:
                with open(path, "rb") as f:
                    data = f.read()
            except Exception as e:
                window["-STATUS-"].update(f"Read failed: {e}")
                continue
            if len(data) == 0:
                window["-STATUS-"].update("Empty file; nothing to upload.")
                continue
            if len(data) > FILE_RX_LIMIT:
                window["-STATUS-"].update(f"File too big for Phase-1 (>{FILE_RX_LIMIT} B).")
                continue

            # Decide file type (0=text, 1=script) – adjust if you track types elsewhere
            ftype = 0 if name.lower().endswith(".txt") else 1

            # Progress UI on
            _upload_ui(True, name=name, total_chunks=0, done_chunks=0)
            window["-STATUS-"].update(f"Uploading {name}… {len(data)} bytes")

            # Run upload + auto-rebuild in a worker thread
            def _do_upload():
                try:
                    # Tell UI we’re busy (disable buttons)
                    window.write_event_value("-GUI-BUSY-", True)
                    window.write_event_value("-GUI-UP-PROG-", ("start", name, len(data)))

                    # Take exclusive ownership
                    mcu.pause_reader()
                    try:
                        mcu.ser.reset_input_buffer()  # drop any pending HB/READY/B
                    except Exception:
                        pass
                    # Flush GUI queue so no stale lines interfere
                    while True:
                        try:
                            rx_q.get_nowait()
                        except queue.Empty:
                            break

                    client = FileClient(
                        mcu.ser,
                        log_fn=lambda s: window.write_event_value("-GUI-LOG-", s),
                        progress_cb=lambda done, total: window.write_event_value("-GUI-UP-PROG-", ("tick", done, total))
                    )

                    client.upload_with_rebuild(name, ftype, data, backup_to_disk_dir=None)

                    window.write_event_value("-GUI-UP-PROG-", ("done", name))
                except Exception as ex:
                    window.write_event_value("-GUI-UP-PROG-", ("err", str(ex)))
                finally:
                    mcu.resume_reader()
                    window.write_event_value("-GUI-BUSY-", False)

            threading.Thread(target=_do_upload, daemon=True).start()

        if event == "-FILE-VIEW-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue
            sel = values.get("-FILE-LIST-", [])
            name = sel[0] if sel else values.get("-FILE-NAME-", "").strip()
            if " (" in name:  # strip " (123 B)"
                name = name.split(" (", 1)[0]
            if not name:
                window["-STATUS-"].update("Select/enter a file name first.")
                continue
            # Start a GET; we won't set file_dl_save_path so nothing auto-saves
            view_after_download = True
            file_dl_save_path = None
            try:
                mcu.send_line(f"FILE,GET,{name}")
                window["-STATUS-"].update(f"Requested GET for {name}…")
            except Exception as e:
                view_after_download = False
                window["-STATUS-"].update(f"FILE,GET error: {e}")

        if event == "-FILE-SHOW-LCD-":
            if not mcu.is_open():
                window["-STATUS-"].update("Not connected.")
                continue

            # Prefer selected listbox item; fallback to text field
            sel = values.get("-FILE-LIST-", [])
            name = sel[0] if sel else values.get("-FILE-NAME-", "").strip()

            # If listbox displays "name (123 B)", strip the size suffix
            if " (" in name:
                name = name.split(" (", 1)[0]

            try:
                if name:
                    # Directly show a specific file on the MSP LCD
                    mcu.send_line(f"FILE,SHOW,{name}")
                    window["-STATUS-"].update(
                        f"Showing '{name}' on LCD (PB0 = next 32 chars)…")
                else:
                    # No name given → open the MSP-side file selector
                    mcu.send_line("FILE,SHOW")
                    window["-STATUS-"].update(
                        "LCD file browser opened (PB0 = next file, PB1 = select)…")

                lcd_show_active = True
                _set_lcd_show_ui(True)

                # We do not wait for a reply; device replies after user exits viewer.
            except Exception as e:
                window["-STATUS-"].update(f"FILE,SHOW error: {e}")

        if event == "-FILE-EXIT-LCD-":
            try:
                mcu.send_line("FILE,SHOW,EXIT")  # Trip the viewer’s escape hatch
                window["-STATUS-"].update("Requested to exit LCD mode…")
                window["-FILE-EXIT-LCD-"].update(disabled=True)  # prevent spamming
                lcd_exit_until = time.time() + 2.0
                lcd_exit_next = time.time()
            except Exception as e:
                window["-STATUS-"].update(f"Exit request failed: {e}")

        if event == "-LCD-SHOW-DONE-":
            lcd_show_active = False
            lcd_exit_until = 0.0
            _set_lcd_show_ui(False)
            window["-STATUS-"].update("Exited LCD file mode.")

        if event == "-GUI-BUSY-":
            busy = bool(values["-GUI-BUSY-"])
            for k in ("-FILE-LS-", "-FILE-UPLOAD-", "-FILE-VIEW-", "-FILE-SHOW-LCD-", "-FILE-DEL-",
                      "-PING-", "-ECHO-",
                      "-BTN-US-SWEEP-", "-BTN-LDR-SWEEP-", "-BTN-COMBO-SWEEP-",
                      "-MEASURE-", "-LDR-", "-BTN-LDR-CAL-", "-BTN-LDR-CAL-DUMP-"):
                window[k].update(disabled=busy)

            # Smoke Test + Control tabs (add this block)
            for k in ("-PING-", "-ECHO-",
                      "-BTN-US-SWEEP-", "-BTN-LDR-SWEEP-", "-BTN-COMBO-SWEEP-",
                      "-MEASURE-", "-LDR-", "-BTN-LDR-CAL-", "-BTN-LDR-CAL-DUMP-"):
                window[k].update(disabled=busy)

        if event in ("-FILE-BROWSE-", "-FILE-PATH-"):
            path = (values.get("-FILE-PATH-", "") or "").strip()
            if path:
                import os
                window["-FILE-NAME-"].update(os.path.basename(path))

        # animate spinner while uploading
        # animate spinner while a worker upload is active
        if uploading:
            spinner_i = (spinner_i + 1) % len(SPINNER)
            window['-FILE-PROG-TXT-'].update(f"{SPINNER[spinner_i]} Uploading {upload_name}…")

        # Retry "FILE,SHOW,EXIT" a few times while LCD mode is active
        if lcd_show_active and lcd_exit_until:
            now = time.time()
            if now >= lcd_exit_next:
                try:
                    mcu.send_line("FILE,SHOW,EXIT")
                except Exception:
                    pass
                lcd_exit_next = now + 0.12
            if now >= lcd_exit_until:
                lcd_exit_until = 0.0  # stop retrying


        # Handle timeouts for pending waits (no 'line' here)
        now = time.time()
        for key in list(pending.keys()):
            deadline, match_fn, on_ok = pending[key]
            if now > deadline:
                on_ok(None)  # let the callback know it timed out
                del pending[key]

    # Cleanup
    mcu.close()
    window.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)

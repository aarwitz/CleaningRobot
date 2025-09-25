#!/usr/bin/env python3
"""send_sequence.py

Send a pick/place sequence (JSON messages) to a serial port
with configurable *per-step* delays. Includes a --dry-run option.
"""

import time
import argparse
import json

try:
    import serial
except Exception:
    serial = None


def make_sequence(pick_base, pick_shoulder, pick_elbow, pick_hand=3.145):
    """Generate a sequence for a pick at the given joint values."""
    return [
        # (msg, delay_in_seconds)
        ({"T":102, "base":0.0, "shoulder":0.0, "elbow":1.35, "hand":0, "spd":0, "acc":20}, 1.0),  # Home
        ({"T":102, "base":pick_base, "shoulder":0.8, "elbow":1.8, "hand":0, "spd":0, "acc":20}, 1.3),  # Above pick
        ({"T":102, "base":pick_base, "shoulder":pick_shoulder, "elbow":pick_elbow, "hand":pick_hand, "spd":0, "acc":20}, 1.8),  # Pick
        ({"T":102, "base":-1.465, "shoulder":0.0, "elbow":1.3, "hand":pick_hand, "spd":0, "acc":20}, 1.5),  # Above trash
        ({"T":106, "cmd":1.2, "spd":0, "acc":20}, 1.5),  # Drop
    ]


# Canonical return-home step used at the end of a run
RETURN_HOME_STEP = ({"T":102, "base":0.0, "shoulder":0.0, "elbow":1.35, "hand":0, "spd":0, "acc":20}, 1.0)


# Define multiple sequences by changing only the pick parameters
# SEQUENCES = { #radians: base, shoulder, elbow
#     "A": make_sequence(-0.2, 1.076, 1.75),    # Sequence A pick coords
#     "B": make_sequence(-.6, 1.07, 1.79),  # Sequence C pick coords
#     "C": make_sequence(0.43, 1.07, 1.75),   # Sequence D pick coords
#     "D": make_sequence(1.82, 1.07, 1.82),   # Sequence B pick coords
# }
SEQUENCES = { #radians: base, shoulder, elbow
    "A": make_sequence(-0.1, 1.076, 1.85),    # Sequence A pick coords
    "B": make_sequence(-.5, 1.07, 1.8),  # Sequence C pick coords
    "C": make_sequence(0.54, 1.07, 1.84),   # Sequence D pick coords
    "D": make_sequence(1.82, 1.07, 1.86),   # Sequence B pick coords
}


def send_sequence(port: str, sequence, dry_run: bool = False):
    """Send the sequence to the serial port, respecting per-step delays."""
    if dry_run:
        print("Dry-run mode: will print messages instead of sending to serial")
    else:
        if serial is None:
            raise RuntimeError("pyserial is required to send to a serial port. Install with: pip install pyserial")

    ser = None
    final_sent = False
    final_step = sequence[-1] if sequence else None
    try:
        if not dry_run:
            ser = serial.Serial(port, baudrate=115200, dsrdtr=None)
            ser.setRTS(False)
            ser.setDTR(False)
            time.sleep(0.1)

        for i, (msg, delay) in enumerate(sequence, start=1):
            line = json.dumps(msg, separators=(',', ':'))
            print(f"Step {i}: Sending {line} (delay {delay}s)")
            if not dry_run:
                ser.write(line.encode('utf-8') + b"\n")
            # If this is the final step in the sequence, mark it sent
            if final_step is not None and msg == final_step[0]:
                final_sent = True
            time.sleep(delay)

    finally:
        # Ensure we attempt to send the final return-home step if it wasn't
        # sent (for example, an exception happened mid-run). If dry-run,
        # just print. If serial is open, send the message.
        if final_step is not None and not final_sent:
            msg, delay = final_step
            print(f"Final step not sent during run; attempting to send final step: {json.dumps(msg, separators=(',', ':'))} (delay {delay}s)")
            if dry_run:
                # print only
                pass
            else:
                if ser is not None:
                    try:
                        ser.write(json.dumps(msg, separators=(',', ':')).encode('utf-8') + b"\n")
                        time.sleep(delay)
                    except Exception as ex:
                        print(f"Failed to send final return-home: {ex}")

        if ser is not None:
            ser.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Send a pick/place JSON sequence to a serial port.')
    parser.add_argument('port', type=str, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--sequence', '-s', type=str, default="A", help='Sequence(s) to send. Use comma or + to separate multiple, e.g. "A,B" or "A+B"')
    parser.add_argument('--dry-run', action='store_true', help='Print messages instead of sending')
    args = parser.parse_args()

    # Allow multiple sequence names separated by commas or plus signs, e.g. "A,B" or "A+B"
    raw = args.sequence
    # split on comma or plus and strip whitespace
    names = [s.strip() for part in raw.split(',') for s in part.split('+') if s.strip()]
    if not names:
        parser.error('No sequence names provided')

    # Validate names
    unknown = [n for n in names if n not in SEQUENCES]
    if unknown:
        parser.error(f'Unknown sequence name(s): {", ".join(unknown)}. Valid names: {", ".join(sorted(SEQUENCES.keys()))}')

    # Concatenate the sequences in the requested order
    combined = []
    for n in names:
        seq = list(SEQUENCES[n])
        # If this is sequence C, add a small post-pick "raise" step so the arm
        # lifts after picking (avoid hitting the trash). We insert the step
        # immediately after the Pick step (which is the 3rd entry in make_sequence).
        if n == 'B':
            # Find index of pick step (msg matching a pick hand value) -- by
            # convention in make_sequence the pick is the third step (index 2).
            pick_index = 2
            # Create a raise step that lifts shoulder and elbow a decent amount
            # while keeping the picked hand closed. Adjust values conservatively.
            raise_step = (
                {"T":102, "base": seq[pick_index][0].get('base', 0.0),
                 "shoulder": seq[pick_index][0].get('shoulder', 0.0) - 0.55,
                 "elbow": seq[pick_index][0].get('elbow', 0.0) - 0.3,
                 "hand": seq[pick_index][0].get('hand', 3.145), "spd":0, "acc":20},
                0.4,
            )
            # Insert after pick
            seq.insert(pick_index + 1, raise_step)

        # If this is not the last requested sequence, remove its final
        # "return home" step so the arm continues directly to the next
        # sequence instead of traveling home and back again.
        if n != names[-1] and seq:
            last_msg = seq[-1][0]
            def _is_return_home(msg):
                return (
                    isinstance(msg, dict) and
                    msg.get('T') == 102 and
                    msg.get('base') == 0.0 and
                    msg.get('shoulder') == 0.0 and
                    msg.get('elbow') == 1.35 and
                    msg.get('hand') == 0
                )

            if _is_return_home(last_msg):
                seq.pop()

        combined.extend(seq)

    # Ensure a single canonical return-home at the end
    if not combined or combined[-1][0] != RETURN_HOME_STEP[0]:
        combined.append(RETURN_HOME_STEP)

    send_sequence(args.port, combined, dry_run=args.dry_run)

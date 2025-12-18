#!/usr/bin/env python3
"""
CAN Message Timing Analysis Tool

Filters CAN messages by ID and/or data bytes, and shows timing deltas
with optional histogram/bucket analysis.

Requires: pip install python-can

Usage:
    ./can_timing_analysis.py                          # All messages
    ./can_timing_analysis.py --id 14EFA2              # Filter by ID pattern
    ./can_timing_analysis.py --id 14EFA2 --data 007D  # Filter by ID and data pattern
    ./can_timing_analysis.py --histogram              # Show timing histogram at end
"""

import argparse
import time
import sys
from collections import defaultdict

try:
    import can
except ImportError:
    print("Error: python-can not installed. Run: pip install python-can")
    sys.exit(1)


def format_message(msg):
    """Format a CAN message for display."""
    data_hex = ' '.join(f'{b:02X}' for b in msg.data)
    return f"{msg.arbitration_id:08X}  [{len(msg.data)}]  {data_hex}"


def matches_filter(msg, id_pattern=None, data_pattern=None):
    """Check if message matches the filters."""
    # Check ID pattern (hex string match)
    if id_pattern:
        msg_id_hex = f"{msg.arbitration_id:08X}"
        if id_pattern.upper() not in msg_id_hex.upper():
            return False
    
    # Check data pattern (hex string match anywhere in data)
    if data_pattern:
        data_hex = ''.join(f'{b:02X}' for b in msg.data)
        if data_pattern.upper() not in data_hex.upper():
            return False
    
    return True


def bucket_delta(delta_ms):
    """Assign a delta to a timing bucket."""
    if delta_ms < 5:
        return "0-5ms"
    elif delta_ms < 10:
        return "5-10ms"
    elif delta_ms < 20:
        return "10-20ms"
    elif delta_ms < 30:
        return "20-30ms"
    elif delta_ms < 40:
        return "30-40ms"
    elif delta_ms < 50:
        return "40-50ms"
    elif delta_ms < 55:
        return "50-55ms"
    elif delta_ms < 60:
        return "55-60ms"
    elif delta_ms < 65:
        return "60-65ms"
    elif delta_ms < 70:
        return "65-70ms"
    elif delta_ms < 75:
        return "70-75ms"
    elif delta_ms < 80:
        return "75-80ms"
    elif delta_ms < 85:
        return "80-85ms"
    elif delta_ms < 90:
        return "85-90ms"
    elif delta_ms < 95:
        return "90-95ms"
    elif delta_ms < 100:
        return "95-100ms"
    elif delta_ms < 150:
        return "100-150ms"
    elif delta_ms < 200:
        return "150-200ms"
    elif delta_ms < 500:
        return "200-500ms"
    elif delta_ms < 1000:
        return "500ms-1s"
    else:
        return ">1s"


def main():
    parser = argparse.ArgumentParser(description='CAN Message Timing Analysis')
    parser.add_argument('--interface', '-i', default='can0', help='CAN interface (default: can0)')
    parser.add_argument('--id', dest='id_pattern', help='Filter by CAN ID pattern (hex, e.g., 14EFA2)')
    parser.add_argument('--data', dest='data_pattern', help='Filter by data pattern (hex, e.g., 007D)')
    parser.add_argument('--histogram', '-H', action='store_true', help='Show timing histogram on exit')
    parser.add_argument('--count', '-n', type=int, help='Stop after N matching messages')
    parser.add_argument('--bustype', '-b', default='socketcan', help='CAN bus type (default: socketcan)')
    
    args = parser.parse_args()
    
    print(f"Connecting to {args.interface}...")
    print(f"Filters: ID={args.id_pattern or 'any'}, Data={args.data_pattern or 'any'}")
    print("Press Ctrl+C to stop\n")
    print(f"{'Delta (ms)':>12}  {'CAN ID':>10}  {'Len':>5}  {'Data':<24}")
    print("-" * 60)
    
    try:
        bus = can.interface.Bus(channel=args.interface, bustype=args.bustype)
    except Exception as e:
        print(f"Error opening {args.interface}: {e}")
        sys.exit(1)
    
    last_time = None
    deltas = []
    buckets = defaultdict(int)
    count = 0
    
    try:
        for msg in bus:
            if not matches_filter(msg, args.id_pattern, args.data_pattern):
                continue
            
            current_time = msg.timestamp
            
            if last_time is not None:
                delta = (current_time - last_time) * 1000  # Convert to ms
                deltas.append(delta)
                buckets[bucket_delta(delta)] += 1
                delta_str = f"{delta:>10.3f}ms"
            else:
                delta_str = f"{'---':>12}"
            
            print(f"{delta_str}  {format_message(msg)}")
            
            last_time = current_time
            count += 1
            
            if args.count and count >= args.count:
                break
                
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        bus.shutdown()
    
    # Print statistics
    if deltas:
        print("\n" + "=" * 60)
        print("STATISTICS")
        print("=" * 60)
        print(f"Messages:  {count}")
        print(f"Min delta: {min(deltas):.3f} ms")
        print(f"Max delta: {max(deltas):.3f} ms")
        print(f"Avg delta: {sum(deltas)/len(deltas):.3f} ms")
        
        if args.histogram:
            print("\nTIMING HISTOGRAM")
            print("-" * 40)
            bucket_order = ["0-5ms", "5-10ms", "10-20ms", "20-30ms", "30-40ms", "40-50ms",
                          "50-55ms", "55-60ms", "60-65ms", "65-70ms", "70-75ms", 
                          "75-80ms", "80-85ms", "85-90ms", "90-95ms", "95-100ms",
                          "100-150ms", "150-200ms", "200-500ms", "500ms-1s", ">1s"]
            max_count = max(buckets.values()) if buckets else 1
            for bucket in bucket_order:
                cnt = buckets.get(bucket, 0)
                bar = '#' * int(40 * cnt / max_count) if cnt > 0 else ''
                print(f"{bucket:>12}: {cnt:>5} {bar}")


if __name__ == '__main__':
    main()

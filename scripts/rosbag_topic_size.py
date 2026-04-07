#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

try:
    import rosbag2_py
except ImportError:
    print("Error: rosbag2_py not found. Ensure your ROS 2 environment is sourced.")
    sys.exit(1)

def format_size(size):
    if size < 1000:
        return f"{size:7d}B"
    elif size >= 1000000000000:
        return f"{size/1000000000000:7.2f}T"
    elif size >= 1000000000:
        return f"{size/1000000000:7.2f}G"
    elif size >= 1000000:
        return f"{size/1000000:7.2f}M"
    elif size >= 1000:
        return f"{size/1000:7.2f}K"

def main():
    parser = argparse.ArgumentParser(
        description='Display topics in a ROS 2 mcap rosbag with sizes, rates, and durations.')
    parser.add_argument('path_to_rosbag', help='Path to rosbag directory or .mcap file')
    args = parser.parse_args()

    bag_path = Path(args.path_to_rosbag)

    if bag_path.is_dir():
        mcap_files = list(bag_path.glob("*.mcap"))
        if not mcap_files:
            print("No .mcap files found in the directory.")
            sys.exit(1)
        file_size = sum(f.stat().st_size for f in mcap_files)
    else:
        file_size = bag_path.stat().st_size

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id='mcap'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Failed to open bag: {e}")
        sys.exit(1)

    topic_size_dict = {}
    topic_count_dict = {}
    topic_first_ts = {}
    topic_last_ts = {}

    total_size = 0
    size_part = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_size = len(data)
        
        # Size tracking
        topic_size_dict[topic] = topic_size_dict.get(topic, 0) + msg_size
        total_size += msg_size
        size_part += msg_size
        
        # Rate and Duration tracking
        topic_count_dict[topic] = topic_count_dict.get(topic, 0) + 1
        if topic not in topic_first_ts:
            topic_first_ts[topic] = timestamp
        topic_last_ts[topic] = timestamp
        
        if size_part > file_size / 10:
            print(f"Processed {total_size / 1000**2:.2f} / {file_size / 1000**2:.2f} MB.")
            size_part %= file_size / 10

    if not topic_first_ts:
        print("No messages found in the bag.")
        sys.exit(0)

    # Calculate overall bag limits
    global_first_ts = min(topic_first_ts.values())
    global_last_ts = max(topic_last_ts.values())
    bag_duration_s = (global_last_ts - global_first_ts) / 1e9

    results = []
    for topic, size in topic_size_dict.items():
        count = topic_count_dict[topic]
        first_ts = topic_first_ts[topic]
        last_ts = topic_last_ts[topic]
        
        duration_s = (last_ts - first_ts) / 1e9
        
        if duration_s > 0 and count > 1:
            rate = (count - 1) / duration_s
        else:
            rate = 0.0
            
        results.append((size, rate, duration_s, topic))

    # Sort results by cumulative size
    results.sort(key=lambda x: x[0])

    print("-" * 70)
    for size, rate, duration_s, topic in results:
        size_str = format_size(size)
        print(f"{size_str}  {rate:6.1f}Hz  {duration_s:8.1f}s  {topic}")

    print("-" * 70)
    print("[size] [rate] [duration] [topic name]")
    print(f"\nOverall Rosbag Stats:")
    print(f"  Total Duration: {bag_duration_s:.1f} seconds")
    print(f"  Total Size:     {total_size / 1000**2:.2f} MB")

if __name__ == '__main__':
    main()

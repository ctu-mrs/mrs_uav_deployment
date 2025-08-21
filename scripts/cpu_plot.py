import pandas as pd
import matplotlib.pyplot as plt

def parse_pidstat_log(filename, cpu_min_threshold=5):
    times = []
    pids = []
    cpus = []
    cmds = []

    current_time = None
    with open(filename, 'r') as f:
        for line in f:
            if 'PID' in line.split():
                continue

            if 'Timestamp:' in line.split():
                current_time = line.split()[2]
                continue

            parts = line.split()
            pid = int(parts[0])
            cpu = float(parts[2])

            if cpu <= cpu_min_threshold:
                continue

            cmd = " ".join(parts[-1])
            times.append(current_time)
            pids.append(pid)
            cpus.append(cpu)
            cmds.append(cmd)

    df = pd.DataFrame({
        'Time': pd.to_datetime(times, errors='coerce'),
        'PID': pids,
        '%CPU': cpus,
        'Command': cmds
    })

    # Drop rows with invalid timestamps
    df.dropna(subset=['Time'], inplace=True)
    return df

def plot_cpu_usage(df):
    plt.figure(figsize=(12, 8))

    pids = df['PID'].unique()
    for pid in pids:
        df_pid = df[df['PID'] == pid]
        plt.plot(df_pid['Time'], df_pid['%CPU'], label=f"PID {pid})")

    plt.xlabel('Time')
    plt.ylabel('CPU Usage (%)')
    plt.title('CPU Usage per Process Over Time')
    plt.legend(loc='upper right', fontsize='small', ncol=2)
    plt.tight_layout()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 3:
        print("Usage: python cpu_plot.py <top_log_file> <cpu_min_threshold_percentage>")
        sys.exit(1)

    filename = sys.argv[1]
    cpu_min_threshold = 0
    df = parse_pidstat_log(filename, cpu_min_threshold)
    if df.empty:
        print("No valid data found in log file.")
        sys.exit(1)
    plot_cpu_usage(df)

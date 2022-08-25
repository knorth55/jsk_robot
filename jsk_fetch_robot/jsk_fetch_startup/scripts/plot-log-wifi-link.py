#!/usr/bin/env python

from matplotlib import pyplot as plt
import csv
import argparse
import time
from datetime import datetime


def parse_date(raw_str):
    return datetime.strptime(
            raw_str.split('+')[0],
            '%Y-%m-%dT%H:%M:%S'
            )


def load_csv(csv_file, start_time, end_time):

    list_time = []
    list_ssid = []
    list_freq = []
    list_signal = []
    list_level = []
    list_noise = []
    list_ping = []

    with open(csv_file) as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='"')
        for row in spamreader:
            try:
                row_time = parse_date(row[0])
                row_ssid = row[1]
                row_freq = int(row[2].strip('.'))
                row_signal = int(row[3].strip('.'))
                row_level = int(row[4].strip('.'))
                row_noise = int(row[5])
                row_ping = float(row[7])

                list_time.append(row_time)
                list_ssid.append(row_ssid)
                list_freq.append(row_freq)
                list_signal.append(row_signal)
                list_level.append(row_level)
                list_noise.append(row_noise)
                list_ping.append(row_ping)
            except (ValueError, IndexError) as e:
                pass

    if start_time is not None:

        start_time = parse_date(start_time)
        list_ssid = [ssid for ssid, t in zip(list_ssid, list_time) if t >= start_time]
        list_freq = [freq for freq, t in zip(list_freq, list_time) if t >= start_time]
        list_signal = [signal for signal, t in zip(list_signal, list_time) if t >= start_time]
        list_level = [level for level, t in zip(list_level, list_time) if t >= start_time]
        list_noise = [noise for noise, t in zip(list_noise, list_time) if t >= start_time]
        list_ping = [ping for ping, t in zip(list_ping, list_time) if t >= start_time]
        list_time = [t for t in list_time if t >= start_time]

    if end_time is not None:

        end_time = parse_date(end_time)
        list_ssid = [ssid for ssid, t in zip(list_ssid, list_time) if t < end_time]
        list_freq = [freq for freq, t in zip(list_freq, list_time) if t < end_time]
        list_signal = [signal for signal, t in zip(list_signal, list_time) if t < end_time]
        list_level = [level for level, t in zip(list_level, list_time) if t < end_time]
        list_noise = [noise for noise, t in zip(list_noise, list_time) if t < end_time]
        list_ping = [ping for ping, t in zip(list_ping, list_time) if t < end_time]
        list_time = [t for t in list_time if t < end_time]

    return list_time, list_ssid, list_freq, list_signal, list_level, list_noise, list_ping


if __name__=='__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file')
    parser.add_argument('--start-time', default=None)
    parser.add_argument('--end-time', default=None)
    parser.add_argument('--num-labels', default=5)

    args = parser.parse_args()

    list_time, list_ssid, list_freq, list_signal, list_level, list_noise, list_ping = load_csv(args.csv_file, args.start_time, args.end_time)
    list_time_unixtime = [int(time.mktime(t.timetuple())) for t in list_time]

    if args.start_time is not None:
        start_time = parse_date(args.start_time)
    else:
        start_time = list_time[0]
    start_time_unixtime = int(time.mktime(start_time.timetuple()))

    if args.end_time is not None:
        end_time = parse_date(args.end_time)
    else:
        end_time = list_time[-1]
    end_time_unixtime = int(time.mktime(end_time.timetuple()))

    fig, axes = plt.subplots(4,1,sharex=True)
    scatter_size=10
    axes[0].scatter(list_time_unixtime, list_signal, s=scatter_size)
    axes[0].set_ylabel('signal')
    axes[0].set_ylim((45,70))
    axes[1].scatter(list_time_unixtime, list_level, s=scatter_size)
    axes[1].set_ylabel('level')
    axes[1].set_ylim((-65,-40))
    axes[2].scatter(list_time_unixtime, list_noise, s=scatter_size)
    axes[2].set_ylabel('noise')
    axes[2].set_ylim((-300,-200))
    axes[3].scatter(list_time_unixtime, list_ping, s=scatter_size)
    axes[3].set_ylabel('ping')
    axes[3].set_ylim((-1100, 2200))
    axes[3].set_xlim((start_time_unixtime, end_time_unixtime))

    list_time_xticklabels = [
            start_time + i * ( (end_time - start_time) / (args.num_labels - 1) )
            for i in range(args.num_labels)]
    list_time_xticks = [int(time.mktime(t.timetuple())) for t in list_time_xticklabels]

    axes[3].set_xticks(list_time_xticks)
    axes[3].set_xticklabels(list_time_xticklabels, rotation=15)
    plt.show()

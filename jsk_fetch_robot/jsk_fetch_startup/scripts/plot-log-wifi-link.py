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

    if start_time is not None:

        start_time = parse_date(start_time)
        list_ssid = list_ssid[list_time >= start_time]
        list_freq = list_freq[list_time >= start_time]
        list_signal = list_signal[list_time >= start_time]
        list_level = list_level[list_time >= start_time]
        list_noise = list_noise[list_time >= start_time]
        list_ping = list_ping[list_time >= start_time]
        list_time = list_time[list_time >= start_time]

    if end_time is not None:

        end_time = parse_date(end_time)
        list_ssid = list_ssid[list_time < end_time]
        list_freq = list_freq[list_time < end_time]
        list_signal = list_signal[list_time < end_time]
        list_level = list_level[list_time < end_time]
        list_noise = list_noise[list_time < end_time]
        list_ping = list_ping[list_time < end_time]
        list_time = list_time[list_time < end_time]

    return list_time, list_ssid, list_freq, list_signal, list_level, list_noise, list_ping


if __name__=='__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file')
    parser.add_argument('--start-time', default=None)
    parser.add_argument('--end-time', default=None)

    args = parser.parse_args()

    list_time, list_ssid, list_freq, list_signal, list_level, list_noise, list_ping = load_csv(args.csv_file, args.start_time, args.end_time)
    list_time_unixtime = [int(time.mktime(t.timetuple())) for t in list_time]

    fig, axes = plt.subplots(4,1,sharex=True)
    axes[0].scatter(list_time_unixtime, list_signal)
    axes[0].set_ylabel('signal')
    axes[1].scatter(list_time_unixtime, list_level)
    axes[1].set_ylabel('level')
    axes[2].scatter(list_time_unixtime, list_noise)
    axes[2].set_ylabel('noise')
    axes[3].scatter(list_time_unixtime, list_ping)
    axes[3].set_ylabel('ping')
    plt.show()

#!/usr/bin/env python

from matplotlib import pyplot as plt
import csv
import argparse
import time
from datetime import datetime


def parse_date(raw_str):
    return datetime.strptime(raw_str,'%Y-%m-%dT%H:%M:%S+09:00')


def load_csv(csv_file):

    list_time = []
    list_ssid = []
    list_freq = []
    list_signal = []
    list_level = []
    list_noise = []

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

                list_time.append(row_time)
                list_ssid.append(row_ssid)
                list_freq.append(row_freq)
                list_signal.append(row_signal)
                list_level.append(row_level)
                list_noise.append(row_noise)
            except Error as e:
                print('got an error {}'.format(e))

    return list_time, list_ssid, list_freq, list_signal, list_level, list_noise


if __name__=='__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file')

    args = parser.parse_args()

    list_time, list_ssid, list_freq, list_signal, list_level, list_noise = load_csv(args.csv_file)
    list_time_unixtime = [int(time.mktime(t.timetuple())) for t in list_time]

    fig, axes = plt.subplots(3,1,sharex=True)
    axes[0].scatter(list_time_unixtime, list_signal)
    axes[0].set_title('signal')
    axes[1].scatter(list_time_unixtime, list_level)
    axes[1].set_title('level')
    axes[2].scatter(list_time_unixtime, list_noise)
    axes[2].set_title('noise')
    plt.show()

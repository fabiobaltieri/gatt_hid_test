#!/bin/env python

import datetime
import binascii

DEVICE = "/dev/hidrawX" # fix this
MSG_TH = 150

ts = datetime.datetime.now()

times = []
ts = datetime.datetime.now()
with open(DEVICE, mode='rb', buffering=0) as fp:
    while True:
        data = fp.read(128)

        new_ts = datetime.datetime.now()
        delta = new_ts - ts
        ts = new_ts
        times.append(delta.microseconds / 1000)
        if len(times) >= MSG_TH:
            ts_min = min(times)
            ts_avg = sum(times) / len(times)
            ts_max = max(times)
            freq = 1000.0 / ts_avg
            data_hex = binascii.b2a_hex(data)
            data_len = len(data)
            print(f"msg: {data_hex} ({data_len}) "
                  f"{ts_min:6.3f} ms {ts_avg:6.3f} ms ({freq:3.0f} Hz) {ts_max:6.3f} ms")
            times = []

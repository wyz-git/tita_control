#!/bin/bash
# 获取设备号
device=$(arecord -l | grep -m1 'UACDemoV1.0' | awk -F'[:, ]+' '{print "hw:"$2","$6}')
if [ -z "$device" ]; then
  echo "ERROR: Device not found!" >&2
  exit 1
fi

# 启动推流
ffmpeg -loglevel error \
  -f alsa \
  -channels 1 \
  -thread_queue_size 4096 \
  -i "$device" \
  -ac 1 \
  -acodec libmp3lame -b:a 128k -ar 44100 \
  -f mpegts \
  "srt://119.23.220.15:8890?streamid=publish:live1"
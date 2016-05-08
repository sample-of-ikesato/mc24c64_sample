#!/usr/bin/env ruby
# -*- coding: utf-8 -*-

$LOAD_PATH.push File.dirname($0)

require "rubygems"
require "serialport"
require 'optparse'

def usage()
  puts "Usage: #{File.basename($0)} [OPTION] USB-DEVICE"
  puts ""
  puts "Options:"
  puts "  -v, --verbose   output verbose format"
  puts "  -d, --debug     debug mode"
  puts "  -h, --help      display this help and exit"
  exit 1
end

def ave(ary)
  ary.inject {|sum,v| sum + v} / ary.count
end

debug=false
verbose=false
OptionParser.new {|opt|
  opt.on("--debug") {
    debug=true
  }
  opt.on("--verbose") {
    verbose=true
  }
  opt.on("--help") {
    usage
  }
  opt.parse!(ARGV)
}

usage if ARGV.length < 1
PORT=ARGV.shift
sp = SerialPort.new(PORT, 9600, 8, 1, 0) # 8bit, stopbit 1, parity none

BUFFLEN = 10

count=0
start = last = Time.now
dfp = File.open("debug.dat", "wb")
ofp = File.open("accel.csv", "w")
accel_x_on_buff = []
accel_x_off_buff = []
accel_y_on_buff = []
accel_y_off_buff = []
prevl = prevlg = 0
while true
  buf = sp.read(2)
  if buf == nil
    puts "Device detatched!"
    break
  end
  if buf.size < 2
    puts "Error: buffer needs 2 bytes."
    puts buf
    break
  end
  cmd, size = buf.unpack("C*")
  unless [0,1,2,3,4,9].include? cmd
    puts "unrecognize command"
    puts buf
    next
  end
  if size > 0
    data = sp.read(size).unpack("C*")
    if data.size < size
      puts "Errror: data needs #{size} bytes"
    end
  end
  if cmd == 4
    accel_x_on = data[0] + (data[1] << 8)
    accel_x_off = data[2] + (data[3] << 8)
    accel_y_on = data[4] + (data[5] << 8)
    accel_y_off = data[6] + (data[7] << 8)
    accel_x_on_buff.unshift(accel_x_on)
    accel_x_off_buff.unshift(accel_x_off)
    accel_y_on_buff.unshift(accel_y_on)
    accel_y_off_buff.unshift(accel_y_off)
    accel_x_on_buff = accel_x_on_buff[0...BUFFLEN]
    accel_x_off_buff = accel_x_off_buff[0...BUFFLEN]
    accel_y_on_buff = accel_y_on_buff[0...BUFFLEN]
    accel_y_off_buff = accel_y_off_buff[0...BUFFLEN]

    #x = ave(accel_x_on_buff)
    #y = ave(accel_y_on_buff)
    x = accel_x_on
    y = accel_y_on

    l = x**2 + y**2
    lg = Math.sqrt((((x.to_f/2664)-0.5)/0.3)**2 + (((y.to_f/2664)-0.5)/0.3)**2)
    ofp.puts [x, y, l, sprintf("%.6f",lg), l-prevl, sprintf("%.6f",lg-prevlg)].join("\t")
    ofp.flush
    prevl = l
    prevlg = lg

    diff = Time.now - start
    p [sprintf("%02d:%02d.%03d",diff/60, diff%60, (diff*1000)%1000), [accel_x_on, accel_x_off, accel_y_on, accel_y_off], [ave(accel_x_on_buff), ave(accel_x_off_buff), ave(accel_y_on_buff), ave(accel_y_off_buff)], [ave(accel_x_on_buff)+ave(accel_x_off_buff), ave(accel_y_on_buff)+ave(accel_y_off_buff)]]
  elsif cmd == 9
    dfp.write(data.pack("C*")) if size > 0
  end
end
puts ""

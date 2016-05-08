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
  if cmd == 9
    if size > 0
      puts data
      dfp.write(data.pack("C*"))
    end
  end
end
puts ""

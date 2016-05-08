#!/usr/bin/env ruby
# -*- coding: utf-8 -*-

$LOAD_PATH.push File.dirname($0)

require "rubygems"
require "serialport"
require 'optparse'

def usage()
  puts "Usage: #{File.basename($0)} [OPTION] USB-DEVICE FILENAME ADDRESS"
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

usage if ARGV.length < 3
PORT=ARGV.shift
sp = SerialPort.new(PORT, 9600, 8, 1, 0) # 8bit, stopbit 1, parity none

# receive
dfp = File.open("debugout.dat", "wb")
th = Thread.new do
  STDOUT.binmode
  rbegin = Time.now
  loop do
    data = sp.read(1)
    if data==nil
      puts "Device was detattched. [#{PORT}]"
      break
    end
    dfp.write(data) if data.size > 0
  end
  rend = Time.now
end

start = Time.now
fname = ARGV.shift
addr = ARGV.shift
fp = File.open(fname)
fsize = fp.size
count = 0
addr = addr =~ /^0x/i ? addr[2..-1].hex : addr.to_i
File.delete("debug2.dat")
while data = fp.read(0x20)
  header = [0x10, data.size].pack("C*")
  header += [addr].pack("S")
File.open("debug2.dat","ab"){|f| f.write(header+data)}
  sp.write(header + data)
  count += data.size
  addr += data.size
  diff = Time.now - start
  printf("\r%02d:%02d.%03d #{count}/#{fsize}", diff/60, diff%60, (diff * 1000)%1000)
end
diff = Time.now - start
printf("\r%02d:%02d.%03d #{count}/#{fsize}\n", diff/60, diff%60, (diff * 1000)%1000)
th.join
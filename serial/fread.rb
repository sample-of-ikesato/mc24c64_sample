#!/usr/bin/env ruby
# -*- coding: utf-8 -*-

$LOAD_PATH.push File.dirname($0)

require "rubygems"
require "serialport"
require 'optparse'

def usage()
  puts "Usage: #{File.basename($0)} [OPTION] USB-DEVICE ADDRESS SIZE"
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

count=0
start = last = Time.now
fp = File.open("out.dat", "wb")
addr = ARGV.shift
rsize = ARGV.shift
addr = addr =~ /^0x/i ? addr[2..-1].hex : addr.to_i
rsize = rsize =~ /^0x/i ? rsize[2..-1].hex : rsize.to_i

dfp = File.open("debugout.dat", "wb")

header = [0x11, [rsize, 0x20].min].pack("C*") + [addr].pack("S")
File.open("debug.dat","wb"){|f| f.write(header)}
sp.write(header)
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
  unless [0,1,2,3,4,9,0x10,0x11,0x12,0x90,0x91].include? cmd
    puts "unrecognize command"
    puts buf.unpack("C*").map{|x| sprintf("%02x", x)}.join(" ")
    next
  end
  if size > 0
    data = sp.read(size).unpack("C*")
    if data.size < size
      puts "Errror: data needs #{size} bytes"
    end
  end
  if cmd == 0x12
    fp.write(data.pack("C*"))
    rsize -= data.size
    addr += data.size
p ["aaaaaaaaaaaaaaaaaaaaaaa", rsize]
    break if rsize <= 0
    header = [0x11, [rsize, 0x20].min].pack("C*") + [addr].pack("S")
File.open("debug.dat","ab"){|f| f.write(header)}
    sp.write(header)
    diff = Time.now - start
    printf("\r%02d:%02d.%03d", diff/60, diff%60, (diff * 1000)%1000)
  elsif cmd == 0x90
    dfp.write(data.pack("C*")) if size > 0
  elsif cmd == 0x91
    puts data if size > 0
  end
end
diff = Time.now - start
printf("\r%02d:%02d.%03d\n", diff/60, diff%60, (diff * 1000)%1000)

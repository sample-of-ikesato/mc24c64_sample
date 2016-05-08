#!/usr/bin/env ruby
# -*- coding: utf-8 -*-

$LOAD_PATH.push File.dirname($0)

require "rubygems"
require "serialport"
require 'optparse'

def usage()
  puts "Usage: #{File.basename($0)} [OPTION] USB-DEVICE FILENAME"
  puts ""
  puts "Options:"
  puts "  -v, --verbose   output verbose format"
  puts "  -d, --debug     debug mode"
  puts "  -h, --help      display this help and exit"
  exit 1
end

debug=false
verbose=false
SPEED=19200
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

usage if ARGV.length < 2
PORT=ARGV.shift
fname = ARGV.shift
fsize = File.size(fname)
#File.open(fname, "rb") do |f|
#  while data = f.read(256)
#    data.each_char do |a|
#      putc a
#    end
#  end
#end

sp = SerialPort.new(PORT, SPEED, 8, 1, 0) # 8bit, stopbit 1, parity none

wbegin = wend = rbegin = rend = 0

# receive
th = Thread.new do
  remain = fsize
  STDOUT.binmode
  rbegin = Time.now
  loop do
    data = sp.read([remain, 256].min)
    if data==nil
      puts "Device was detattched. [#{PORT}]"
      break
    end
    remain -= data.size
    STDOUT.write(data)
    break if remain <= 0
  end
  rend = Time.now
end

# write
c=0
File.open(fname, "rb") do |f|
  wbegin = Time.now
  while data = f.read(256)
    data.each_char do |s|
      sp.write s # 1文字づつ転送する（バッファがあふれるので）
    end
    #STDERR.printf("\r%d", c+=data.length)
  end
  wend = Time.now
end

#STDERR.puts("wait to read")
th.join

wtime = wend - wbegin
rtime = rend - rbegin
STDERR.puts "File Size            : #{fsize}[byte]"
STDERR.puts "Serial Write Elapsed : #{wtime < 0 ? wtime : sprintf("%.2f", wtime)}[s]"
STDERR.puts "Serial Write Speed   : #{(fsize/wtime).to_i}[byte/s]"
STDERR.puts "Serial Read Elapsed  : #{rtime < 0 ? rtime : sprintf("%.2f", rtime)}[s]"
STDERR.puts "Serial Read Speed    : #{(fsize/rtime).to_i}[byte/s]"




## receive
#Thread.new do
#  begin
#    loop do
#      line = sp.gets
#      if line==nil
#        puts "Device was detattched. [#{PORT}]"
#        exit 1
#      end
#      line = line.scan(/[[:print:]]/).join
#      if line.index("received,")==0
#        r=RemoconAnalyzer.parse(line)
#        puts r.dump
#        puts r.verbose if verbose
#        puts line if debug
#      elsif line.index("echo,")==0
#        puts line if debug
#      else
#        puts line
#      end
#    end
#  rescue =>ex
#    Thread.main.raise ex
#  end
#end
#
## send
#loop do
#  str = STDIN.gets
#  data = RemoconAnalyzer.make_send_data(str)
#  next if data.nil?
#  (data+"\r\n").each_char {|s| sp.write s} # 1文字づつ転送する（バッファがあふれるので）
#end
#sp.close

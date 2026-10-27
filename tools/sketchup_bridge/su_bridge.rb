# -*- coding: utf-8 -*-
# =============================================================================
# su_bridge.rb — SketchUp <-> 外部进程 直连桥（装进 Plugins 目录即可）
# =============================================================================
#
# 原理:
#   SketchUp 只暴露「进程内 Ruby API」，没有对外的自动化接口、没有无头模式。
#   本扩展在 SketchUp 内起一个本地 TCP 服务，并在**主线程**用 UI.start_timer
#   轮询（accept_nonblock / read_nonblock），把外部发来的 Ruby 代码串在主线程
#   执行，再把结果回传。全程不用 Ruby 线程 —— SketchUp 里线程访问 API 不可靠。
#
# 安装:
#   1) 复制本文件到 Plugins 目录:
#      C:\Users\<user>\AppData\Roaming\SketchUp\SketchUp 2026\SketchUp\Plugins\
#   2) 启动 SketchUp（Plugins 下的顶层 .rb 会自动加载）
#   3) 外部用 su_client.py 连接 127.0.0.1:47123
#
# 协议（每连接一次请求）:
#   请求:  base64(<ruby 代码>) + "\n"
#   响应:  base64(JSON {ok, result|error, backtrace?}) + "\n"
#
# 安全: 只监听 127.0.0.1；不要改成 0.0.0.0。eval 会执行任意 Ruby，仅限本机自用。
# =============================================================================

begin
  require 'sketchup.rb'
rescue LoadError, StandardError
  nil
end

require 'socket'
require 'json'
require 'base64'

module SuBridge
  HOST = '127.0.0.1'
  PORT = 47123
  LOG  = File.join(ENV['TEMP'] || '.', 'su_bridge.log')

  @server  = nil
  @clients = []
  @ticks   = 0
  @started = false

  def self.log(msg)
    File.open(LOG, 'a') { |f| f.puts("#{Time.now.strftime('%H:%M:%S')} #{msg}") }
  rescue StandardError
    nil
  end

  def self.run_one(code)
    begin
      value = eval(code, TOPLEVEL_BINDING, '(su_bridge)', 1)
      { 'ok' => true, 'result' => value.inspect }
    rescue Exception => e
      { 'ok' => false,
        'error' => "#{e.class}: #{e.message}",
        'backtrace' => (e.backtrace || [])[0, 8] }
    end
  end

  def self.close_client(c)
    return if c[:closed]
    c[:closed] = true
    begin
      c[:sock].close
    rescue StandardError
      nil
    end
  end

  def self.accept_new
    loop do
      sock = @server.accept_nonblock(exception: false)
      break if sock == :wait_readable
      break if sock.nil?
      @clients << { sock: sock, buf: (+''), closed: false }
      log("connect (#{@clients.size})")
    end
  rescue StandardError => e
    log("accept error: #{e.class}: #{e.message}")
  end

  def self.service(c)
    return if c[:closed]

    data = c[:sock].read_nonblock(65_536, exception: false)
    if data == :wait_readable
      return
    elsif data.nil?
      close_client(c)
      return
    end
    c[:buf] << data

    idx = c[:buf].index("\n")
    return if idx.nil?

    line = c[:buf].slice!(0..idx)
    code = Base64.decode64(line.strip)
    log("req #{code[0, 80].inspect}")
    res  = run_one(code)
    begin
      c[:sock].write(Base64.strict_encode64(JSON.generate(res)) + "\n")
    rescue StandardError
      nil
    end
    log("resp ok=#{res['ok']}")
    close_client(c)
  rescue StandardError => e
    log("service error: #{e.class}: #{e.message}")
    close_client(c)
  end

  def self.pump
    accept_new
    @clients.each { |c| service(c) }
    @clients.reject! { |c| c[:closed] }
  end

  def self.start
    return if @started
    @started = true

    begin
      @server = TCPServer.new(HOST, PORT)
      begin
        @server.setsockopt(Socket::IPPROTO_TCP, Socket::TCP_NODELAY, 1)
      rescue StandardError
        nil
      end
      log("listening #{HOST}:#{PORT}")
    rescue StandardError => e
      log("bind failed: #{e.class}: #{e.message}")
      return
    end

    UI.start_timer(0.05, true) do
      @ticks += 1
      log("tick #{@ticks}") if @ticks == 1 || (@ticks % 2000).zero?
      begin
        pump
      rescue StandardError => e
        log("pump error: #{e.class}: #{e.message}")
      end
    end

    version = (Sketchup.version rescue 'unknown')
    log("loaded on SketchUp #{version}")
  end
end

SuBridge.start

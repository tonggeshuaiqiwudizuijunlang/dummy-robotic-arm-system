#!/usr/bin/env bash
set -e

echo "检测并开启 BBR 拥塞控制..."

if ! grep -q "net.core.default_qdisc=fq" /etc/sysctl.conf 2>/dev/null; then
  echo "net.core.default_qdisc=fq" | sudo tee -a /etc/sysctl.conf
fi

if ! grep -q "net.ipv4.tcp_congestion_control=bbr" /etc/sysctl.conf 2>/dev/null; then
  echo "net.ipv4.tcp_congestion_control=bbr" | sudo tee -a /etc/sysctl.conf
fi

echo "应用 sysctl 配置..."
sudo sysctl -p

echo "当前拥塞控制算法："
sysctl net.ipv4.tcp_congestion_control

echo "当前 qdisc："
sysctl net.core.default_qdisc

echo "附加低延迟网络参数..."
sudo sysctl -w net.core.rmem_max=2500000
sudo sysctl -w net.core.wmem_max=2500000
sudo sysctl -w net.ipv4.tcp_rmem="4096 87380 2500000"
sudo sysctl -w net.ipv4.tcp_wmem="4096 65536 2500000"

echo "完成。部分设置可能需要重启后完全生效。"


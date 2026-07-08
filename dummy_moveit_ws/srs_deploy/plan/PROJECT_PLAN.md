# 低延迟视频传输项目方案

## 项目概述
- **目标**: 实现 Linux → 云服务器 → Windows 的低延迟视频传输
- **应用场景**: 机械臂远程控制
- **延迟要求**: < 150ms
- **部署环境**: 公网

---

## 1. 技术架构决策

### 1.1 协议选择
| 协议 | 延迟 | 适用场景 | 本项目选择 |
|------|------|----------|-----------|
| WebRTC (SFU) | 50-150ms | 最低延迟，抗丢包 | ⭐ 拉流端使用 |
| SRT | 100-300ms | 弱网对抗强 | ⭐ 推流端使用 |
| RTMP | 1-3s | 兼容性好 | ❌ 延迟过高 |

**决策**: SRT 推流 + WebRTC 拉流

### 1.2 服务器选型
| 方案 | 特点 | 适用场景 |
|------|------|----------|
| SRS | 功能全、中文文档丰富、Docker一键部署 | ⭐ 推荐 |
| ZLMediaKit | 轻量、高性能、低资源占用 | 低配服务器 |
| LiveKit | 现代WebRTC框架、SDK完善 | 需快速开发UI |

**决策**: SRS (Simple Realtime Server)

---

## 2. 系统架构

```
┌─────────────┐      SRT/RTMP       ┌─────────────┐      WebRTC       ┌─────────────┐
│  Linux发送端  │  ═══════════════════► │  云服务器SRS  │ ═══════════════════► │ Windows接收端 │
│  (机械臂端)   │                      │   (中转Relay) │                      │   (控制端)    │
└─────────────┘                      └─────────────┘                      └─────────────┘
      │                                    │                                    │
      │                                    │                                    │
   硬件编码                           协议转换+低缓存                        低延迟解码
   (h264_vaapi)                       (GOP Cache=0)                      (buffer=0)
```

---

## 3. 功能需求 (PRD)

### 3.1 发送端 (Linux)
- [ ] 硬件加速编码 (Intel VA-API)
- [ ] 断线重连机制 (<1秒)
- [ ] 动态码率调整 (1-4 Mbps)
- [ ] 软编码 fallback

### 3.2 云服务器端
- [ ] SRS Docker 部署
- [ ] SRT/RTMP 输入
- [ ] WebRTC/SRT 输出
- [ ] 低缓存配置 (GOP Cache=0)
- [ ] BBR 拥塞控制算法

### 3.3 接收端 (Windows)
- [ ] 低延迟解码 (buffer=0)
- [ ] UI 界面 (PyQt)
- [ ] 实时状态显示:
  - [ ] RTT 延迟
  - [ ] 在线观看人数 (SRS API)
  - [ ] 推流状态
  - [ ] 控制反馈延迟

---

## 4. 部署步骤

### 4.1 服务器端 (云主机)
```bash
# 运行 SRS Docker
docker run --rm -it \
  -p 1935:1935 \
  -p 8080:8080 \
  -p 10080:10080 \
  -p 8000:8000/udp \
  ossrs/srs:5
```

### 4.2 发送端 (Linux PC)
```bash
# FFmpeg SRT 推流
ffmpeg -f v4l2 -i /dev/video0 \
  -c:v h264_vaapi \
  -profile:v baseline \
  -tune zerolatency \
  -b:v 2M \
  -f flv rtmp://<服务器IP>/live/livestream
```

### 4.3 接收端 (Windows)
```bash
# ffplay 低延迟播放
ffplay -fflags nobuffer -flags low_delay rtmp://<服务器IP>/live/livestream
```

---

## 5. 关键配置

### 5.1 服务器优化
- 选择物理距离最近的机房 (ping < 20ms)
- 开启 BBR 拥塞控制
- 禁用 GOP Cache

### 5.2 编码参数
- 预设: `zerolatency`
- 码率: 动态 1-4 Mbps
- Profile: `baseline` (低延迟)

---

## 6. 风险与应对

| 风险 | 影响 | 应对措施 |
|------|------|----------|
| WiFi 切换导致断流 | 高 | 实现 <1s 自动重连 |
| 硬件编码失败 | 中 | 软编码 fallback |
| 带宽不足 | 中 | 动态码率调整 |
| 服务器延迟高 | 高 | 就近选择机房 + BBR |

---

## 7. 里程碑

1. **M1 - 原型验证** (Day 1-2)
   - SRS Docker 跑通
   - FFmpeg 推流测试
   - ffplay 拉流验证

2. **M2 - 功能完善** (Day 3-5)
   - 断线重连
   - 动态码率
   - 硬件编码优化

3. **M3 - UI 集成** (Day 6-7)
   - Windows UI 开发
   - SRS API 集成
   - 状态显示

4. **M4 - 测试优化** (Day 8-10)
   - 压力测试
   - 弱网测试
   - 延迟优化

---

## 8. 参考资源

- SRS 文档: https://ossrs.net/lts/zh-cn/
- SRT 协议: https://github.com/Haivision/srt
- FFmpeg VA-API: https://trac.ffmpeg.org/wiki/Hardware/VAAPI

---

*创建时间: 2026-02-23*
*项目状态: 规划中*

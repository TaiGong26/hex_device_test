# hex_device_test
hex_device 多设备耐久性测试工具

> ⚠️ 仅限内部使用。

---

## 🚀 快速开始

### 1. 配置虚拟环境
``` bash
mkdir ~/hex_test && cd ~/hex_test
uv venv --python 3.11
source ~/hex_test/.venv/bin/activate
```

### 2. 安装
``` bash
git clone https://github.com/TaiGong26/hex_device_test.git
cd hex_device_test
uv pip install -e .
```

### 3. 使用
```bash
python --view --timeout --url ws://xx.xx.xx.xx:8439
```

#### 参数说明
- --view 开始可视化
- --timeout 开启api timeout检查
- --url 必选项，输入一个连接地址

#### 使用须知
- 如果遇到错误情况，会使得所有设备刹车，此时控制台有提示 "[dev x]: running -> brake  reason : xxxx "
- 后续仅支持退出操作：进行一次ctrl c
- 然后等待程序回收退出
- 程序退出过程中不要多次按 ctrl c，因为此时机械臂正在返回home位置。
- 程序退出后，状态会保存在~/hex_device_log中，可以通过column -s, -t ~/hex_device_log/arm_test_xxx.csv | less进行查看。(xxx为时间)

---

## ⚙️ 功能特性

* 多设备协同调度
* 基于进程的执行模式
* 实时状态追踪
* 支持 CSV 日志记录
* 可扩展的控制器架构

---



## 📌 注意事项

* 专为内部耐久性测试设计
* 依赖 hex_device 运行环境
* 运行测试前请确保设备连接正常
* 其他的问题在[Q&A](./docs/Q&A.md)
---

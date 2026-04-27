## 其他问题
### API timeout
- 由于连接有概率API timeout，有以下几个解决方案：
    - 避免使用wifi连接
    - 不使用--timeout参数，以此屏蔽掉当一个设备出现timeout后，其他设备刹车的问题
    - 使用了--timeout参数，但是连接初期的timeout: ctrl c, 并重新运行脚本

---
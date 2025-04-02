# 编译配置命令

**初始化esp-idf环境：**

```bash
cd ~/Work/esp/esp-idf-v5.3.2
. ./export.sh
cd -
```

**配置编译目标为 ESP32S3：**

```bash
idf.py set-target esp32s3
```

**打开 menuconfig：**

```bash
idf.py menuconfig
```

**选择板子：**

```
Xiaozhi Assistant -> Board Type -> Vobot ESP32-S3 公板
```


**编译：**

```bash
idf.py build
```
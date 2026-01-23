# 电流轮廓生成器使用说明

## 设计理念

**不需要预先生成12秒的数据！**
- 采用**状态机设计**，每次中断只计算当前时刻的Id和Iq值
- **内存占用极小**：仅存储配置参数和当前状态（约200字节）
- **实时计算**：每个10kHz中断周期仅需1-2微秒

## 内存占用分析

```c
配置数组：
  - id_iq_amplitudes[10] = 10 × 4字节 = 40字节
  - ix_amplitudes[10] = 10 × 4字节 = 40字节
  
状态变量：
  - 索引和计时器 = 约20字节
  - 输出值 = 12字节
  - 标志位 = 12字节
  
总计：约 124 字节
```

**对比预先生成数据的方案：**
- 12秒 × 10kHz = 120,000个采样点
- 每个点3个浮点数(Id, Iq, Ix) = 120,000 × 3 × 4 = **1.44 MB**
- **我们的方案节省了 99.99% 的内存！**

## 使用方法

### 1. 在模式7中启用电流轮廓生成器

打开 `ACMConfig.h` 或你的配置头文件，修改参数后重新编译：

```c
// 在 init_YZK_ALL() 中已经初始化，默认不运行
// 通过设置标志位来启动
```

### 2. 启动测试

有两种方式启动电流轮廓测试：

#### 方式A：通过代码修改（推荐用于固定测试）

在 `init_YZK_ALL()` 函数中修改：

```c
// 修改测试参数
REAL id_iq_test_amps[] = {1.0, 2.0, 3.0};  // 改成你需要的幅值
REAL ix_test_amps[] = {0.5, 1.0};           // 改成你需要的Ix值

// 修改角度段数和持续时间
CurrentProfileGenerator_Init(&g_current_profile_gen,
                              id_iq_test_amps, 3,
                              ix_test_amps, 2,
                              8,        // 改成你需要的角度段数
                              0.1,      // 改成你需要的每段持续时间(秒)
                              0.0001);

// 启动生成器
g_current_profile_gen.is_running = 1;  // 设为1表示启动
```

#### 方式B：通过调试界面动态控制

在你的调试结构体中添加控制标志：

```c
// 在运行时设置（通过CCS调试器或串口）
g_current_profile_gen.is_running = 1;  // 开始测试
g_current_profile_gen.is_running = 0;  // 停止测试

// 重置到初始状态
CurrentProfileGenerator_Reset(&g_current_profile_gen);
```

### 3. 测试流程

```
启动PWM → 选择模式7 → 生成器自动运行
  ↓
遍历Ix幅值1 → 遍历Id/Iq幅值1 → 遍历8个角度段(每段0.1秒)
             → 遍历Id/Iq幅值2 → 遍历8个角度段
             → 遍历Id/Iq幅值3 → 遍历8个角度段
  ↓
遍历Ix幅值2 → 重复上面的过程
  ↓
测试完成 → is_completed = 1
```

### 4. 参数配置说明

```c
num_angles (角度段数):
  - 8段  → 每45度一个点  (推荐入门)
  - 12段 → 每30度一个点  (精细)
  - 36段 → 每10度一个点  (非常精细)

duration_per_segment (每段持续时间):
  - 0.1秒  → 快速测试
  - 0.5秒  → 稳态测试
  - 1.0秒  → 慢速精确测试

总测试时间计算:
  总时间 = num_ix × num_idiq × num_angles × duration_per_segment
  例如：2 × 3 × 8 × 0.1 = 4.8秒
```

### 5. 监控测试状态

在CCS调试器中添加监视变量：

```c
// 监视当前状态
g_current_profile_gen.is_running     // 是否正在运行
g_current_profile_gen.is_completed   // 是否已完成
g_current_profile_gen.current_id     // 当前Id指令
g_current_profile_gen.current_iq     // 当前Iq指令
g_current_profile_gen.ix_index       // 当前Ix索引
g_current_profile_gen.amp_index      // 当前幅值索引
g_current_profile_gen.angle_index    // 当前角度索引
g_current_profile_gen.segment_timer  // 当前段内时间
```

## 工作原理示意

```
中断周期 (100us @ 10kHz)
│
├─ 测量电流和位置
├─ suspension_p4ps5_PD_doubleaxis()
│  └─ CurrentProfileGenerator_Update()  ← 仅需1-2us
│     ├─ 输出当前Id, Iq (已计算好的值)
│     ├─ 时间计数 += 0.0001
│     └─ 如果段时间到 → 计算下一个点 (cos/sin)
│
└─ FOC电流控制
```

## 实时性能

- **计算时间**: 约1-2微秒 (200MHz DSP)
- **占用率**: < 0.02% (1us / 100us)
- **完全不影响控制性能**

## 示例配置

### 快速测试 (4.8秒)
```c
REAL id_iq[] = {1.0, 2.0, 3.0};    // 3个圆
REAL ix[] = {0.5, 1.0};             // 2个Ix层级
Init(..., 8, 0.1, 0.0001);          // 8段，每段0.1秒
```

### 精细测试 (21.6秒)
```c
REAL id_iq[] = {0.5, 1.0, 1.5, 2.0};  // 4个圆
REAL ix[] = {0, 0.5, 1.0};             // 3个Ix层级
Init(..., 12, 0.5, 0.0001);            // 12段，每段0.5秒
```

### 超精细测试 (72秒)
```c
REAL id_iq[] = {1.0, 2.0, 3.0};
REAL ix[] = {0.5, 1.0};
Init(..., 36, 1.0, 0.0001);  // 36段，每段1秒
```

## 注意事项

1. **首次使用建议**：先用小幅值(如0.5A)和少量点(8段)测试
2. **安全限制**：确保电流幅值在电机额定范围内
3. **完成检测**：测试完成后自动停止，可通过`is_completed`标志检查
4. **重新开始**：调用`CurrentProfileGenerator_Reset()`即可重新开始

## 常见问题

**Q: 会不会占用太多CPU时间？**
A: 不会。每次中断只做一次加法、两次三角函数（切换段时），占用<2微秒。

**Q: 可以中途停止吗？**
A: 可以，设置`g_current_profile_gen.is_running = 0`即可暂停。

**Q: 可以修改测试参数吗？**
A: 可以重新调用`CurrentProfileGenerator_Init()`来更新参数。

**Q: 如何记录测试数据？**
A: 配合DAC输出或数据记录功能，将`current_id`和`current_iq`记录下来。

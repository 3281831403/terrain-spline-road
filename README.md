# terrain-spline-road

## 题面

程序化生成盘山公路后，弯道处路面变窄、接缝一直闪：Catmull-Rom 没有重复端点，首尾切线退化为零；重采样按段数等分而不是按弧长，弯道处采样点变密；切线没有归一化，横截面宽度随速度变化；副法线用切线叉乘 up 后未再次归一，路面发生扭曲；横截面车道数没有校验，会出现负值；路面接缝重复顶点，产生 Z-fighting；接缝处法线没有插值，出现硬边；急弯没有检测，路面自交；批量生成共用顶点缓冲，道路之间数据串扰；`spacing=0` 没有校验，会进入死循环。road/spline.py 是核心，tests/test_spline.py 15个红。要修的是：上述 10 个缺陷全部修复，并保证 15 个测试转绿。

## 运行方式

在仓库根目录执行（Windows PowerShell）：

```powershell
cd D:\gsb-20261119\terrain-spline-road
python -m pytest tests -q
```

初始失败环境应恰好输出：

```text
15 failed
```

15 个失败均为桩函数抛出的 `NotImplementedError`，不应出现 collection、import 或
syntax error。修复 `road/spline.py` 后，同一命令应变为 `15 passed`。

## 字段表

| 字段 | 取值 |
| --- | --- |
| 所属领域 | 游戏、图形与多媒体 |
| 子方向 | 图形渲染 |
| 任务类型 | Bugfix |
| 语言/框架 | Python 3, pytest |
| 环境可复现等级 | 无外部依赖 |

## 约束

- `tests/test_spline.py` 是评测契约，**不可修改**；只能修改 `road/spline.py`。
- 仅使用 Python 标准库（`math`）与 pytest，离线运行，不使用随机数。
- 所有点坐标为 `(x, y, z)`，单位为米；几何比较容差为 `1e-6`。

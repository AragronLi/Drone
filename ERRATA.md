# 开发错误记录

> 只记录调试过程中踩过的坑，不记录常规开发过程。

---

## #1 TIM_OCMode 选错导致 ESC PWM 占空比反转

**日期**：2026-05-26

**现象**：设置 CCR=1060μs，实测输出 94.7% 占空比（18940μs HIGH，1060μs LOW）。

**根因**：照搬官方 `TIM/PWM_Output` 样例的 `PWM2` 模式。官方样例 CCR=ARR/2（50% 占空比），PWM1/PWM2 效果相同看不出差异。ESC 需要窄脉冲时两者行为相反。

**PWM1 vs PWM2（High 极性下）**：

| 模式 | CNT < CCR | CNT >= CCR | 适用场景 |
|------|-----------|------------|---------|
| PWM1 | **HIGH** | LOW | ESC 窄脉冲、舵机控制 |
| PWM2 | LOW | **HIGH** | 常规高占空比输出 |

**修复**：[bsp_esc_pwm.c](../Common/bsp_esc_pwm.c) — `TIM_OCMode_PWM2` → `TIM_OCMode_PWM1`


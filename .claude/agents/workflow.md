# Agent 协作流程

```
用户需求
   │
   ▼
Architect ── 出方案 ──→ 用户确认
   │
   ▼ (分派)
┌──┴──┬──────┬──────┬──────┐
│     │      │      │      │
Core  Algo   OCC    UI    Build
Agent Agent  Agent  Agent  Agent
│     │      │      │      │
└──┬──┴──────┴──────┴──────┘
   │
   ▼ (代码就绪)
Test Agent ── 编写+运行测试
   │
   ▼
Review Agent ── 代码审查
   │
   ▼
Doc Agent ── 更新 architecture.md
   │
   ▼
Build Agent ── 最终编译验证
```

**并行原则**：无依赖的 Agent 尽量并行启动（如 Core + Algorithm + OCC 可同时工作）。

# Qt 界面美化指南

本文档介绍如何为 GrindingApp 应用 Qt 界面美化。

## 📁 文件结构

```
GrindingApp/
├── styles/
│   └── DarkTheme.qss          # 深色主题样式表
├── src/styles/
│   ├── StyleManager.h         # 样式管理器头文件
│   └── StyleManager.cpp       # 样式管理器实现
├── execute/
│   └── main.cpp               # 已集成样式加载
└── docs/
    ├── QT_STYLING.md          # 本文档
    └── web_ui_sample.html     # Web 界面样例
```

## 🎨 主题效果

### 深色主题配色

| 用途 | 颜色代码 | 效果 |
|------|----------|------|
| 背景色 | `#1e1e1e` | 主背景（类似 VS Code） |
| 表面色 | `#2d2d2d` | 卡片、菜单、输入框 |
| 边框色 | `#3e3e3e` | 分隔线、边框 |
| 主色调 | `#0078d4` | 按钮、高亮、选中状态 |
| 成功色 | `#00c853` | 成功状态、确认操作 |
| 警告色 | `#ff9800` | 警告提示 |
| 危险色 | `#f44336` | 错误、急停按钮 |
| 文字色 | `#ffffff` | 主要文字 |

## 🚀 使用方式

### 方式 1：自动应用（已配置）

`main.cpp` 已经配置好自动加载深色主题，程序启动时会自动应用。

### 方式 2：手动加载样式表

```cpp
#include <QFile>
#include <QTextStream>

void loadStyleSheet(const QString& filePath) {
    QFile file(filePath);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream stream(&file);
        QString styleSheet = stream.readAll();
        file.close();
        qApp->setStyleSheet(styleSheet);
    }
}

// 在 main() 中调用
loadStyleSheet("styles/DarkTheme.qss");
```

### 方式 3：使用 StyleManager 类

```cpp
#include "styles/StyleManager.h"

using namespace nl::ui;

// 获取单例并应用主题
StyleManager::instance()->applyTheme(StyleManager::DarkTheme);

// 或者直接调用
StyleManager::instance()->applyDarkTheme();
```

## 🎯 自定义控件样式

### 为控件添加对象名

在 QSS 中，可以使用 `#objectName` 选择器为特定控件设置独特样式：

```cpp
// C++ 代码中设置对象名
QPushButton* emergencyBtn = new QPushButton("急停");
emergencyBtn->setObjectName("emergencyStopButton");

// QSS 中使用
QPushButton#emergencyStopButton {
    background-color: #f44336;
    min-width: 100px;
    min-height: 60px;
    font-size: 18px;
}
```

### 常用对象名约定

| 对象名 | 用途 | 样式效果 |
|--------|------|----------|
| `primaryButton` | 主要操作按钮 | 绿色背景 |
| `dangerButton` | 危险操作按钮 | 红色背景 |
| `emergencyStopButton` | 急停按钮 | 大尺寸红色 |
| `jogButton` | 点动按钮 | 大尺寸方形 |
| `headingLabel` | 标题标签 | 蓝色大字体 |
| `infoLabel` | 信息标签 | 灰色文字 |

## 📝 修改样式

### 修改颜色

编辑 `styles/DarkTheme.qss`，使用查找替换功能批量修改颜色：

```css
/* 将主色调从蓝色改为紫色 */
#0078d4 → #7b1fa2
```

### 添加新控件样式

在 QSS 文件末尾添加：

```css
/* 自定义进度环 */
QCircularProgress {
    background-color: #3e3e3e;
    progress-color: #0078d4;
}
```

### 调整字体大小

```css
QWidget {
    font-size: 14px;  /* 修改此值调整全局字体 */
}
```

## 🔧 编译和运行

### 首次编译

```bash
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

### 运行程序

样式表会在程序启动时自动加载。确保 `styles/DarkTheme.qss` 文件存在于以下位置之一：

1. 编译目录的 `styles/` 文件夹
2. 可执行文件同级目录的 `styles/` 文件夹
3. 项目根目录的 `styles/` 文件夹

## 🌐 Web 界面参考

`docs/web_ui_sample.html` 提供了与 Qt 深色主题一致的 Web 界面样例。

### 在浏览器中查看

直接用浏览器打开：

```
file:///E:/Code/GrindingApp/docs/web_ui_sample.html
```

### Web 与 Qt 架构对比

| 特性 | Qt 桌面端 | Web 前端 |
|------|----------|----------|
| 3D 渲染 | OpenCascade | Three.js |
| 样式系统 | QSS | CSS |
| 布局系统 | QVBoxLayout 等 | Flexbox/Grid |
| 通信方式 | 信号槽 | WebSocket/HTTP |
| 跨平台 | Windows/Linux/macOS | 所有浏览器 |

## 📋 常见问题

### Q: 样式没有生效？

**A:** 检查以下几点：
1. 确认 QSS 文件路径正确
2. 检查控件类型是否与 QSS 选择器匹配
3. 某些控件需要设置 `setAttribute(Qt::WA_StyledWidget)`
4. 自定义控件可能需要调用 `style()->polish(this)`

### Q: 如何恢复默认样式？

**A:** 调用以下代码清除自定义样式：

```cpp
qApp->setStyleSheet("");
```

### Q: 如何添加主题切换功能？

**A:** 
1. 创建 `LightTheme.qss` 浅色主题文件
2. 在 UI 中添加主题切换按钮
3. 使用 `StyleManager::applyTheme()` 切换

### Q: 某些控件样式不一致？

**A:** 某些 Qt 控件（如 QOpenGLWidget）不支持 QSS，需要使用其他方式美化。

## 🔗 相关资源

- [Qt 官方样式表文档](https://doc.qt.io/qt-5/stylesheet.html)
- [Qt 样式表参考](https://doc.qt.io/qt-5/stylesheet-reference.html)
- [Qt 样式表语法](https://doc.qt.io/qt-5/stylesheet-syntax.html)
- [Bootstrap 5 文档](https://getbootstrap.com/docs/5.3/)

## 📦 下一步

1. **运行并测试** - 编译程序查看深色主题效果
2. **调整样式** - 根据喜好修改颜色、字体
3. **添加功能** - 为特定控件（如 JogPanel）添加专属样式
4. **Web 集成** - 参考 `web_ui_sample.html` 设计 Web 界面
5. **API 架构** - 将业务逻辑封装为 REST API，支持多端访问

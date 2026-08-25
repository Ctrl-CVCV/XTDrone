# XeLaTeX 工作空间

这是一个面向中文报告的 XeLaTeX 项目模板。主文件是 `main.tex`，章节位于
`sections`，图片放在 `figures`，编译产物输出到 `build`。

## 编译

推荐安装 TeX Live 或 MiKTeX，并使用 `latexmk`：

```powershell
latexmk main.tex
```

`.latexmkrc` 已将默认引擎设置为 XeLaTeX。也可以显式执行：

```powershell
latexmk -xelatex main.tex
```

清理编译产物：

```powershell
latexmk -C
```

在 VS Code 中安装 LaTeX Workshop 扩展后，保存 `main.tex` 即会自动使用
XeLaTeX 编译。

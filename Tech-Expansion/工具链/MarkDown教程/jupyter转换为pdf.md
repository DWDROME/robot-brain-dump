---

==Ubuntu22.04==

### 1. 安装 LaTeX 和中文支持组件

#### 安装 LaTeX 完整版和中文组件
```bash
sudo apt-get install texlive-full texlive-xetex texlive-lang-chinese texlive-fonts-recommended
```

#### 如果安装 LaTeX 太慢,或者进程被卡住（用top查看），可以尝试以下方法：
1. **清理锁定和缓存文件**：
   
   ```bash
   sudo rm -f /var/lib/dpkg/lock-frontend
   sudo rm -f /var/lib/dpkg/lock
   sudo apt-get clean
   ```
2. **使用替代镜像源**（例如更换为清华源）：
   打开 `/etc/apt/sources.list` 文件：
   ```bash
   sudo nano /etc/apt/sources.list
   ```
   替换或添加以下镜像源：
   ```
   deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
   deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
   ```
   更新缓存后重新安装：
   ```bash
   sudo apt-get update
   sudo apt-get install texlive-full texlive-xetex texlive-lang-chinese texlive-fonts-recommended
   ```

#### 如果仍然无法安装成功，可以重新安装：
```bash
sudo apt-get purge texlive-full
sudo apt-get autoremove
sudo apt-get install texlive-full
```

---

### 2. 安装 Jupyter Notebook

#### 安装 Jupyter Notebook
```bash
pip install notebook
```

#### 安装 `nbconvert`（用于转换文件为 PDF）
```bash
pip install nbconvert
```

---

### 3. 创建目录并存放 PDF

#### 在当前目录创建文件夹
```bash
mkdir pdf_output
```

#### 批量将 Jupyter Notebook 文件转换为 PDF
在目标文件夹中运行以下命令：
```bash
for file in *.ipynb; do
    jupyter nbconvert --to pdf "$file" --output-dir=./pdf_output
done
```

---

### 4. 在 Jupyter Notebook 中安装中文支持模板
为确保 PDF 导出时支持中文，安装 `latex` 模板：

#### 安装 `latex` 和模板
```bash
sudo apt-get install latex-cjk-all
```

---

### 5. jpynb->pdf

#### 命令运行流程（分块编写）

1. **创建 PDF 存放目录**：
   ```bash
   mkdir pdf_output
   ```

2. **转换 `.ipynb` 文件为 PDF**：
   ```bash
   jupyter nbconvert --to pdf my_notebook.ipynb --output-dir=./pdf_output
   ```

3. **批量转换所有 `.ipynb` 文件**：
   ```bash
   for file in *.ipynb; do
       jupyter nbconvert --to pdf "$file" --output-dir=./pdf_output
   done
   ```

---

### 补充说明
- 如果需要支持中文的 LaTeX 模板，确保导出时选择 `PDF via LaTeX`。
- 如果需要进一步调整 LaTeX 文档，可以将 `.ipynb` 转换为 `.tex`，然后手动编辑并生成 PDF。

---

如果有其他问题或需要更详细的步骤，请随时告诉我！ 😊
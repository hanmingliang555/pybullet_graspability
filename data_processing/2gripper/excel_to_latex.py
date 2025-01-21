import pandas as pd

# 读取 Excel 文件
df = pd.read_excel('filter_obj_json2025.1.13/output/increase_mass/all.xlsx')
output_path="filter_obj_json2025.1.13/output/increase_mass/increase_mass.tex"
# 将 DataFrame 转换为 LaTeX 格式
latex_code = df.to_latex(index=False)  # 设置 index=False 不输出行号

# 输出 LaTeX 代码到 .tex 文件
with open(output_path, 'w') as f:
    f.write(latex_code)

print("LaTeX 表格已输出")

import pandas as pd

# 读取 Excel 文件
df = pd.read_excel('3gripper_data/output/decrease_force/decrease_force.xlsx')
# 输出 LaTeX 代码到 .tex 文件
output_path = "3gripper_data/output/decrease_force/decrease_force.tex"

# 数字到物体名称的映射字典
name_mapping = {
    0: "cracker box",
    1: "tomato soup can",  # 使用大括号包裹含星号的文本
    2: "mustard bottle",
    3: "potted meat can",
    4: "banana",
    5: "bowl",
    6: "chips can",
    7: "strawberry",
    8: "orange",
    9: "knife",
    10: "racquetball",
    11: "toy airplane",
    12: "wash soup",
    13: "dabao sod",
    14: "baoke marker", 
    15: "camel",
    16: "large elephant",
    17: "darlie box",
    18: "mouse",
    19: "shampoo",
    # 可以继续添加其他数字和名称的映射
}


# 使用映射字典替换 'Object Name' 列中的数字为名称
#df['Object Name'] = df['Object Name'].replace(name_mapping)
# 将 DataFrame 中的所有数值列四舍五入为四位有效数字
df = df.applymap(lambda x: f"{x:.4g}" if isinstance(x, (int, float)) else x)
# 将 DataFrame 转换为 LaTeX 格式
latex_code = df.to_latex(index=False)  # 设置 index=False 不输出行号
with open(output_path, 'w') as f:
    f.write(latex_code)
print("LaTeX 表格已输出")


import pandas as pd

# 读取四个 Excel 文件
file_paths = [
    'filter_obj_json2025.1.13/output/random_perturbation/origin_1.xlsx',
    'filter_obj_json2025.1.13/output/random_perturbation/FC_1.xlsx',
    'filter_obj_json2025.1.13/output/random_perturbation/FR_1.xlsx',
    'filter_obj_json2025.1.13/output/random_perturbation/PAG_1.xlsx',
]
output_file='filter_obj_json2025.1.13/output/random_perturbation/all_1.xlsx'

# 创建一个空的 DataFrame 来存储合并后的数据
merged_df = pd.DataFrame()

# 遍历所有文件
for file_path in file_paths:
    df = pd.read_excel(file_path)

    # 将每个文件的 'Object Name' 列设置为索引，这样可以方便地进行合并
    df.set_index('Object Name', inplace=True)

    # 如果 merged_df 为空，直接将第一个文件的内容复制到 merged_df 中
    if merged_df.empty:
        merged_df = df
    else:
        # 根据 'Object Name' 合并
        merged_df = merged_df.join(df, how='outer')
merged_df.sort_index(inplace=True)
# 将合并后的数据保存为新的 Excel 文件
merged_df.to_excel(output_file, index=True)


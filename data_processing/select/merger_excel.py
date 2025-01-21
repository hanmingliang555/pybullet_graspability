import pandas as pd

# 读取四个表格
file1 = pd.read_excel('3gripper_data/output/random_perturbation/se_origin.xlsx')  
file2 = pd.read_excel('3gripper_data/output/random_perturbation/se_FC.xlsx')  
file3 = pd.read_excel('3gripper_data/output/random_perturbation/se_FR.xlsx')  
file4 = pd.read_excel('3gripper_data/output/random_perturbation/se_PAG.xlsx')  

# 合并四个DataFrame，按 Object Name 列进行合并
merged_df = pd.merge(file1, file2, on="Object Name", how="inner")
merged_df = pd.merge(merged_df, file3, on="Object Name", how="inner")
merged_df = pd.merge(merged_df, file4, on="Object Name", how="inner")
merged_df.sort_index(inplace=True)
# 保存合并后的结果
merged_df.to_excel('data_processing/select/3gripper_r.xlsx', index=False)


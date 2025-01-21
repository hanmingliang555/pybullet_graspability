import os
import pandas as pd
import re
import numpy as np

data_base="alldata/output/increase_mass/change_obj"
data_FC_base=os.path.join(data_base,"FC")
data_FR_base=os.path.join(data_base,"FR")
data_PAG_base=os.path.join(data_base,"PAG")
friction=0.8
distance=0.05
angle=10

output_file_1 = os.path.join(data_base,'se_FC.xlsx')
output_file_2 = os.path.join(data_base,'se_FR.xlsx')
output_file_3 = os.path.join(data_base,'se_PAG.xlsx')
excel_output = os.path.join(data_base,"increase_xlsx")

folder_names_1 = []
mean_forces_1 = []
mean_angles_1 = []
mean_move_distances_1 = []
folder_names_2 = []
mean_forces_2 = []
mean_angles_2 = []
mean_move_distances_2 = []
folder_names_3 = []
mean_forces_3 = []
mean_angles_3 = []
mean_move_distances_3 = []


for folder in os.listdir(data_FC_base):
    folder_path = os.path.join(data_FC_base, folder)
    folder_key = f"{int(folder):04d}"
    # 确保是文件夹
    if os.path.isdir(folder_path):
        for sub_folder in os.listdir(folder_path):
            sub_folder_path=os.path.join(folder_path,sub_folder)
            max_force = []
            angles=[]
            move_distances=[]
            # 遍历文件夹中的所有 CSV 文件
            for file in os.listdir(sub_folder_path):
                if "increase_mass" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(sub_folder_path, file)
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    i=len(df)
                    while(i>0):                    
                        # 获取最后一行值
                        last_row = df.iloc[i-1]
                        move_distance = last_row[5]
                        angle_value = last_row[4]
                        if move_distance<distance and angle_value<angle:
                            angles.append(angle_value)
                            move_distances.append(move_distance)
                            mass_value = last_row[3]
                            mean_forces_1.append(mass_value)
                            folder_names_1.append(f"{folder_key}_{sub_folder}")
                            break
                        else :
                            i-=1
                    if i == 0:
                        mean_forces_1.append(1)
                        angles.append(angle)
                        move_distances.append(distance)
                        folder_names_1.append(f"{folder_key}_{sub_folder}")    

# 将文件夹名称和均值写入 Excel 文件
df_result_1 = pd.DataFrame({
            'Object Name': folder_names_1,
            'Mean Mass': mean_forces_1,
            #'Mean Angle': mean_angles,
            #'Mean Move Distance': mean_move_distances
        })
df_result_1.to_excel(output_file_1, index=False)

for folder in os.listdir(data_FR_base):
    folder_path = os.path.join(data_FR_base, folder)
    folder_key = f"{int(folder):04d}"
    # 确保是文件夹
    if os.path.isdir(folder_path):
        for sub_folder in os.listdir(folder_path):
            sub_folder_path=os.path.join(folder_path,sub_folder)
            max_force = []
            angles=[]
            move_distances=[]
            # 遍历文件夹中的所有 CSV 文件
            for file in os.listdir(sub_folder_path):
                if "increase_mass" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(sub_folder_path, file)
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    i=len(df)
                    while(i>0):                    
                        # 获取最后一行值
                        last_row = df.iloc[i-1]
                        move_distance = last_row[5]
                        angle_value = last_row[4]
                        if move_distance<distance and angle_value<angle:
                            angles.append(angle_value)
                            move_distances.append(move_distance)
                            mass_value = last_row[3]
                            mean_forces_2.append(mass_value)
                            folder_names_2.append(f"{folder_key}_{sub_folder}")
                            break
                        else :
                            i-=1
                    if i == 0:
                        mean_forces_2.append(1)
                        angles.append(angle)
                        move_distances.append(distance)
                        folder_names_2.append(f"{folder_key}_{sub_folder}")    

# 将文件夹名称和均值写入 Excel 文件
df_result_2 = pd.DataFrame({
            'Object Name': folder_names_2,
            'Mean Mass': mean_forces_2,
            #'Mean Angle': mean_angles,
            #'Mean Move Distance': mean_move_distances
        })
df_result_2.to_excel(output_file_2, index=False)


for folder in os.listdir(data_PAG_base):
    folder_path = os.path.join(data_PAG_base, folder)
    folder_key = f"{int(folder):04d}"
    # 确保是文件夹
    if os.path.isdir(folder_path):
        for sub_folder in os.listdir(folder_path):
            sub_folder_path=os.path.join(folder_path,sub_folder)
            max_force = []
            angles=[]
            move_distances=[]
            # 遍历文件夹中的所有 CSV 文件
            for file in os.listdir(sub_folder_path):
                if "increase_mass" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(sub_folder_path, file)
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    i=len(df)
                    while(i>0):                    
                        # 获取最后一行值
                        last_row = df.iloc[i-1]
                        move_distance = last_row[5]
                        angle_value = last_row[4]
                        if move_distance<distance and angle_value<angle:
                            angles.append(angle_value)
                            move_distances.append(move_distance)
                            mass_value = last_row[3]
                            mean_forces_3.append(mass_value)
                            folder_names_3.append(f"{folder_key}_{sub_folder}")
                            break
                        else :
                            i-=1
                    if i == 0:
                        mean_forces_3.append(1)
                        angles.append(angle)
                        move_distances.append(distance)
                        folder_names_3.append(f"{folder_key}_{sub_folder}")    

# 将文件夹名称和均值写入 Excel 文件
df_result_3 = pd.DataFrame({
            'Object Name': folder_names_3,
            'Mean Mass': mean_forces_3,
            #'Mean Angle': mean_angles,
            #'Mean Move Distance': mean_move_distances
        })
df_result_3.to_excel(output_file_3, index=False)

# 读取四个表格
file1 = pd.read_excel(output_file_1)  
file2 = pd.read_excel(output_file_2)  
file3 = pd.read_excel(output_file_3)  
 

# 合并四个DataFrame，按 Object Name 列进行合并
merged_df = pd.merge(file1, file2, on="Object Name", how="inner")
merged_df = pd.merge(merged_df, file3, on="Object Name", how="inner")

merged_df.sort_index(inplace=True)
# 保存合并后的结果
merged_df.to_excel(excel_output, index=False)
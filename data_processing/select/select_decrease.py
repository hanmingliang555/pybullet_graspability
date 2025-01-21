import os
import pandas as pd
import re
import numpy as np

data_base="alldata/output/decrease_force/change_obj"
data_FC_base=os.path.join(data_base,"FC")
data_FR_base=os.path.join(data_base,"FR")
data_PAG_base=os.path.join(data_base,"PAG")
friction=0.8
distance=0.05
angle=10

output_file_1 = os.path.join(data_base,'se_FC.xlsx')
output_file_2 = os.path.join(data_base,'se_FR.xlsx')
output_file_3 = os.path.join(data_base,'se_PAG.xlsx')


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
                if "decrease_force" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(sub_folder_path, file)
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    i=len(df)
                    while(i>0):
                        # 获取最后一行值
                        last_row = df.iloc[-1]
                        angle_value = last_row[4]
                        move_distance = last_row[5]
                        if move_distance<distance and angle_value<angle:
                            angles.append(angle_value)
                            move_distances.append(move_distance)
                            # 提取 "left" 和 "right" 的值
                            force_value = last_row[2]
                            match = re.match(r"left:(-?\d+(\.\d+)?),right:(-?\d+(\.\d+)?),middle:(-?\d+(\.\d+)?)", str(force_value))
                            if match:
                                left_value = float(match.group(1))
                                right_value = float(match.group(3))
                                middle_value = float(match.group(5))    
                                folder_names_1.append(f"{folder_key}_{sub_folder}")
                                mean_forces_1.append(max(left_value, right_value,middle_value))
                                break
                        else :
                            i-=1
                    if i == 0:
                        mean_forces_1.append(50)
                        angles.append(angle)
                        move_distances.append(distance)
                        folder_names_1.append(f"{folder_key}_{sub_folder}")  


# 将文件夹名称和均值写入 Excel 文件
df_result_1 = pd.DataFrame({
            'Object Name': folder_names_1,
            'Mean Force': mean_forces_1,
            #'Mean Angle': mean_angles,
            #'Mean Move Distance': mean_move_distances
        })
# 保留有效数字

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
                if "decrease_force" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(sub_folder_path, file)
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    i=len(df)
                    while(i>0):
                        # 获取最后一行值
                        last_row = df.iloc[-1]
                        angle_value = last_row[4]
                        move_distance = last_row[5]
                        if move_distance<distance and angle_value<angle:
                            angles.append(angle_value)
                            move_distances.append(move_distance)
                            # 提取 "left" 和 "right" 的值
                            force_value = last_row[2]
                            match = re.match(r"left:(-?\d+(\.\d+)?),right:(-?\d+(\.\d+)?),middle:(-?\d+(\.\d+)?)", str(force_value))
                            if match:
                                left_value = float(match.group(1))
                                right_value = float(match.group(3))
                                middle_value = float(match.group(5))    
                                folder_names_2.append(f"{folder_key}_{sub_folder}")
                                mean_forces_2.append(max(left_value, right_value,middle_value))
                                break
                        else :
                            i-=1
                    if i == 0:
                        mean_forces_2.append(50)
                        angles.append(angle)
                        move_distances.append(distance)
                        folder_names_2.append(f"{folder_key}_{sub_folder}")  


# 将文件夹名称和均值写入 Excel 文件
df_result_2 = pd.DataFrame({
            'Object Name': folder_names_2,
            'Mean Force': mean_forces_2,
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
                if "decrease_force" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(sub_folder_path, file)
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    i=len(df)
                    while(i>0):
                        # 获取最后一行值
                        last_row = df.iloc[-1]
                        angle_value = last_row[4]
                        move_distance = last_row[5]
                        if move_distance<distance and angle_value<angle:
                            angles.append(angle_value)
                            move_distances.append(move_distance)
                            # 提取 "left" 和 "right" 的值
                            force_value = last_row[2]
                            match = re.match(r"left:(-?\d+(\.\d+)?),right:(-?\d+(\.\d+)?),middle:(-?\d+(\.\d+)?)", str(force_value))
                            if match:
                                left_value = float(match.group(1))
                                right_value = float(match.group(3))
                                middle_value = float(match.group(5))    
                                folder_names_3.append(f"{folder_key}_{sub_folder}")
                                mean_forces_3.append(max(left_value, right_value,middle_value))
                                break
                        else :
                            i-=1
                    if i == 0:
                        mean_forces_3.append(50)
                        angles.append(angle)
                        move_distances.append(distance)
                        folder_names_3.append(f"{folder_key}_{sub_folder}")  


# 将文件夹名称和均值写入 Excel 文件
df_result_3 = pd.DataFrame({
            'Object Name': folder_names_3,
            'Mean Force': mean_forces_3,
            #'Mean Angle': mean_angles,
            #'Mean Move Distance': mean_move_distances
        })
# 保留有效数字

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
merged_df.to_excel(data_base, index=False)
import os
import pandas as pd
import re
import numpy as np
# 输入文件夹路径和输出 Excel 文件路径
base_dir = '3gripper_data/output'  # 替换为 decrease_force 的路径
json_dir = 'random_perturbation'
input_dir=os.path.join(base_dir,json_dir)
friction=0.8
distance=0.05
angle=100
# 用于存储每个文件夹和均值的数据
folder_names = []
mean_forces = []
mean_angles = []
mean_move_distances = []
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
origin_obj_output=os.path.join(input_dir, 'obj')
change_obj_output_1=os.path.join(input_dir,'change_obj/FC')
change_obj_output_2=os.path.join(input_dir,'change_obj/FR')
change_obj_output_3=os.path.join(input_dir,'change_obj/PAG')
output_file_1 = os.path.join(base_dir,json_dir,'se_origin.xlsx')
output_file_2 = os.path.join(base_dir,json_dir,'se_FC.xlsx')
output_file_3 = os.path.join(base_dir,json_dir,'se_FR.xlsx')
output_file_4 = os.path.join(base_dir,json_dir,'se_PAG.xlsx')

for folder in os.listdir(origin_obj_output):
    folder_path = os.path.join(origin_obj_output, folder)
    # 确保是文件夹
    if os.path.isdir(folder_path):
        values = []
        max_force = []
        angles=[]
        move_distances=[]
        # 遍历文件夹中的所有 CSV 文件
        for file in os.listdir(folder_path):
            if "random_perturbation" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                file_path = os.path.join(folder_path, file)
                file_key=file.split("_")[1]
                if os.path.exists(file_path):
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    i=len(df)
                    while(i>0):  
                        # 获取最后一行值
                        last_row = df.iloc[i-1]
                        angle_value = last_row[5]
                        move_distance = last_row[6]
                        if move_distance<distance and angle_value<angle:
                            angles.append(angle_value)
                            move_distances.append(move_distance)
                            mass_value = last_row[3]
                            mean_forces.append(mass_value)
                            folder_names.append(f"{folder}_{file_key}")
                            break
                        else :
                            i-=1
                    if i == 0:
                        mean_forces.append(1)
                        angles.append(angle)
                        move_distances.append(distance)
                        folder_names.append(f"{folder}_{file_key}")   


# 将文件夹名称和均值写入 Excel 文件
df_result = pd.DataFrame({
            'Object Name': folder_names,
            'Mean random_perturbation': mean_forces,
            #'Mean Angle': mean_angles,
            #'Mean Move Distance': mean_move_distances
        })
df_result.to_excel(output_file_1, index=False)



for folder in os.listdir(change_obj_output_1):
    #if folder=="0000":
        folder_path = os.path.join(change_obj_output_1, folder)
        standard_path = os.path.join(origin_obj_output, folder)
        # 确保是文件夹
        if os.path.isdir(folder_path):
            max_force = []
            angles=[]
            move_distances=[]
            # 遍历文件夹中的所有 CSV 文件
            for file in os.listdir(folder_path):
                if "random_perturbation" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(folder_path, file)
                    standard_path_1 = os.path.join(standard_path,file)
                    file_key=file.split("_")[1]
                    if os.path.exists(standard_path_1) and os.path.exists(file_path):
                        # 读取 CSV 文件
                        df = pd.read_csv(file_path)
                        standard_csv = pd.read_csv(standard_path_1)
                        j=len(standard_csv)
                        while(j>0):
                            standard_data = standard_csv.iloc[j-1]
                            standard_move = standard_data[6]/friction
                            standard_angle = standard_data[5]/friction
                            if standard_data[6]<distance and standard_data[5]<angle:
                                i=len(df)
                                while(i>0):
                                        # 获取最后一行值
                                        last_row = df.iloc[i-1]
                                        move_distance = last_row[6]
                                        angle_value = last_row[5]
                                        #if move_distance<standard_move and angle_value<standard_angle:
                                        if move_distance<distance and angle_value<angle:
                                            angles.append(angle_value)
                                            move_distances.append(move_distance)

                                            mass_value = last_row[3]
                                            mean_forces_1.append(mass_value)
                                            folder_names_1.append(f"{folder}_{file_key}")
                                            break
                                        else :
                                            i-=1
                                if i == 0:
                                    mean_forces_1.append(1)
                                    angles.append(standard_angle)
                                    move_distances.append(standard_move)
                                    folder_names_1.append(f"{folder}_{file_key}")
                                break
                            else:
                                j-=1
                        

# 将文件夹名称和均值写入 Excel 文件
df_result_1 = pd.DataFrame({
            'Object Name': folder_names_1,
            'FC_Mean random_perturbation': mean_forces_1,
            #'FC_Mean Angle': mean_angles_1,
            #'FC_Mean Move Distance': mean_move_distances_1
        })

df_result_1.to_excel(output_file_2,index=False)

for folder in os.listdir(change_obj_output_2):
    #if folder=="0000":
        folder_path = os.path.join(change_obj_output_2, folder)
        standard_path = os.path.join(origin_obj_output, folder)
        # 确保是文件夹
        if os.path.isdir(folder_path):
            max_force = []
            angles=[]
            move_distances=[]
            # 遍历文件夹中的所有 CSV 文件
            for file in os.listdir(folder_path):
                if "random_perturbation" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(folder_path, file)
                    standard_path_2 = os.path.join(standard_path,file)
                    file_key=file.split("_")[1]
                    if os.path.exists(standard_path_2) and os.path.exists(file_path):
                        # 读取 CSV 文件
                        df = pd.read_csv(file_path)
                        standard_csv = pd.read_csv(standard_path_2)
                        j=len(standard_csv)
                        while(j>0):
                            standard_data = standard_csv.iloc[j-1]
                            standard_move = standard_data[6]/friction
                            standard_angle = standard_data[5]/friction
                            if standard_data[6]<distance and standard_data[5]<angle:
                                i=len(df)
                                while(i>0):
                                        # 获取最后一行值
                                        last_row = df.iloc[i-1]
                                        move_distance = last_row[6]
                                        angle_value = last_row[5]
                                        #if move_distance<standard_move and angle_value<standard_angle:
                                        if move_distance<distance and angle_value<angle:
                                            angles.append(angle_value)
                                            move_distances.append(move_distance)
                                            # 提取 "left" 和 "right" 的值
                                            mass_value = last_row[3]
                                            mean_forces_2.append(mass_value)
                                            folder_names_2.append(f"{folder}_{file_key}")
                                            break
                                        else :
                                            i-=1
                                if i == 0:
                                    mean_forces_2.append(1)
                                    angles.append(standard_angle)
                                    move_distances.append(standard_move)
                                    folder_names_2.append(f"{folder}_{file_key}")
                                break
                            else:
                                j-=1
                        


# 将文件夹名称和均值写入 Excel 文件
df_result_2 = pd.DataFrame({
            'Object Name': folder_names_2,
            'FR_Mean random_perturbation': mean_forces_2,
            #'FR_Mean Angle': mean_angles_2,
            #'FR_Mean Move Distance': mean_move_distances_2
        })
df_result_2.to_excel(output_file_3, index=False)

for folder in os.listdir(change_obj_output_3):
    #if folder=="0000":
        folder_path = os.path.join(change_obj_output_3, folder)
        standard_path = os.path.join(origin_obj_output, folder)
        # 确保是文件夹
        if os.path.isdir(folder_path):
            max_force = []
            angles=[]
            move_distances=[]
            # 遍历文件夹中的所有 CSV 文件
            for file in os.listdir(folder_path):
                if "random_perturbation" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(folder_path, file)
                    standard_path_3 = os.path.join(standard_path,file)
                    file_key=file.split("_")[1]
                    if os.path.exists(standard_path_3) and os.path.exists(file_path):
                        # 读取 CSV 文件
                        df = pd.read_csv(file_path)
                        standard_csv = pd.read_csv(standard_path_3)
                        j=len(standard_csv)
                        while(j>0):
                            standard_data = standard_csv.iloc[j-1]
                            standard_move = standard_data[6]/friction
                            standard_angle = standard_data[5]/friction
                            if standard_data[6]<distance and standard_data[5]<angle:
                                i=len(df)
                                while(i>0):
                                        # 获取最后一行值
                                        last_row = df.iloc[i-1]
                                        move_distance = last_row[6]
                                        angle_value = last_row[5]
                                        #if move_distance<standard_move and angle_value<standard_angle:
                                        if move_distance<distance and angle_value<angle:
                                            angles.append(angle_value)
                                            move_distances.append(move_distance)

                                            mass_value = last_row[3]
                                            mean_forces_3.append(mass_value)
                                            folder_names_3.append(f"{folder}_{file_key}")
                                            break
                                        else :
                                            i-=1
                                if i == 0:
                                    mean_forces_3.append(1)
                                    angles.append(standard_angle)
                                    move_distances.append(standard_move)
                                    folder_names_3.append(f"{folder}_{file_key}")
                                break
                            else:
                                j-=1
                        



# 将文件夹名称和均值写入 Excel 文件
df_result_3 = pd.DataFrame({
            'Object Name': folder_names_3,
            'PAG_Mean random_perturbation': mean_forces_3,
            #'PAG_Mean Angle': mean_angles_3,
            #'PAG_Mean Move Distance': mean_move_distances_3
        })
df_result_3.to_excel(output_file_4, index=False)

# 读取四个表格
file1 = pd.read_excel(output_file_1)  
file2 = pd.read_excel(output_file_2)  
file3 = pd.read_excel(output_file_3)  
file4 = pd.read_excel(output_file_4)  

# 合并四个DataFrame，按 Object Name 列进行合并
merged_df = pd.merge(file1, file2, on="Object Name", how="inner")
merged_df = pd.merge(merged_df, file3, on="Object Name", how="inner")
merged_df = pd.merge(merged_df, file4, on="Object Name", how="inner")
merged_df.sort_index(inplace=True)
# 保存合并后的结果
merged_df.to_excel('data_processing/select/3gripper_r.xlsx', index=False)


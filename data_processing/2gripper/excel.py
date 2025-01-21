import os
import pandas as pd
import re
# 输入文件夹路径和输出 Excel 文件路径
base_dir = 'filter_obj_json2025.1.13/output'  # 替换为 decrease_force 的路径
json_dir = 'random_perturbation'
input_dir=os.path.join(base_dir,json_dir)

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
output_file_1 = os.path.join(base_dir,json_dir,'origin.xlsx')
output_file_2 = os.path.join(base_dir,json_dir,'FC.xlsx')
output_file_3 = os.path.join(base_dir,json_dir,'FR.xlsx')
output_file_4 = os.path.join(base_dir,json_dir,'PAG.xlsx')

for folder in os.listdir(origin_obj_output):
    folder_path = os.path.join(origin_obj_output, folder)
    # 确保是文件夹
    if os.path.isdir(folder_path):
        values = []
        max_force = []
        # 遍历文件夹中的所有 CSV 文件
        for file in os.listdir(folder_path):
            if "random_perturbation" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                angles=[]
                move_distances=[]
                file_path = os.path.join(folder_path, file)
                try:
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    
                    # 获取最后一行值
                    last_row = df.iloc[-1]
                    angle_value = last_row[4]
                    move_distance = last_row[5]
                    angles.append(angle_value)
                    move_distances.append(move_distance)
                    # 提取 "left" 和 "right" 的值
                    force_value = last_row[2]
                    match = re.match(r"left:(-?\d+(\.\d+)?),right:(-?\d+(\.\d+)?)", str(force_value))
                    if match:
                        left_value = float(match.group(1))
                        right_value = float(match.group(3))
                        
                        # 取最大值
                        max_force.append(min(left_value, right_value))
                except Exception as e:
                    print(f"无法读取文件 {file_path}: {e}")
        # 计算均值
        if max_force:
            mean_force = sum(max_force) / len(max_force)
            mean_angle = sum(angles) / len(angles)
            mean_move_distance = sum(move_distances) / len(move_distances)
            folder_names.append(folder)  # 保存文件夹名称
            mean_forces.append(mean_force)  # 保存均值
            mean_angles.append(mean_angle)
            mean_move_distances.append(mean_move_distance)
        else:
            folder_names.append(folder)
            mean_forces.append(None)
            mean_angles.append(None)
            mean_move_distances.append(None)

# 将文件夹名称和均值写入 Excel 文件
df_result = pd.DataFrame({
            'Object Name': folder_names,
            'Mean Force': mean_forces,
            'Mean Angle': mean_angles,
            'Mean Move Distance': mean_move_distances
        })
df_result.to_excel(output_file_1, index=False)



for folder in os.listdir(change_obj_output_1):
    folder_path = os.path.join(change_obj_output_1, folder)
    # 确保是文件夹
    if os.path.isdir(folder_path):
        max_force = []
        angles=[]
        move_distances=[]
        # 遍历文件夹中的所有 CSV 文件
        for file in os.listdir(folder_path):
            if "random_perturbation" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                file_path = os.path.join(folder_path, file)
                try:
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    
                    # 获取最后一行值
                    last_row = df.iloc[-1]
                    angle_value = last_row[4]
                    move_distance = last_row[5]
                    angles.append(angle_value)
                    move_distances.append(move_distance)
                    # 提取 "left" 和 "right" 的值
                    force_value = last_row[2]
                    match = re.match(r"left:(-?\d+(\.\d+)?),right:(-?\d+(\.\d+)?)", str(force_value))
                    if match:
                        left_value = float(match.group(1))
                        right_value = float(match.group(3))
                        
                        # 取最大值
                        max_force.append(min(left_value, right_value))
                except Exception as e:
                    print(f"无法读取文件 {file_path}: {e}")
        # 计算均值
        if max_force:
            mean_force = sum(max_force) / len(max_force)
            mean_angle = sum(angles) / len(angles)
            mean_move_distance = sum(move_distances) / len(move_distances)
            folder_names_1.append(folder)  # 保存文件夹名称
            mean_forces_1.append(mean_force)  # 保存均值
            mean_angles_1.append(mean_angle)
            mean_move_distances_1.append(mean_move_distance)
        else:
            folder_names_1.append(folder)
            mean_forces_1.append(None)
            mean_angles_1.append(None)
            mean_move_distances_1.append(None)

# 将文件夹名称和均值写入 Excel 文件
df_result_1 = pd.DataFrame({
            'Object Name': folder_names_1,
            'FC_Mean Force': mean_forces_1,
            'FC_Mean Angle': mean_angles_1,
            'FC_Mean Move Distance': mean_move_distances_1
        })
df_result_1.to_excel(output_file_2, index=False)

for folder in os.listdir(change_obj_output_2):
    folder_path = os.path.join(change_obj_output_2, folder)
    # 确保是文件夹
    if os.path.isdir(folder_path):
        max_force = []
        angles=[]
        move_distances=[]
        # 遍历文件夹中的所有 CSV 文件
        for file in os.listdir(folder_path):
            if "random_perturbation" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                file_path = os.path.join(folder_path, file)
                try:
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    
                    # 获取最后一行值
                    last_row = df.iloc[-1]
                    angle_value = last_row[4]
                    move_distance = last_row[5]
                    angles.append(angle_value)
                    move_distances.append(move_distance)
                    # 提取 "left" 和 "right" 的值
                    force_value = last_row[2]
                    match = re.match(r"left:(-?\d+(\.\d+)?),right:(-?\d+(\.\d+)?)", str(force_value))
                    if match:
                        left_value = float(match.group(1))
                        right_value = float(match.group(3))
                        
                        # 取最大值
                        max_force.append(min(left_value, right_value))
                except Exception as e:
                    print(f"无法读取文件 {file_path}: {e}")
        # 计算均值
        if max_force:
            mean_force = sum(max_force) / len(max_force)
            mean_angle = sum(angles) / len(angles)
            mean_move_distance = sum(move_distances) / len(move_distances)
            folder_names_2.append(folder)  # 保存文件夹名称
            mean_forces_2.append(mean_force)  # 保存均值
            mean_angles_2.append(mean_angle)
            mean_move_distances_2.append(mean_move_distance)
        else:
            folder_names_2.append(folder)
            mean_forces_2.append(None)
            mean_angles_2.append(None)
            mean_move_distances_2.append(None)

# 将文件夹名称和均值写入 Excel 文件
df_result_2 = pd.DataFrame({
            'Object Name': folder_names_2,
            'FR_Mean Force': mean_forces_2,
            'FR_Mean Angle': mean_angles_2,
            'FR_Mean Move Distance': mean_move_distances_2
        })
df_result_2.to_excel(output_file_3, index=False)

for folder in os.listdir(change_obj_output_3):
    folder_path = os.path.join(change_obj_output_3, folder)
    # 确保是文件夹
    if os.path.isdir(folder_path):
        max_force = []
        angles=[]
        move_distances=[]
        # 遍历文件夹中的所有 CSV 文件
        for file in os.listdir(folder_path):
            if "decrease_froce" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                file_path = os.path.join(folder_path, file)
                try:
                    # 读取 CSV 文件
                    df = pd.read_csv(file_path)
                    
                    # 获取最后一行值
                    last_row = df.iloc[-1]
                    angle_value = last_row[4]
                    move_distance = last_row[5]
                    angles.append(angle_value)
                    move_distances.append(move_distance)
                    # 提取 "left" 和 "right" 的值
                    force_value = last_row[2]
                    match = re.match(r"left:(-?\d+(\.\d+)?),right:(-?\d+(\.\d+)?)", str(force_value))
                    if match:
                        left_value = float(match.group(1))
                        right_value = float(match.group(3))
                        
                        # 取最大值
                        max_force.append(min(left_value, right_value))
                except Exception as e:
                    print(f"无法读取文件 {file_path}: {e}")
        # 计算均值
        if max_force:
            mean_force = sum(max_force) / len(max_force)
            mean_angle = sum(angles) / len(angles)
            mean_move_distance = sum(move_distances) / len(move_distances)
            folder_names_3.append(folder)  # 保存文件夹名称
            mean_forces_3.append(mean_force)  # 保存均值
            mean_angles_3.append(mean_angle)
            mean_move_distances_3.append(mean_move_distance)
        else:
            folder_names_3.append(folder)
            mean_forces_3.append(None)
            mean_angles_3.append(None)
            mean_move_distances_3.append(None)

# 将文件夹名称和均值写入 Excel 文件
df_result_3 = pd.DataFrame({
            'Object Name': folder_names_3,
            'PAG_Mean Force': mean_forces_3,
            'PAG_Mean Angle': mean_angles_3,
            'PAG_Mean Move Distance': mean_move_distances_3
        })
df_result_3.to_excel(output_file_4, index=False)



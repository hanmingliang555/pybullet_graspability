import os
import pandas as pd
import re
import numpy as np
# 输入文件夹路径和输出 Excel 文件路径
base_dir = '2gripper_data/output'  # 替换为 decrease_force 的路径
json_dir = 'decrease_force'
input_dir=os.path.join(base_dir,json_dir)
friction=0.8
distance=0.05
angle=10
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
        angles=[]
        move_distances=[]
        # 遍历文件夹中的所有 CSV 文件
        for file in os.listdir(folder_path):
            if "decrease_force" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                file_path = os.path.join(folder_path, file)
                if os.path.exists(file_path):
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
                            match = re.match(r"left:(-?\d+(\.\d+)?),right:(-?\d+(\.\d+)?)", str(force_value))
                            if match:
                                left_value = float(match.group(1))
                                right_value = float(match.group(3))   
                                # 取最大值
                                max_force.append(min(left_value, right_value))
                                break
                        else :
                            i-=1
                    if i == 0:
                        max_force.append(-50)
                        angles.append(angle)
                        move_distances.append(distance)  

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
mean_forces = [abs(mean_force if mean_force is not None else 0) for mean_force in mean_forces]
# 将文件夹名称和均值写入 Excel 文件
df_result = pd.DataFrame({
            'Object Name': folder_names,
            'Mean Force': mean_forces,
            #'Mean Angle': mean_angles,
            #'Mean Move Distance': mean_move_distances
        })
# 保留有效数字
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
                if "decrease_force" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(folder_path, file)
                    standard_path_1 = os.path.join(standard_path,file)
                    if os.path.exists(standard_path_1) and os.path.exists(file_path):
                        #try:
                        # 读取 CSV 文件
                        df = pd.read_csv(file_path)
                        standard_csv = pd.read_csv(standard_path_1)
                        j=len(standard_csv)
                        while(j>0):
                            standard_data = standard_csv.iloc[j-1]
                            standard_move = standard_data[5]+0.01
                            standard_angle = standard_data[4]/friction
                            if standard_data[5]<distance and standard_data[4]<angle:
                                i=len(df)
                                while(i>0):
                                    # 获取最后一行值
                                    last_row = df.iloc[i-1]
                                    move_distance = last_row[5]
                                    angle_value = last_row[4]
                                    if move_distance<standard_move and angle_value<standard_angle:

                                        angles.append(angle_value)
                                        move_distances.append(move_distance)
                                        # 提取 "left" 和 "right" 的值
                                        force_value = last_row[2]
                                        match = re.match(r"left:(-?\d+(\.\d+)?),right:(-?\d+(\.\d+)?)", str(force_value))
                                        if match:
                                            left_value = float(match.group(1))
                                            right_value = float(match.group(3))
                                            
                                            # 取最大值
                                            max_force.append(min(left_value,right_value))
                                        break
                                    else :
                                        i-=1
                                if i == 0:
                                    max_force.append(-50)
                                    angles.append(standard_angle)
                                    move_distances.append(standard_move)
                                break
                            else :
                                j-=1

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
mean_forces_1 = [abs(mean_force if mean_force is not None else 0) for mean_force in mean_forces_1]
# 将文件夹名称和均值写入 Excel 文件
df_result_1 = pd.DataFrame({
            'Object Name': folder_names_1,
            'FC_Mean Force': mean_forces_1,
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
                if "decrease_force" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(folder_path, file)
                    standard_path_1 = os.path.join(standard_path,file)
                    if os.path.exists(standard_path_1) and os.path.exists(file_path):
                        #try:
                        # 读取 CSV 文件
                        df = pd.read_csv(file_path)
                        standard_csv = pd.read_csv(standard_path_1)
                        j=len(standard_csv)
                        while(j>0):
                            standard_data = standard_csv.iloc[j-1]
                            standard_move = standard_data[5]+0.01
                            standard_angle = standard_data[4]/friction
                            if standard_data[5]<distance and standard_data[4]<angle:
                                i=len(df)
                                while(i>0):
                                    # 获取最后一行值
                                    last_row = df.iloc[i-1]
                                    move_distance = last_row[5]
                                    angle_value = last_row[4]
                                    if move_distance<standard_move and angle_value<standard_angle:

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
                                        break
                                    else :
                                        i-=1
                                if i == 0:
                                    max_force.append(-50)
                                    angles.append(standard_angle)
                                    move_distances.append(standard_move)
                                break
                            else :
                                j-=1
                                
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
mean_forces_2 = [abs(mean_force if mean_force is not None else 0) for mean_force in mean_forces_2]
# 将文件夹名称和均值写入 Excel 文件
df_result_2 = pd.DataFrame({
            'Object Name': folder_names_2,
            'FR_Mean Force': mean_forces_2,
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
                if "decrease_force" in file and file.endswith(".csv"): # 检查是否是 CSV 文件  
                    file_path = os.path.join(folder_path, file)
                    standard_path_1 = os.path.join(standard_path,file)
                    if os.path.exists(standard_path_1) and os.path.exists(file_path):
                        #try:
                        # 读取 CSV 文件
                        df = pd.read_csv(file_path)
                        standard_csv = pd.read_csv(standard_path_1)
                        j=len(standard_csv)
                        while(j>0):
                            standard_data = standard_csv.iloc[j-1]
                            standard_move = standard_data[5]+0.01
                            standard_angle = standard_data[4]/friction
                            if standard_data[5]<distance and standard_data[4]<angle:
                                i=len(df)
                                while(i>0):
                                    # 获取最后一行值
                                    last_row = df.iloc[i-1]
                                    move_distance = last_row[5]
                                    angle_value = last_row[4]
                                    if move_distance<standard_move and angle_value<standard_angle:

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
                                        break
                                    else :
                                        i-=1
                                if i == 0:
                                    max_force.append(-50)
                                    angles.append(standard_angle)
                                    move_distances.append(standard_move)
                                break
                            else :
                                j-=1
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

mean_forces_3 = [abs(mean_force if mean_force is not None else 0) for mean_force in mean_forces_3]
# 将文件夹r名称和均值写入 Excel 文件
df_result_3 = pd.DataFrame({
            'Object Name': folder_names_3,
            'PAG_Mean Force':  mean_forces_3,
            #'PAG_Mean Angle': mean_angles_3,
            #'PAG_Mean Move Distance': mean_move_distances_3
        })
df_result_3.to_excel(output_file_4, index=False)


# 读取四个 Excel 文件
file_paths = [output_file_1,output_file_2,output_file_3,output_file_4,
]
output_file='2gripper_data/output/decrease_force/decrease_force.xlsx'

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

# 函数：保留四位有效数字
def round_to_significant_digits(x, digits=4):
    if pd.isna(x):  # 如果是 NaN 或 None，跳过四舍五入
        return x
    if x == 0:
        return 0
    else:
        return round(x, digits - int(np.floor(np.log10(abs(x)))) - 1)

# 对除了 'Object Name' 列以外的所有数值列进行四舍五入保留四位有效数字
merged_df = merged_df.applymap(lambda x: round_to_significant_digits(x) if isinstance(x, (int, float)) else x)


# 将合并后的数据保存为新的 Excel 文件
merged_df.to_excel(output_file, index=True)


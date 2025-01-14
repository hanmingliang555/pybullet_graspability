import os
import re
import shutil

# 根目录路径
root_dir = "rst_0113_5_B/rst_0113_5/PAG-30-0.01"
output_dir = "filter_obj_json_B/change_obj/PAG"  # 输出文件夹路径
os.makedirs(output_dir, exist_ok=True)  # 创建输出目录

# 遍历根目录中的所有子文件夹
for folder in os.listdir(root_dir):
    folder_path = os.path.join(root_dir, folder)
    if os.path.isdir(folder_path):  # 检查是否是文件夹

        max_iter_file = None
        max_major_iter = -1  # 主迭代
        max_minor_iter = -1  # 次迭代

        # 遍历文件夹中的所有 .obj 文件
        for file in os.listdir(folder_path):
            if file.endswith(".obj"):
                # 使用正则表达式提取 iter_<数字>_<数字> 的内容
                match = re.search(r"iter_(\d+)_(\d+)", file)
                if match:
                    major_iter = int(match.group(1))  # 提取主迭代次数
                    minor_iter = int(match.group(2))  # 提取次迭代次数

                    # 比较主迭代次数和次迭代次数
                    if major_iter > max_major_iter or (major_iter == max_major_iter and minor_iter > max_minor_iter):
                        max_major_iter = major_iter
                        max_minor_iter = minor_iter
                        max_iter_file = file

        # 如果找到最大迭代次数的文件
        if max_iter_file:
            # 提取文件夹名称中的数字
            match = re.search(r"obj(\d+)-.*-(\d+)", folder)
            if match:
                obj_number = match.group(1)  # 提取 `38`
                scene_number = match.group(2)  # 提取 `0`

                # 创建对应物体编号的子文件夹
                obj_output_dir = os.path.join(output_dir, obj_number.zfill(4))
                os.makedirs(obj_output_dir, exist_ok=True)

                # 复制文件到子文件夹，并重命名为 `38_0.obj`
                src_file_path = os.path.join(folder_path, max_iter_file)
                dest_file_path = os.path.join(obj_output_dir, f"{obj_number}_{scene_number}.obj")
                shutil.copy(src_file_path, dest_file_path)
                print(f"复制并重命名文件：{src_file_path} -> {dest_file_path}")

print(f"所有文件已分组并重命名复制到目录：{output_dir}")

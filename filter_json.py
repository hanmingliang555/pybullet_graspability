import os
import json
import pandas as pd

num = 7
# 根目录路径
root_dir = "new_json"
output_dir = "merged_json_1"
csv_root_dir = "output_new_json"  # CSV 文件所在的根目录
os.makedirs(output_dir, exist_ok=True)  # 创建输出目录

# 获取所有场景文件夹
scene_folders = sorted([f for f in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir, f))])

# 提取每个物体的 JSON 文件并合并
object_files = {}  # 用于记录每个物体的 JSON 文件路径

for scene_folder in scene_folders:
    if scene_folder == "scene_0111":  # 只处理 scene_0110
        scene_path = os.path.join(root_dir, scene_folder)
        csv_path = os.path.join(csv_root_dir, scene_folder)
        json_files = sorted([f for f in os.listdir(scene_path) if f.endswith(".json")])
    else:
        continue
    for json_file in json_files:
        object_id = json_file  # 每个物体的 JSON 文件名
        if object_id not in object_files:
            object_files[object_id] = []
        object_files[object_id].append(os.path.join(scene_path, json_file))

# 合并 JSON 文件
for object_id, file_paths in object_files.items():
    merged_data = []  # 存放最终的合并数据

    # 读取对应的 CSV 文件
    scene_id = "scene_0110"  # 当前场景 ID
    object_folder_name = object_id.split('_')[-1].split('.')[0]  # 获取物体文件夹名称
    csv_file_path = os.path.join(csv_path, object_folder_name, f"{object_folder_name}.csv")
    
    if not os.path.exists(csv_file_path):
        print(f"警告：未找到 CSV 文件 {csv_file_path}，已跳过。")
        continue

    # 读取 CSV 文件，筛选抓取成功的索引
    csv_data = pd.read_csv(csv_file_path)
    success_indices = csv_data[csv_data["抓取结果"] == "抓取成功"].index.tolist()  # 假设抓取成功标志为列名 "抓取结果"

    # 遍历每个文件路径
    for file_path in file_paths:
        with open(file_path, "r") as f:
            try:
                json_data = json.load(f)  # 加载 JSON 数据
                if isinstance(json_data, list) and len(json_data) > 0:
                    # 根据成功的索引筛选数据并限制数量
                    for i, item in enumerate(json_data):
                        if i in success_indices and len(merged_data) < num:
                            merged_data.append(item)
            except json.JSONDecodeError:
                print(f"警告：文件 {file_path} 无法解析为 JSON,已跳过。")
                continue

    # 写入合并后的数据
    if len(merged_data) > 0:
        output_file = os.path.join(output_dir, object_id)
        with open(output_file, "w") as f:
            json.dump(merged_data, f, indent=4)
        print(f"物体 {object_id} 已合并完成，数据数量：{len(merged_data)}。")
    else:
        print(f"警告：物体 {object_id} 没有任何可用数据，已跳过。")

print(f"合并完成！合并后的文件保存在: {output_dir}")

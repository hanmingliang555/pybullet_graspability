# README

## 环境配置

    pip install -r requirements.txt

## 二爪实验

减少抓取力实验
原物体  python 2gripper_test/decrease_force.py
优化后物体     python 2gripper_test/change_obj_decrease_force.py

增加物体质量
原物体  python 2gripper_test/increase_mass.py
优化后物体  python 2gripper_test/change_obj_increase_mass.py

添加扰动
原物体  python 2gripper_test/random_perturbation.py
优化后物体  python 2gripper_test/change_obj_random_perturbation.py

上面对原物体的实验读取的是2gripper_data/json和对应的2gripper_data/obj，将结果存储在2gripper_data/ouput/对应实验名/obj
对原物体的实验会遍历2gripper_data/json下所有的json进行实验

对优化后物体的实验读取的是2gripper_data/json和对应的2gripper_data/chagn_obj/方法名/物体序号/物体_姿势.obj，将结果存储在2gripper_data/ouput/对应实验名/change_obj/方法名/物体序号/xxx.csv（csv文件命名规律是物体序号_姿势序号_实验名字）
对优化后的物体实验是一个方法一个方法进行的，以2gripper_test/change_obj_decrease_force.py为例.如果要修改优化方法只要修改944行的"change_obj/FR"，将FR修改为对应的文件名，此时是对该方法文件夹下的所有模型进行遍历实验;如果要针对某个方法的某个物体进行实验将953行的if scene_folder.endswith(".json"):修改为if scene_folder.endswith(".json") and scene_folder=="0000_00xx.json":即可（xx是物体序号）
另外两个实验同上

## 三爪实验

减少抓取力实验
原物体  python 3gripper_test/decrease_force.py
优化后物体     python 3gripper_test/change_obj_decrease_force.py

增加物体质量
原物体  python 3gripper_test/increase_mass.py
优化后物体  python 3gripper_test/change_obj_increase_mass.py

添加扰动
原物体  python 3gripper_test/random_perturbation.py
优化后物体  python 3gripper_test/change_obj_random_perturbation.py

3gripper_test里面以all开头的python是用来对所有优化方法产生的所有迭代物体进行循环遍历实验，结果输出到alldata/output/实验名/change_obj/方法名/物体序号/姿势序号/xxx.csv(xxx前两个数字使用的是实验所用的obj的文件名)(这里所输出的文件路径多了一层output和change_obj是为了和2爪，3爪产生的实验结果输出路径对齐)

## 数据处理

2爪数据处理

处理减少抓取力数据：python data_processing/2gripper/excel_decrease_force.py
这个是将筛选后的数据求平均得到的，将结果输出到了2gripper_data/output/decrease_force下的五个.xlsx文件，分别是三个方法，一个原始物体以及总的表格。
如果要修改筛选条件，只需要修改10和11行的distance=0.05，angle=10这两个值即可，单位分别是米和度。那个friction没什么用

处理增加物体质量数据：python data_processing/2gripper/excel_increase_mass.py
处理添加扰动数据：python data_processing/2gripper/excel_random_perturbation.py
使用方法同上
3爪数据处理同上

data_processing/select中的select_xxx.python是用来处理对所有迭代物体进行实验产生的所有数据，输出的文件在alldata/output/实验名/change_obj下面
其他的python没用。

有一个rts_filter.py是为了从迭代产生的所有的优化obj中过滤出迭代次数最多的obj

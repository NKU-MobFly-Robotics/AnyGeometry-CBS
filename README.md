## AnyGeometry-CBS

**Any Geometry Conflict-Based Search for Multi-Agent Path Finding**

### ℹ️ About / 项目简介

**EN:**  
AG-CBS is a framework designed for Multi-Agent Path Finding (MAPF) with arbitrary shapes. Unlike traditional single grid-based or point-agent methods, AG-CBS handles complex geometries to ensure collision-free paths in more realistic environments.

**CN:**  
AG-CBS 是一个专为具有任意形状智能体设计的多智能体路径规划（MAPF）框架。相比传统的基于单个栅格或质点模型的算法，AG-CBS 能够处理复杂的几何形状，确保在更真实的场景中实现无碰撞路径规划。

### Build Instructions / 编译步骤

请在终端中执行以下命令：

```bash
# 1. Clone the repository / 克隆仓库
git clone https://github.com/NKU-MobFly-Robotics/AnyGeometry-CBS.git
cd AnyGeometry-CBS

# 2. Build with CMake / 使用 CMake 编译
mkdir build && cd build
cmake ..
make -j
```

### 🚀 Usage / 运行示例

**EN:**  
You can test the planners using the provided map. Please ensure the output directory `../src/output/` exists before running.

**CN:**  
你可以使用内置的地图进行测试。运行前请确保输出目录 `../src/output/` 已存在。

#### Running AG-CBS:

```bash
./AG_CBS -i ../src/maps/warehouse_agent6_3.yaml -o ../src/output/result1.yaml
```

#### Running AG-ECBS:

```bash
./AG_ECBS -i ../src/maps/warehouse_agent6_3.yaml -o ../src/output/result2.yaml
```

### 🤝 Acknowledgments / 致谢

**EN:**  
This implementation incorporates and extends parts of the [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning) library. We thank the original authors for their contributions to the MAPF community.

**CN:**  
本项目参考并扩展了 libMultiRobotPlanning 库的部分功能。感谢原作者对 MAPF 社区的贡献。

### 📄 License / 开源协议

This project is licensed under the MIT License. 
本项目采用 MIT License 开源。

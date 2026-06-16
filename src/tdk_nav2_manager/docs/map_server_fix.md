# Fix: Nav2 無法載入預掃描地圖

## 問題描述

Nav2 路徑規劃時會穿透牆壁，因為 global costmap 的 `static_layer` 沒有收到預掃描地圖，
只依賴即時 lidar scan（`obstacle_layer`），導致視野外的牆壁對規劃器不存在。

## 根本原因

`nav_launch.py` 只啟動 `navigation_launch.py`（nav2 navigation stack），
沒有任何節點負責發佈靜態地圖到 `/map` topic。

雖然 Cartographer 的 `cartographer_occupancy_grid_node` 也會發佈 `/map`，
但存在以下問題：
- **QoS 不相容**：nav2 `static_layer` 以 `TRANSIENT_LOCAL` 訂閱，Cartographer 可能以 `VOLATILE` 發佈
- **啟動時序**：nav2 lifecycle 啟動時 Cartographer 可能尚未發佈第一幀地圖

## 修改內容

### 1. `src/tdk_nav2_manager/launch/nav_launch.py`

新增 `map_server` 節點與其專屬 `lifecycle_manager`，
負責將預掃描地圖以正確 QoS（`TRANSIENT_LOCAL`）發佈到 `/map`。

```
新增節點：
  - map_server          → 讀取 carto_map_0.yaml，發佈 /map
  - lifecycle_manager_map → 獨立管理 map_server 生命週期

新增 launch argument：
  - map  (預設: .../maps/carto_map_0.yaml，可覆蓋)
```

### 2. `src/tdk_slam_manager/launch/sim_spawn_launch.py`
### 3. `src/tdk_slam_manager/launch/spawn_launch.py`

`cartographer_occupancy_grid_node` 加上 topic remapping，
避免和 `map_server` 同時發佈 `/map` 造成衝突。

```
/map  →  /carto_map   (僅供 RViz 視覺化，不影響 nav2)
```

### 4. `src/tdk_slam_manager/launch/spawn_launch.py`（amcl 模式修復）

amcl 模式下的 `map_server` 原本 `yaml_filename` 被 comment 掉，
補回正確路徑。

```python
# 修改前
# {'yaml_filename': map_yaml_file},

# 修改後
{'yaml_filename': map_yaml_file},   # carto_map_0.yaml
```

## 修改後的架構

```
cartographer_node (localization)  ──→  TF: map → odom → base_footprint
map_server (carto_map_0.yaml)     ──→  /map  (TRANSIENT_LOCAL QoS)
cartographer_occupancy_grid_node  ──→  /carto_map  (RViz 視覺化用)

Nav2 global_costmap:
  static_layer   ←── /map  ✓ 預掃描牆壁全部可見
  obstacle_layer ←── /scan   即時障礙物
```

## 使用方式

預設使用 `carto_map_0.yaml`，無需額外參數：

```bash
ros2 launch tdk_nav2_manager nav_launch.py
```

指定其他地圖：

```bash
ros2 launch tdk_nav2_manager nav_launch.py map:=/path/to/your_map.yaml
```

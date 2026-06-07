#!/usr/bin/env python3
"""区域数据持久化 — JSON 文件读写

提供线程安全的区域增删改查操作, 数据存储在 JSON 文件中.
支持旧格式 (x1/y1/x2/y2) 到新格式 (cx/cy/width/height) 的自动迁移.
"""

import json
import os
import threading

DEFAULT_COLOR = '#4A90D9'


class RegionStore:
    """区域数据存储器

    线程安全 (threading.Lock), 适用于 ROS2 多线程回调环境.

    数据格式 (JSON):
    {
        "regions": [
            {
                "name": "客厅",
                "cx": -1.5, "cy": 2.0,
                "width": 2.0, "height": 2.0,
                "rotation": 0.0,
                "color": "#FF6B6B"
            }
        ]
    }
    """

    def __init__(self, file_path):
        """
        Args:
            file_path: JSON 文件路径
        """
        self._file_path = file_path
        self._lock = threading.Lock()
        self._regions = []
        self._load()

    def _load(self):
        """从 JSON 文件加载区域数据"""
        if os.path.exists(self._file_path):
            try:
                with open(self._file_path, 'r') as f:
                    data = json.load(f)
                self._regions = data.get('regions', [])
                self._migrate()
            except (json.JSONDecodeError, IOError):
                self._regions = []
        else:
            self._regions = []

    def _migrate(self):
        """旧格式 → 新格式自动迁移

        旧格式: {name, x1, y1, x2, y2, center_x, center_y, color}
        新格式: {name, cx, cy, width, height, rotation, color}

        迁移后自动保存为新格式.
        """
        changed = False
        for r in self._regions:
            if 'x1' in r:
                # 旧格式含 x1/y1/x2/y2 → 转为 cx/cy/width/height
                r['cx'] = r.pop('center_x', (r['x1'] + r['x2']) / 2.0)
                r['cy'] = r.pop('center_y', (r['y1'] + r['y2']) / 2.0)
                r['width'] = abs(r.pop('x2') - r.pop('x1'))
                r['height'] = abs(r.pop('y2') - r.pop('y1'))
                r['rotation'] = 0.0
                r.pop('center_x', None)
                r.pop('center_y', None)
                changed = True
        if changed:
            self._save()

    def _save(self):
        """保存区域数据到 JSON 文件"""
        os.makedirs(os.path.dirname(self._file_path), exist_ok=True)
        with open(self._file_path, 'w') as f:
            json.dump({'regions': self._regions}, f, indent=2, ensure_ascii=False)

    def add(self, name, cx, cy, width, height, rotation=0.0, color=DEFAULT_COLOR):
        """添加或更新区域 (同名覆盖)

        Args:
            name: 区域名称 (唯一标识)
            cx, cy: 区域中心坐标 (map 坐标系)
            width, height: 区域宽度和高度 (米)
            rotation: 旋转角 (弧度)
            color: HEX 颜色字符串, 如 "#FF6B6B"
        """
        with self._lock:
            # 移除同名的旧区域 (相当于 upsert)
            self._regions = [r for r in self._regions if r['name'] != name]
            self._regions.append({
                'name': name,
                'cx': cx, 'cy': cy,
                'width': width, 'height': height,
                'rotation': rotation,
                'color': color,
            })
            self._save()

    def delete(self, name):
        """删除指定名称的区域

        Returns:
            True 如果找到并删除, False 如果不存在
        """
        with self._lock:
            before = len(self._regions)
            self._regions = [r for r in self._regions if r['name'] != name]
            if len(self._regions) != before:
                self._save()
                return True
            return False

    def get(self, name):
        """按名称查找区域

        Returns:
            区域 dict 或 None
        """
        for r in self._regions:
            if r['name'] == name:
                return r
        return None

    def all(self):
        """返回所有区域的副本"""
        return list(self._regions)

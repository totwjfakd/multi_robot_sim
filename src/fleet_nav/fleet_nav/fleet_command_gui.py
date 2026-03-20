#!/usr/bin/env python3
"""
Fleet Command GUI
-----------------
현재 multi_robot_controller의 외부 명령 인터페이스
  /fleet_nav/command
  /fleet_nav/status
에 맞춰 특정 로봇을 특정 node id로 보내는 GUI.
"""

import json
import tkinter as tk
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


COMMAND_TOPIC = '/fleet_nav/command'
STATUS_TOPIC = '/fleet_nav/status'


class FleetCommandGuiNode(Node):
    def __init__(self):
        super().__init__('fleet_command_gui')

        self.declare_parameter('num_robots', 10)
        self.declare_parameter('robot_prefix', 'robot_')
        self.declare_parameter('graph_file', '')
        self.declare_parameter('window_title', 'Fleet Command GUI')

        num_robots = int(self.get_parameter('num_robots').value)
        robot_prefix = str(self.get_parameter('robot_prefix').value)

        self.window_title = str(self.get_parameter('window_title').value)
        self.robot_names = [f'{robot_prefix}{i + 1}' for i in range(num_robots)]
        self.node_coords, self.edge_pairs = self._load_graph(
            str(self.get_parameter('graph_file').value)
        )
        self.node_map = {node_id: (x, y) for node_id, x, y in self.node_coords}
        self.graph_bounds = self._compute_graph_bounds()

        self.robot_poses = {name: None for name in self.robot_names}
        self.pose_version = 0
        self._status_listeners = []

        self._command_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.create_subscription(String, STATUS_TOPIC, self._status_cb, 10)

        for robot_name in self.robot_names:
            self.create_subscription(
                Odometry,
                f'/{robot_name}/odom',
                lambda msg, name=robot_name: self._odom_cb(msg, name),
                10,
            )

    def _load_graph(self, graph_file: str):
        if not graph_file:
            return [], []

        try:
            with open(graph_file) as f:
                data = json.load(f)
        except Exception as exc:
            self.get_logger().error(f'graph_file 로드 오류: {exc}')
            return [], []

        nodes = []
        raw_edges = []
        for feat in data.get('features', []):
            geometry = feat.get('geometry', {})
            geometry_type = geometry.get('type')
            props = feat.get('properties', {})

            if geometry_type == 'Point':
                try:
                    node_id = int(props['id'])
                    x = float(geometry['coordinates'][0])
                    y = float(geometry['coordinates'][1])
                except (KeyError, TypeError, ValueError):
                    continue
                nodes.append((node_id, x, y))
                continue

            if geometry_type == 'MultiLineString':
                try:
                    start_id = int(props['startid'])
                    end_id = int(props['endid'])
                except (KeyError, TypeError, ValueError):
                    continue
                raw_edges.append((start_id, end_id))

        node_ids = {node_id for node_id, _, _ in nodes}
        edge_pairs = []
        seen_edges = set()
        for start_id, end_id in raw_edges:
            key = tuple(sorted((start_id, end_id)))
            if key in seen_edges:
                continue
            if start_id not in node_ids or end_id not in node_ids:
                continue
            seen_edges.add(key)
            edge_pairs.append((start_id, end_id))

        return sorted(nodes, key=lambda item: item[0]), edge_pairs

    def _compute_graph_bounds(self):
        if not self.node_coords:
            return None

        xs = [x for _, x, _ in self.node_coords]
        ys = [y for _, _, y in self.node_coords]
        return {
            'min_x': min(xs),
            'max_x': max(xs),
            'min_y': min(ys),
            'max_y': max(ys),
        }

    def add_status_listener(self, callback):
        self._status_listeners.append(callback)

    def _status_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {
                'level': 'error',
                'message': msg.data,
            }

        for callback in self._status_listeners:
            callback(payload)

    def _odom_cb(self, msg: Odometry, robot_name: str):
        new_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        previous_pose = self.robot_poses.get(robot_name)
        self.robot_poses[robot_name] = new_pose

        if previous_pose is None:
            self.pose_version += 1
            return

        dx = new_pose[0] - previous_pose[0]
        dy = new_pose[1] - previous_pose[1]
        if (dx * dx + dy * dy) >= 0.0004:
            self.pose_version += 1

    def send_command(self, robot_name: str, goal_node_id: int):
        payload = {
            'robot_name': robot_name,
            'goal_node_id': int(goal_node_id),
            'source': self.get_name(),
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._command_pub.publish(msg)
        return payload


class FleetCommandGuiApp:
    ROS_POLL_MS = 50
    GRAPH_PADDING = 44
    NODE_RADIUS = 6

    def __init__(self, node: FleetCommandGuiNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title(node.window_title)
        self.root.geometry('1280x820')
        self.root.configure(bg='#f3efe6')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self._build_style()

        default_robot = node.robot_names[0] if node.robot_names else ''
        default_node = str(node.node_coords[0][0]) if node.node_coords else ''

        self.robot_var = tk.StringVar(value=default_robot)
        self.node_var = tk.StringVar(value=default_node)
        self.selection_var = tk.StringVar(value='노드를 선택하면 좌표가 표시됩니다.')
        self.zoom_var = tk.StringVar(value='100%')
        self.graph_hint_var = tk.StringVar(
            value='마우스 휠로 확대/축소, 드래그로 이동, 노드 클릭으로 선택할 수 있습니다. 전송은 버튼이나 Ctrl+Enter를 사용하세요.'
        )

        self._view_scale = 1.0
        self._view_offset_x = 0.0
        self._view_offset_y = 0.0
        self._drag_start = None
        self._dragging = False

        self._render_pending = False
        self._graph_dirty = True
        self._robots_dirty = True
        self._last_canvas_size = None
        self._last_pose_version = node.pose_version
        self._syncing_table_selection = False
        self._node_table_items = {}
        self._canvas_items = {}

        self.node.add_status_listener(self._handle_status)

        self._build_layout()
        self._populate_nodes()
        self._refresh_selection_label()
        self.root.after(0, self._request_render)

    def _build_style(self):
        style = ttk.Style()
        style.theme_use('clam')

        style.configure('App.TFrame', background='#f3efe6')
        style.configure('Panel.TFrame', background='#fffaf2')
        style.configure('Title.TLabel',
                        background='#f3efe6',
                        foreground='#2e3b2f',
                        font=('TkDefaultFont', 16, 'bold'))
        style.configure('Subtitle.TLabel',
                        background='#f3efe6',
                        foreground='#5b6656',
                        font=('TkDefaultFont', 10))
        style.configure('Field.TLabel',
                        background='#fffaf2',
                        foreground='#314032',
                        font=('TkDefaultFont', 10, 'bold'))
        style.configure('Body.TLabel',
                        background='#fffaf2',
                        foreground='#455243',
                        font=('TkDefaultFont', 10))
        style.configure('Hint.TLabel',
                        background='#fffaf2',
                        foreground='#5f6f5b',
                        font=('TkDefaultFont', 9))
        style.configure('Accent.TButton',
                        font=('TkDefaultFont', 10, 'bold'),
                        padding=(10, 8))
        style.map('Accent.TButton',
                  background=[('active', '#c9d8b6'), ('!disabled', '#d8e6c8')],
                  foreground=[('!disabled', '#24311f')])

    def _build_layout(self):
        self.root.columnconfigure(0, weight=5)
        self.root.columnconfigure(1, weight=3)
        self.root.rowconfigure(1, weight=3)
        self.root.rowconfigure(2, weight=2)

        header = ttk.Frame(self.root, style='App.TFrame', padding=(18, 18, 18, 8))
        header.grid(row=0, column=0, columnspan=2, sticky='nsew')
        header.columnconfigure(0, weight=1)

        ttk.Label(
            header,
            text='Fleet Navigation Command Panel',
            style='Title.TLabel',
        ).grid(row=0, column=0, sticky='w')
        ttk.Label(
            header,
            text=f'토픽: {COMMAND_TOPIC}  |  상태: {STATUS_TOPIC}',
            style='Subtitle.TLabel',
        ).grid(row=1, column=0, sticky='w', pady=(4, 0))

        graph_panel = ttk.Frame(self.root, style='Panel.TFrame', padding=18)
        graph_panel.grid(row=1, column=0, sticky='nsew', padx=(18, 9), pady=(0, 12))
        graph_panel.columnconfigure(0, weight=1)
        graph_panel.rowconfigure(1, weight=1)

        graph_header = ttk.Frame(graph_panel, style='Panel.TFrame')
        graph_header.grid(row=0, column=0, sticky='ew')
        graph_header.columnconfigure(0, weight=1)

        ttk.Label(graph_header, text='Navigation Graph', style='Field.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Label(graph_header, textvariable=self.zoom_var, style='Body.TLabel').grid(row=0, column=1, sticky='e', padx=(0, 8))
        ttk.Button(graph_header, text='-', width=3, command=self._zoom_out).grid(row=0, column=2, sticky='e', padx=(0, 4))
        ttk.Button(graph_header, text='+', width=3, command=self._zoom_in).grid(row=0, column=3, sticky='e', padx=(0, 4))
        ttk.Button(graph_header, text='맞춤', command=self._reset_view).grid(row=0, column=4, sticky='e')

        self.graph_canvas = tk.Canvas(
            graph_panel,
            background='#fffdf8',
            highlightthickness=0,
            relief='flat',
        )
        self.graph_canvas.grid(row=1, column=0, sticky='nsew', pady=(10, 10))
        self.graph_canvas.bind('<ButtonPress-1>', self._on_canvas_press)
        self.graph_canvas.bind('<B1-Motion>', self._on_canvas_drag)
        self.graph_canvas.bind('<ButtonRelease-1>', self._on_canvas_release)
        self.graph_canvas.bind('<MouseWheel>', self._on_mousewheel)
        self.graph_canvas.bind('<Button-4>', self._on_mousewheel_linux)
        self.graph_canvas.bind('<Button-5>', self._on_mousewheel_linux)
        self.graph_canvas.bind('<Configure>', self._on_canvas_configure)
        self.root.bind('<Control-Return>', self._send_command)

        ttk.Label(
            graph_panel,
            textvariable=self.graph_hint_var,
            style='Hint.TLabel',
            wraplength=720,
            justify='left',
        ).grid(row=2, column=0, sticky='w')

        sidebar = ttk.Frame(self.root, style='Panel.TFrame', padding=18)
        sidebar.grid(row=1, column=1, sticky='nsew', padx=(9, 18), pady=(0, 12))
        sidebar.columnconfigure(0, weight=1)
        sidebar.rowconfigure(5, weight=1)

        ttk.Label(sidebar, text='명령 패널', style='Field.TLabel').grid(row=0, column=0, sticky='w')

        controls = ttk.Frame(sidebar, style='Panel.TFrame')
        controls.grid(row=1, column=0, sticky='ew', pady=(10, 16))
        controls.columnconfigure(1, weight=1)

        ttk.Label(controls, text='로봇', style='Field.TLabel').grid(row=0, column=0, sticky='w')
        self.robot_combo = ttk.Combobox(
            controls,
            textvariable=self.robot_var,
            values=self.node.robot_names,
            state='readonly',
        )
        self.robot_combo.grid(row=0, column=1, sticky='ew', pady=(0, 12))
        self.robot_combo.bind('<<ComboboxSelected>>', self._on_robot_changed)

        ttk.Label(controls, text='목적지 node id', style='Field.TLabel').grid(row=1, column=0, sticky='w')
        self.node_combo = ttk.Combobox(
            controls,
            textvariable=self.node_var,
            values=[str(node_id) for node_id in self.node.node_map],
        )
        self.node_combo.grid(row=1, column=1, sticky='ew', pady=(0, 12))
        self.node_combo.bind('<Return>', self._send_command)

        ttk.Label(
            controls,
            textvariable=self.selection_var,
            style='Body.TLabel',
            wraplength=320,
            justify='left',
        ).grid(row=2, column=0, columnspan=2, sticky='w', pady=(0, 16))

        ttk.Button(
            controls,
            text='이동 명령 전송',
            style='Accent.TButton',
            command=self._send_command,
        ).grid(row=3, column=0, columnspan=2, sticky='ew')

        node_panel = ttk.Frame(sidebar, style='Panel.TFrame')
        node_panel.grid(row=5, column=0, sticky='nsew')
        node_panel.columnconfigure(0, weight=1)
        node_panel.rowconfigure(1, weight=1)

        ttk.Label(node_panel, text='그래프 노드 목록', style='Field.TLabel').grid(row=0, column=0, sticky='w')

        self.node_table = ttk.Treeview(
            node_panel,
            columns=('id', 'x', 'y'),
            show='headings',
            height=14,
        )
        self.node_table.heading('id', text='Node ID')
        self.node_table.heading('x', text='X')
        self.node_table.heading('y', text='Y')
        self.node_table.column('id', width=90, anchor='center')
        self.node_table.column('x', width=110, anchor='e')
        self.node_table.column('y', width=110, anchor='e')
        self.node_table.grid(row=1, column=0, sticky='nsew', pady=(10, 0))
        self.node_table.bind('<<TreeviewSelect>>', self._on_table_selected)

        log_panel = ttk.Frame(self.root, style='Panel.TFrame', padding=18)
        log_panel.grid(row=2, column=0, columnspan=2, sticky='nsew', padx=18, pady=(0, 18))
        log_panel.columnconfigure(0, weight=1)
        log_panel.rowconfigure(1, weight=1)

        ttk.Label(log_panel, text='상태 로그', style='Field.TLabel').grid(row=0, column=0, sticky='w')
        self.log = ScrolledText(
            log_panel,
            height=12,
            wrap='word',
            bg='#fcfaf5',
            fg='#24311f',
            relief='flat',
            padx=10,
            pady=10,
        )
        self.log.grid(row=1, column=0, sticky='nsew', pady=(10, 0))
        self.log.configure(state='disabled')

        self.node_var.trace_add('write', self._on_node_var_changed)

    def _populate_nodes(self):
        for node_id, x, y in self.node.node_coords:
            item_id = self.node_table.insert('', 'end', values=(node_id, f'{x:.3f}', f'{y:.3f}'))
            self._node_table_items[node_id] = item_id

    def _on_node_var_changed(self, *_args):
        self._refresh_selection_label()
        self._mark_graph_dirty()

    def _on_robot_changed(self, _event=None):
        self._mark_robot_dirty()

    def _on_table_selected(self, _event=None):
        if self._syncing_table_selection:
            return

        selection = self.node_table.selection()
        if not selection:
            return

        values = self.node_table.item(selection[0], 'values')
        selected_node = str(values[0])
        if self.node_var.get() != selected_node:
            self.node_var.set(selected_node)

    def _refresh_selection_label(self):
        try:
            node_id = int(self.node_var.get())
        except ValueError:
            self.selection_var.set('정수 node id를 입력하세요.')
            return

        coords = self.node.node_map.get(node_id)
        if coords is None:
            self.selection_var.set(f'node {node_id}는 현재 그래프에 없습니다.')
            return

        x, y = coords
        self.selection_var.set(f'선택된 목적지: node {node_id}  |  좌표 ({x:.3f}, {y:.3f})')

    def _focus_table_node(self, node_id: int):
        item = self._node_table_items.get(node_id)
        if item is None:
            return

        if self.node_table.selection() != (item,):
            self._syncing_table_selection = True
            try:
                self.node_table.selection_set(item)
            finally:
                self._syncing_table_selection = False

        self.node_table.focus(item)
        self.node_table.see(item)

    def _compute_transform(self):
        if not self.node.node_coords or self.node.graph_bounds is None:
            return None

        width = max(self.graph_canvas.winfo_width(), 200)
        height = max(self.graph_canvas.winfo_height(), 200)
        min_x = self.node.graph_bounds['min_x']
        max_x = self.node.graph_bounds['max_x']
        min_y = self.node.graph_bounds['min_y']
        max_y = self.node.graph_bounds['max_y']

        data_width = max(max_x - min_x, 1.0)
        data_height = max(max_y - min_y, 1.0)
        usable_width = max(width - 2 * self.GRAPH_PADDING, 80)
        usable_height = max(height - 2 * self.GRAPH_PADDING, 80)
        base_scale = min(usable_width / data_width, usable_height / data_height)
        scale = base_scale * self._view_scale

        return {
            'world_center_x': (min_x + max_x) / 2.0,
            'world_center_y': (min_y + max_y) / 2.0,
            'scale': scale,
            'width': width,
            'height': height,
            'center_x': width / 2.0 + self._view_offset_x,
            'center_y': height / 2.0 + self._view_offset_y,
        }

    def _graph_to_canvas(self, x: float, y: float, transform: dict):
        canvas_x = transform['center_x'] + (x - transform['world_center_x']) * transform['scale']
        canvas_y = transform['center_y'] - (y - transform['world_center_y']) * transform['scale']
        return canvas_x, canvas_y

    def _canvas_to_graph(self, canvas_x: float, canvas_y: float, transform: dict):
        if transform['scale'] == 0:
            return None

        x = transform['world_center_x'] + (canvas_x - transform['center_x']) / transform['scale']
        y = transform['world_center_y'] - (canvas_y - transform['center_y']) / transform['scale']
        return x, y

    def _draw_graph_background(self):
        width = max(self.graph_canvas.winfo_width(), 200)
        height = max(self.graph_canvas.winfo_height(), 200)
        self.graph_canvas.delete('all')
        self.graph_canvas.create_rectangle(
            0, 0, width, height,
            fill='#fffdf8',
            outline='',
            tags=('static',),
        )

        grid_gap = 80
        for x in range(0, width, grid_gap):
            self.graph_canvas.create_line(x, 0, x, height, fill='#efe8dc', tags=('static',))
        for y in range(0, height, grid_gap):
            self.graph_canvas.create_line(0, y, width, y, fill='#efe8dc', tags=('static',))

    def _draw_edges(self, transform: dict):
        for start_id, end_id in self.node.edge_pairs:
            x1, y1 = self.node.node_map[start_id]
            x2, y2 = self.node.node_map[end_id]
            cx1, cy1 = self._graph_to_canvas(x1, y1, transform)
            cx2, cy2 = self._graph_to_canvas(x2, y2, transform)
            self.graph_canvas.create_line(
                cx1, cy1, cx2, cy2,
                fill='#b8c1ab',
                width=3,
                tags=('static',),
            )

    def _draw_nodes(self, transform: dict):
        selected_node = None
        try:
            selected_node = int(self.node_var.get())
        except ValueError:
            selected_node = None

        for node_id, x, y in self.node.node_coords:
            cx, cy = self._graph_to_canvas(x, y, transform)
            self._canvas_items[node_id] = (cx, cy)
            is_selected = node_id == selected_node
            radius = self.NODE_RADIUS + (3 if is_selected else 0)

            self.graph_canvas.create_oval(
                cx - radius, cy - radius,
                cx + radius, cy + radius,
                fill='#da6c47' if is_selected else '#335b74',
                outline='#fff7ef',
                width=2,
                tags=('static',),
            )
            self.graph_canvas.create_text(
                cx,
                cy - 16,
                text=str(node_id),
                fill='#21313e' if is_selected else '#475b67',
                font=('TkDefaultFont', 8, 'bold'),
                tags=('static',),
            )

    def _draw_robots(self, transform: dict):
        selected_robot = self.robot_var.get().strip()
        show_all_labels = len(self.node.robot_names) <= 24 or self._view_scale >= 1.6

        for robot_name in self.node.robot_names:
            pose = self.node.robot_poses.get(robot_name)
            if pose is None:
                continue

            cx, cy = self._graph_to_canvas(pose[0], pose[1], transform)
            highlight = robot_name == selected_robot

            self.graph_canvas.create_oval(
                cx - (9 if highlight else 7),
                cy - (9 if highlight else 7),
                cx + (9 if highlight else 7),
                cy + (9 if highlight else 7),
                fill='#f3b33d' if highlight else '#3aa17e',
                outline='#2b3a31',
                width=2,
                tags=('robots',),
            )
            if highlight or show_all_labels:
                self.graph_canvas.create_text(
                    cx,
                    cy + 18,
                    text=robot_name,
                    fill='#294234' if highlight else '#345948',
                    font=('TkDefaultFont', 8, 'bold'),
                    tags=('robots',),
                )

    def _draw_legend(self):
        legend_x = 18
        legend_y = 18
        lines = [
            ('#335b74', 'graph node'),
            ('#da6c47', 'selected node'),
            ('#3aa17e', 'robot'),
            ('#f3b33d', 'selected robot'),
        ]
        box_height = 24 + len(lines) * 18
        self.graph_canvas.create_rectangle(
            legend_x,
            legend_y,
            legend_x + 150,
            legend_y + box_height,
            fill='#fff8ee',
            outline='#e6d8bf',
            tags=('static',),
        )
        self.graph_canvas.create_text(
            legend_x + 12,
            legend_y + 14,
            text='Legend',
            anchor='w',
            fill='#425244',
            font=('TkDefaultFont', 9, 'bold'),
            tags=('static',),
        )
        for idx, (color, label) in enumerate(lines):
            y = legend_y + 32 + idx * 18
            self.graph_canvas.create_oval(
                legend_x + 12, y - 4,
                legend_x + 22, y + 6,
                fill=color,
                outline='',
                tags=('static',),
            )
            self.graph_canvas.create_text(
                legend_x + 30,
                y + 1,
                text=label,
                anchor='w',
                fill='#4e5f52',
                font=('TkDefaultFont', 8),
                tags=('static',),
            )

    def _render_graph(self):
        if not self.root.winfo_exists():
            return

        if not (self._graph_dirty or self._robots_dirty):
            return

        transform = self._compute_transform()

        if self._graph_dirty:
            self._draw_graph_background()
            self._canvas_items = {}

            if transform is not None:
                self._draw_edges(transform)
                self._draw_nodes(transform)
                self._draw_legend()
            else:
                self.graph_canvas.create_text(
                    40, 40,
                    text='graph_file에서 노드를 불러오지 못했습니다.',
                    anchor='nw',
                    fill='#6b5a4d',
                    font=('TkDefaultFont', 10, 'bold'),
                    tags=('static',),
                )

            self._graph_dirty = False
            self._robots_dirty = True

        if self._robots_dirty:
            self.graph_canvas.delete('robots')
            if transform is not None:
                self._draw_robots(transform)
            self._robots_dirty = False

    def _request_render(self):
        if self._render_pending or not self.root.winfo_exists():
            return
        self._render_pending = True
        self.root.after_idle(self._flush_render)

    def _flush_render(self):
        self._render_pending = False
        self._render_graph()

    def _mark_graph_dirty(self):
        self._graph_dirty = True
        self._request_render()

    def _mark_robot_dirty(self):
        self._robots_dirty = True
        self._request_render()

    def _closest_node_from_canvas(self, canvas_x: float, canvas_y: float):
        closest_id = None
        closest_dist_sq = None

        for node_id, (x, y) in self._canvas_items.items():
            dist_sq = (x - canvas_x) ** 2 + (y - canvas_y) ** 2
            if closest_dist_sq is None or dist_sq < closest_dist_sq:
                closest_dist_sq = dist_sq
                closest_id = node_id

        if closest_id is None or closest_dist_sq is None:
            return None

        if closest_dist_sq > (self.NODE_RADIUS + 10) ** 2:
            return None

        return closest_id

    def _select_node_from_canvas(self, canvas_x: float, canvas_y: float):
        node_id = self._closest_node_from_canvas(canvas_x, canvas_y)
        if node_id is None:
            return None

        node_value = str(node_id)
        if self.node_var.get() != node_value:
            self.node_var.set(node_value)
        self._focus_table_node(node_id)
        return node_id

    def _zoom_by(self, factor: float, anchor_x=None, anchor_y=None):
        transform = self._compute_transform()
        if transform is None:
            return

        if anchor_x is None:
            anchor_x = transform['width'] / 2.0
        if anchor_y is None:
            anchor_y = transform['height'] / 2.0

        world_pos = self._canvas_to_graph(anchor_x, anchor_y, transform)
        if world_pos is None:
            return

        new_scale = max(0.35, min(8.0, self._view_scale * factor))
        if abs(new_scale - self._view_scale) < 1e-6:
            return

        self._view_scale = new_scale
        new_transform = self._compute_transform()
        new_canvas_x, new_canvas_y = self._graph_to_canvas(world_pos[0], world_pos[1], new_transform)
        self._view_offset_x += anchor_x - new_canvas_x
        self._view_offset_y += anchor_y - new_canvas_y
        self.zoom_var.set(f'{int(self._view_scale * 100)}%')
        self._mark_graph_dirty()

    def _zoom_in(self):
        self._zoom_by(1.2)

    def _zoom_out(self):
        self._zoom_by(1.0 / 1.2)

    def _reset_view(self):
        self._view_scale = 1.0
        self._view_offset_x = 0.0
        self._view_offset_y = 0.0
        self.zoom_var.set('100%')
        self._mark_graph_dirty()

    def _on_mousewheel(self, event):
        factor = 1.12 if event.delta > 0 else (1.0 / 1.12)
        self._zoom_by(factor, event.x, event.y)

    def _on_mousewheel_linux(self, event):
        if event.num == 4:
            self._zoom_by(1.12, event.x, event.y)
        elif event.num == 5:
            self._zoom_by(1.0 / 1.12, event.x, event.y)

    def _on_canvas_press(self, event):
        self._drag_start = (event.x, event.y)
        self._dragging = False

    def _on_canvas_drag(self, event):
        if self._drag_start is None:
            return

        dx = event.x - self._drag_start[0]
        dy = event.y - self._drag_start[1]

        if not self._dragging and abs(dx) + abs(dy) < 4:
            return

        self._dragging = True
        self._view_offset_x += dx
        self._view_offset_y += dy
        self._drag_start = (event.x, event.y)
        self._mark_graph_dirty()

    def _on_canvas_release(self, event):
        was_dragging = self._dragging
        self._drag_start = None
        self._dragging = False

        if was_dragging:
            return

        self._select_node_from_canvas(event.x, event.y)

    def _on_canvas_configure(self, event):
        size = (event.width, event.height)
        if size == self._last_canvas_size:
            return
        self._last_canvas_size = size
        self._mark_graph_dirty()

    def _append_log(self, line: str):
        self.log.configure(state='normal')
        self.log.insert('end', line + '\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def _format_status(self, payload: dict) -> str:
        level = str(payload.get('level', 'info')).upper()
        robot_name = payload.get('robot_name', '-')
        goal_node_id = payload.get('goal_node_id', '-')
        message = payload.get('message', '')
        return f'[{level}] {robot_name} -> node {goal_node_id} | {message}'

    def _handle_status(self, payload: dict):
        self._append_log(self._format_status(payload))

    def _send_command(self, _event=None):
        robot_name = self.robot_var.get().strip()
        if not robot_name:
            messagebox.showerror('입력 오류', '로봇을 먼저 선택하세요.')
            return

        try:
            goal_node_id = int(self.node_var.get().strip())
        except ValueError:
            messagebox.showerror('입력 오류', 'node id는 정수여야 합니다.')
            return

        if self.node.node_map and goal_node_id not in self.node.node_map:
            messagebox.showerror('입력 오류', f'node {goal_node_id}가 그래프에 없습니다.')
            return

        self.node.send_command(robot_name, goal_node_id)

        coords = self.node.node_map.get(goal_node_id)
        if coords is None:
            self._append_log(f'[LOCAL] {robot_name} -> node {goal_node_id} 전송')
        else:
            self._append_log(
                f'[LOCAL] {robot_name} -> node {goal_node_id} 전송 '
                f'(좌표 {coords[0]:.3f}, {coords[1]:.3f})'
            )

    def _poll_ros(self):
        if not self.root.winfo_exists():
            return
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if self.node.pose_version != self._last_pose_version:
                self._last_pose_version = self.node.pose_version
                self._mark_robot_dirty()
            self.root.after(self.ROS_POLL_MS, self._poll_ros)

    def _on_close(self):
        self.root.destroy()

    def run(self):
        self._poll_ros()
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = FleetCommandGuiNode()
    app = FleetCommandGuiApp(node)

    try:
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

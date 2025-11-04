import os
import yaml
import pathlib
from typing import List, Dict, Optional

def save_wav_from_uint8(raw: bytes | bytearray | list[int], out_path: str | pathlib.Path):
    """
    raw : 原始 wav 文件的全部字节（uint8 数组 / bytes / bytearray）
    out_path : 要保存的 wav 文件名
    """
    # 如果给的是 list[int] 或 np.ndarray，先转 bytes
    if isinstance(raw, (list, tuple)):
        raw = bytes(raw)
    elif hasattr(raw, 'tobytes'):   # np.ndarray
        raw = raw.tobytes()

    with open(out_path, 'wb') as f:
        f.write(raw)

def check_file(file_path):
    # 检查文件是否存在，并删除
    if os.path.exists(file_path):
        os.remove(file_path)

def write_waypoints(node, req, out_file):
    pts = []
    for waypt in req.waypoints:
        wav_path = ''
        if waypt.voice.has_audio_file:
            # node.get_logger().info(f"audio_file_name: {waypt.voice.audio_file_name}, audio_file_format: {waypt.voice.audio_file_format}")
            wav_path = f'/tmp/{waypt.voice.audio_file_name}'
            save_wav_from_uint8(waypt.voice.audio_file_data, wav_path)
            node.get_logger().info(f'voice_id {waypt.voice.voice_id} 已写入 {wav_path}')
        if waypt.waypoint_id == '':
            node.get_logger().warn('waypoint_id 为空！')
        pt = {
            'id': waypt.waypoint_id,
            'x': waypt.x,
            'y': waypt.y,
            'yaw': waypt.yaw,
            'action': {'command': waypt.action.command, 'play_delay': waypt.action.play_delay},
            'voice': {'voice_id': waypt.voice.voice_id, 'voice_content': waypt.voice.voice_content,
                      'play_delay': waypt.voice.play_delay, 'play_duration': waypt.voice.play_duration,
                      'file_path': wav_path}
        }
        pts.append(pt)

    data = {
        'request_id': req.request_id,
        'api_id': req.api_id,
        'route_id': req.route_id,
        'route_name': req.route_name,
        'waypoints': pts,
    }
    check_file(out_file)
    with open(out_file, 'w', encoding='utf-8') as file:
        yaml.dump(data, file, allow_unicode=True, default_flow_style=False)
        node.get_logger().info(f'waypoints 已写入 {out_file}')

def write_waypoints_status(waypoints_file: str, waypoints_status: dict):
    if not os.path.exists(waypoints_file):
        raise FileNotFoundError(f'Waypoints file does not exist: {waypoints_file}')
    
    with open(waypoints_file, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    
    waypoints = data.get('waypoints', [])
    for wp in waypoints:
        wp_id = wp.get('id')
        if wp_id is not None:
            wp['status'] = waypoints_status[wp_id]
    
    with open(waypoints_file, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, allow_unicode=True, default_flow_style=False)

def load_waypoints(waypoints_file: str, node) -> Optional[List[Dict]]:
    if not waypoints_file:
        node.get_logger().error('waypoints_file 参数为空！')
        return None
    if not os.path.exists(waypoints_file):
        node.get_logger().error(f'waypoints_file 文件不存在：{waypoints_file}')
        return None
    node.get_logger().info(f'load waypoints_file: {waypoints_file}')
    try:
        with open(waypoints_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)['waypoints']
    except Exception as e:
        node.get_logger().error(f'加载 waypoints_file 出错：{type(e).__name__} - {str(e)}')
        return None

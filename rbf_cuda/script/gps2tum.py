#!/usr/bin/env python3
import rosbag
from pyproj import CRS, Transformer
from tqdm import tqdm

bag_path = "/root/dataset/mountain.bag"
output_file = "/root/code/ws_lio/result/mountain/gps_trajectory.tum"
topic = "/ublox_driver/receiver_lla"

origin_lat = None
origin_lon = None
origin_alt = None

transformer_lla2ecef = Transformer.from_crs("epsg:4979", "epsg:4978", always_xy=True)

def lla_to_ecef(lat, lon, alt):
    x, y, z = transformer_lla2ecef.transform(lon, lat, alt)
    return x, y, z

def ecef_to_enu(x, y, z, x0, y0, z0, lat0, lon0):
    import math
    # 转换矩阵
    lam = math.radians(lon0)
    phi = math.radians(lat0)

    t = [
        [-math.sin(lam), math.cos(lam), 0],
        [-math.sin(phi)*math.cos(lam), -math.sin(phi)*math.sin(lam), math.cos(phi)],
        [math.cos(phi)*math.cos(lam), math.cos(phi)*math.sin(lam), math.sin(phi)]
    ]

    dx = x - x0
    dy = y - y0
    dz = z - z0

    enu = [
        t[0][0]*dx + t[0][1]*dy + t[0][2]*dz,
        t[1][0]*dx + t[1][1]*dy + t[1][2]*dz,
        t[2][0]*dx + t[2][1]*dy + t[2][2]*dz
    ]
    return enu

msg_count = 0

with rosbag.Bag(bag_path, 'r') as bag, open(output_file, 'w') as f_out:
    for topic_name, msg, t in tqdm(bag.read_messages(topics=[topic])):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        x, y, z = lla_to_ecef(lat, lon, alt)

        if origin_lat is None:
            origin_lat = lat
            origin_lon = lon
            origin_alt = alt
            x0, y0, z0 = lla_to_ecef(lat, lon, alt)

        enu = ecef_to_enu(x, y, z, x0, y0, z0, origin_lat, origin_lon)
        print(enu)

        # timestamp = msg.header.stamp.to_sec()
        timestamp = t.to_sec()
        f_out.write(f"{timestamp:.6f} {enu[0]:.6f} {enu[1]:.6f} {enu[2]:.6f} 0 0 0 1\n")
        msg_count += 1

print(f"输出完成，总共 {msg_count} 条数据 -> {output_file}")

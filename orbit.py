#!/usr/bin/env python3
"""
Orbital Mechanics & Link Budget for LEO Constellation
======================================================
物理驱动的卫星轨道模型和链路预算计算。

轨道模型:
  - 圆形轨道 (LEO 近似足够)
  - Walker Delta 星座
  - 地心惯性坐标系 (ECI)

链路模型:
  - 星间链路 (ISL): 自由空间光通信, 带宽 ∝ 1/d²
  - 星地链路 (SGL): RF, 考虑仰角和大气衰减
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional

# ════════════════════════════════════════════════
#  物理常数
# ════════════════════════════════════════════════

EARTH_RADIUS_KM = 6371.0
EARTH_MU = 398600.4418        # km³/s²  (标准引力参数)
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


# ════════════════════════════════════════════════
#  轨道计算
# ════════════════════════════════════════════════

@dataclass
class OrbitalElements:
    """轨道根数 (Keplerian)"""
    semi_major_axis_km: float    # a (km), = R_earth + altitude
    eccentricity: float = 0.0    # e, 圆形=0
    inclination_deg: float = 53.0  # i (度), 典型 LEO=53°
    raan_deg: float = 0.0        # Ω (度), 升交点赤经
    arg_perigee_deg: float = 0.0 # ω (度)
    mean_anomaly_deg: float = 0.0 # M₀ (度), 初始平近点角

    @property
    def period_seconds(self) -> float:
        """轨道周期 (秒), T = 2π√(a³/μ)"""
        return 2.0 * math.pi * math.sqrt(self.semi_major_axis_km**3 / EARTH_MU)

    @property
    def mean_motion_rad_per_s(self) -> float:
        """平均角速度 (rad/s), n = √(μ/a³)"""
        return math.sqrt(EARTH_MU / self.semi_major_axis_km**3)


def orbital_position(oe: OrbitalElements, t_seconds: float) -> np.ndarray:
    """
    计算卫星在 ECI 坐标系中的位置 (km)。

    步骤:
      1. 计算当前时刻的平近点角 M = M₀ + n·t
      2. 圆形轨道: 真近点角 ν = M
      3. 在轨道平面中: x_orb = a·cos(ν), y_orb = a·sin(ν)
      4. 旋转到 ECI: 依次绕 Z(Ω), X(i), Z(ω) 旋转
    """
    n = oe.mean_motion_rad_per_s
    M0 = oe.mean_anomaly_deg * DEG2RAD
    M = M0 + n * t_seconds

    # 圆形轨道: 真近点角 = 平近点角
    nu = M
    a = oe.semi_major_axis_km

    # 轨道平面坐标
    x_orb = a * math.cos(nu)
    y_orb = a * math.sin(nu)
    z_orb = 0.0

    # 旋转矩阵 (3-1-3 Euler: Ω, i, ω)
    Omega = oe.raan_deg * DEG2RAD
    i = oe.inclination_deg * DEG2RAD
    omega = oe.arg_perigee_deg * DEG2RAD

    # R_z(Omega)
    x1 = x_orb * math.cos(Omega) - y_orb * math.sin(Omega)
    y1 = x_orb * math.sin(Omega) + y_orb * math.cos(Omega)
    z1 = z_orb

    # R_x(i)
    x2 = x1
    y2 = y1 * math.cos(i) - z1 * math.sin(i)
    z2 = y1 * math.sin(i) + z1 * math.cos(i)

    # R_z(omega)
    x3 = x2 * math.cos(omega) - y2 * math.sin(omega)
    y3 = x2 * math.sin(omega) + y2 * math.cos(omega)
    z3 = z2

    return np.array([x3, y3, z3])


def distance_km(pos1: np.ndarray, pos2: np.ndarray) -> float:
    """两点间欧氏距离 (km)"""
    return float(np.linalg.norm(pos1 - pos2))


# ════════════════════════════════════════════════
#  Walker Delta 星座
# ════════════════════════════════════════════════

@dataclass
class Satellite:
    sat_id: str
    oe: OrbitalElements
    plane_id: int
    slot_id: int


def walker_constellation(
    n_planes: int,
    sats_per_plane: int,
    altitude_km: float,
    inclination_deg: float,
    phasing: int = 1,
    prefix: str = "SAT",
) -> List[Satellite]:
    """
    Walker Delta 星座。

    参数:
      n_planes:       轨道面数量
      sats_per_plane: 每个轨道面的卫星数
      altitude_km:    轨道高度 (km)
      inclination_deg: 轨道倾角 (度)
      phasing:        相位因子 F (0..N-1)
      prefix:         卫星 ID 前缀

    返回卫星列表。
    """
    satellites = []
    a = EARTH_RADIUS_KM + altitude_km

    # 每个轨道面之间的 RAAN 间隔
    delta_raan = 360.0 / n_planes

    # 同一轨道面内卫星之间的平近点角间隔
    delta_ma = 360.0 / sats_per_plane

    # 相邻轨道面之间的相位偏移
    phase_offset = phasing * 360.0 / (n_planes * sats_per_plane)

    for p in range(n_planes):
        raan = p * delta_raan
        for s in range(sats_per_plane):
            # 初始平近点角: s·ΔM + p·phase_offset
            ma0 = (s * delta_ma + p * phase_offset) % 360.0
            sat_id = f"{prefix}-{p}{s}"
            oe = OrbitalElements(
                semi_major_axis_km=a,
                eccentricity=0.0,
                inclination_deg=inclination_deg,
                raan_deg=raan,
                mean_anomaly_deg=ma0,
            )
            satellites.append(Satellite(sat_id, oe, p, s))
    return satellites


# ════════════════════════════════════════════════
#  链路预算
# ════════════════════════════════════════════════

@dataclass
class LinkBudget:
    """自由空间光通信 (ISL) 链路预算"""
    tx_power_watts: float = 1.0          # 发射功率 (W)
    tx_gain_db: float = 90.0             # 发射天线增益 (dB)
    rx_gain_db: float = 90.0             # 接收天线增益 (dB)
    wavelength_nm: float = 1550.0        # 波长 (nm), 典型 1550nm
    data_rate_per_watt_mbps: float = 1000.0  # 每瓦特的数据率 (简化模型)
    min_elevation_deg: float = 10.0      # 最小仰角 (ISL)
    max_range_km: float = 5000.0         # 最大通信距离 (km)


def isl_bandwidth_mbps(distance_km: float, budget: LinkBudget = LinkBudget()) -> float:
    """
    自由空间光 ISL 带宽 (Mbps)。

    简化模型: 带宽 ∝ 1/d², 在参考距离处校准。
    实际 FSO 终端有固定的最大数据率，超出最大距离后断链。
    """
    if distance_km <= 0:
        return 0.0
    if distance_km > budget.max_range_km:
        return 0.0

    # 自由空间路径损耗 ∝ (λ/(4πd))²
    # 接收功率 ∝ 1/d²
    # 简化: bandwidth = k / d²
    ref_distance = 1000.0  # 1000km 参考距离
    ref_bandwidth = 1000.0  # 参考带宽 1 Gbps
    bw = ref_bandwidth * (ref_distance / distance_km) ** 2

    return round(bw, 1)


def sgl_bandwidth_mbps(distance_km: float, elevation_deg: float,
                       budget: LinkBudget = LinkBudget()) -> float:
    """
    星地链路 (SGL) 带宽 (Mbps)。

    受自由空间损耗 + 大气衰减影响。
    低仰角时大气路径更长 = 更多衰减。
    """
    if distance_km <= 0 or elevation_deg < budget.min_elevation_deg:
        return 0.0
    if distance_km > budget.max_range_km:
        return 0.0

    # 自由空间部分
    ref_distance = 1000.0
    ref_bandwidth = 500.0   # SGL 通常比 ISL 带宽低
    fs_bw = ref_bandwidth * (ref_distance / distance_km) ** 2

    # 大气衰减因子 (简化: 低仰角衰减大)
    atm_factor = 1.0 - math.exp(-elevation_deg / 30.0)
    if atm_factor < 0:
        atm_factor = 0

    return round(fs_bw * atm_factor, 1)


def elevation_angle_deg(sat_pos: np.ndarray, gs_pos: np.ndarray) -> float:
    """
    计算地面站看卫星的仰角 (度)。

    gs_pos 是地心地固坐标系 (ECEF) 中的地面站位置 (km)。
    简化: gs_pos 的长度 = R_earth, sat_pos 的长度 = R_earth + altitude
    """
    # 地心角
    cos_theta = np.dot(sat_pos, gs_pos) / (np.linalg.norm(sat_pos) * np.linalg.norm(gs_pos))
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)  # 地心角 (rad)

    r_earth = np.linalg.norm(gs_pos)
    r_sat = np.linalg.norm(sat_pos)

    # 仰角计算
    # tan(el) = (cos(theta) - r_earth/r_sat) / sin(theta)
    sin_theta = math.sin(theta)
    if sin_theta < 1e-10:
        return 90.0

    tan_el = (cos_theta - r_earth / r_sat) / sin_theta
    el_rad = math.atan(tan_el)
    el_deg = el_rad * RAD2DEG

    return max(0.0, el_deg)


# ════════════════════════════════════════════════
#  链路发现
# ════════════════════════════════════════════════

def discover_links(
    satellites: List[Satellite],
    ground_stations: List[Tuple[str, np.ndarray]],  # [(id, ecef_pos_km)]
    t_seconds: float,
    isl_budget: LinkBudget = LinkBudget(),
    sgl_budget: LinkBudget = LinkBudget(),
) -> dict:
    """
    发现所有可行的星间链路和星地链路。

    返回:
      {
        "SAT-A": {"SAT-B": bw_mbps, ...},    # ISL
        "SAT-X": {"GS-Y": bw_mbps, ...},      # SGL
      }
    """
    # 计算所有卫星位置
    sat_positions = {}
    for sat in satellites:
        sat_positions[sat.sat_id] = orbital_position(sat.oe, t_seconds)

    topology = {sat.sat_id: {} for sat in satellites}
    for gs_id, _ in ground_stations:
        topology[gs_id] = {}

    # 星间链路
    for i, sat_a in enumerate(satellites):
        for sat_b in satellites[i+1:]:
            d = distance_km(sat_positions[sat_a.sat_id],
                           sat_positions[sat_b.sat_id])
            bw = isl_bandwidth_mbps(d, isl_budget)
            if bw > 0:
                topology[sat_a.sat_id][sat_b.sat_id] = bw
                topology[sat_b.sat_id][sat_a.sat_id] = bw

    # 星地链路
    for sat in satellites:
        sat_pos = sat_positions[sat.sat_id]
        for gs_id, gs_pos in ground_stations:
            d = distance_km(sat_pos, gs_pos)
            el = elevation_angle_deg(sat_pos, gs_pos)
            bw = sgl_bandwidth_mbps(d, el, sgl_budget)
            if bw > 0:
                topology[sat.sat_id][gs_id] = bw
                topology[gs_id][sat.sat_id] = bw

    return topology


# ════════════════════════════════════════════════
#  地面站位置
# ════════════════════════════════════════════════

def ground_station_ecef(lat_deg: float, lon_deg: float, alt_km: float = 0.0) -> np.ndarray:
    """地理坐标 → ECEF (km)"""
    lat = lat_deg * DEG2RAD
    lon = lon_deg * DEG2RAD
    r = EARTH_RADIUS_KM + alt_km
    x = r * math.cos(lat) * math.cos(lon)
    y = r * math.cos(lat) * math.sin(lon)
    z = r * math.sin(lat)
    return np.array([x, y, z])

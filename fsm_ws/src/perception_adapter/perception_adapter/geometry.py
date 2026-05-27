from __future__ import annotations

import math


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


def cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        float(a[1] * b[2] - a[2] * b[1]),
        float(a[2] * b[0] - a[0] * b[2]),
        float(a[0] * b[1] - a[1] * b[0]),
    )


def vector_length(value: tuple[float, float, float]) -> float:
    return math.sqrt(float(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]))


def normalize(value: tuple[float, float, float]) -> tuple[float, float, float]:
    length = vector_length(value)
    if length <= 1e-9:
        return (1.0, 0.0, 0.0)
    return (float(value[0] / length), float(value[1] / length), float(value[2] / length))


def normalize_quaternion(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    length = math.sqrt(float(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]))
    if length <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (float(q[0] / length), float(q[1] / length), float(q[2] / length), float(q[3] / length))


def quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q: tuple[float, float, float, float], value: tuple[float, float, float]) -> tuple[float, float, float]:
    q = normalize_quaternion(q)
    vector_q = (float(value[0]), float(value[1]), float(value[2]), 0.0)
    q_conj = (-q[0], -q[1], -q[2], q[3])
    rotated = quat_multiply(quat_multiply(q, vector_q), q_conj)
    return (float(rotated[0]), float(rotated[1]), float(rotated[2]))


def quaternion_from_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
    m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
    m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    return normalize_quaternion(q)

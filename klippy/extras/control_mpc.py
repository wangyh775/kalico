import logging
import math
import time

import numpy as np

try:
    from numba import jit, prange

    NUMBA_AVAILABLE = True
except ImportError:
    NUMBA_AVAILABLE = False

    def jit(*args, **kwargs):
        def decorator(func):
            return func

        if len(args) == 1 and callable(args[0]):
            return args[0]
        return decorator

    prange = range

AMBIENT_TEMP = 25.0
PIN_MIN_TIME = 0.100

FILAMENT_TEMP_SRC_AMBIENT = "ambient"
FILAMENT_TEMP_SRC_FIXED = "fixed"
FILAMENT_TEMP_SRC_SENSOR = "sensor"


@jit(nopython=True, cache=True, fastmath=True)
def _numba_model_step(
    T_h,
    T_b,
    T_s,
    power,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
):
    """
    Numba加速的单步模型仿真

    使用前向欧拉法计算一个时间步长后的温度变化
    """
    T_env_K = T_env + 273.15
    T_b_K = T_b + 273.15

    dT_h = theta_8 * power - theta_1 * (T_h - T_b)

    q_hb = theta_2 * (T_h - T_b)
    q_bs = theta_3 * (T_b - T_s)
    P_conv = theta_5 * (T_b - T_env)
    P_cond = theta_6 * (T_b - T_cold)
    P_rad = theta_7 * (T_b_K**4 - T_env_K**4)
    P_extrusion = v_f * c_p * theta_9 * (T_b - T_filament)

    dT_b = q_hb - q_bs - P_conv - P_cond - P_rad - P_extrusion
    dT_s = theta_4 * (T_b - T_s)

    T_h_new = T_h + dT_h * dt
    T_b_new = T_b + dT_b * dt
    T_s_new = T_s + dT_s * dt

    if T_h_new < -50.0:
        T_h_new = -50.0
    elif T_h_new > 600.0:
        T_h_new = 600.0

    if T_b_new < -50.0:
        T_b_new = -50.0
    elif T_b_new > 600.0:
        T_b_new = 600.0

    if T_s_new < -50.0:
        T_s_new = -50.0
    elif T_s_new > 600.0:
        T_s_new = 600.0

    return T_h_new, T_b_new, T_s_new


@jit(nopython=True, cache=True, fastmath=True)
def _numba_predict_trajectory(
    T_h_init,
    T_b_init,
    T_s_init,
    power_sequence,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
):
    """
    Numba加速的多步预测

    迭代计算未来N步的温度变化
    """
    n_steps = len(power_sequence)
    T_h_arr = np.zeros(n_steps + 1)
    T_b_arr = np.zeros(n_steps + 1)
    T_s_arr = np.zeros(n_steps + 1)

    T_h = T_h_init
    T_b = T_b_init
    T_s = T_s_init

    T_h_arr[0] = T_h
    T_b_arr[0] = T_b
    T_s_arr[0] = T_s

    for i in range(n_steps):
        T_h, T_b, T_s = _numba_model_step(
            T_h,
            T_b,
            T_s,
            power_sequence[i],
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
        )
        T_h_arr[i + 1] = T_h
        T_b_arr[i + 1] = T_b
        T_s_arr[i + 1] = T_s

    return T_h_arr, T_b_arr, T_s_arr


@jit(nopython=True, cache=True, fastmath=True)
def _numba_mpc_objective(
    u,
    T_h_init,
    T_b_init,
    T_s_init,
    setpoint,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
    Np,
    Nc,
    w_t,
    w_terminal,
    w_r,
    last_control,
):
    """
    Numba加速的MPC目标函数计算

    目标函数:
        J = Σ w_track * (T_s[k] - T_set)² + w_terminal * (T_s[Np] - T_set)²
            + w_rate * (u[0] - u_last)² + Σ w_rate * (u[k] - u[k-1])²

    参数:
        u: 控制序列 (长度Nc)
        T_h_init, T_b_init, T_s_init: 初始状态
        setpoint: 目标温度
        w_t: 跟踪权重
        w_terminal: 终端权重
        w_r: 变化率权重
        last_control: 上一次控制量

    返回:
        目标函数值
    """
    u_full = np.zeros(Np)
    for i in range(Np):
        idx = i
        if idx > Nc - 1:
            idx = Nc - 1
        u_full[i] = u[idx]

    _, _, T_s_pred = _numba_predict_trajectory(
        T_h_init,
        T_b_init,
        T_s_init,
        u_full,
        dt,
        T_env,
        T_cold,
        v_f,
        T_filament,
        theta_1,
        theta_2,
        theta_3,
        theta_4,
        theta_5,
        theta_6,
        theta_7,
        theta_8,
        theta_9,
        c_p,
    )

    tracking_error = 0.0
    for i in range(1, Np + 1):
        tracking_error += w_t * (T_s_pred[i] - setpoint) ** 2

    terminal_cost = w_terminal * (T_s_pred[Np] - setpoint) ** 2

    rate_penalty = w_r * (u[0] - last_control) ** 2
    if Nc > 1:
        for i in range(1, Nc):
            rate_penalty += w_r * (u[i] - u[i - 1]) ** 2

    return tracking_error + terminal_cost + rate_penalty


# =============================================================================
# EKF (扩展卡尔曼滤波器) - Numba优化实现
# Extended Kalman Filter - Numba Optimized Implementation
# =============================================================================


@jit(nopython=True, cache=True, fastmath=False)
def _ekf_model_jacobian(
    T_h,
    T_b,
    T_s,
    power,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
):
    """
    计算EKF状态转移矩阵的雅可比矩阵

    状态向量: x = [T_h, T_b, T_s]
    观测向量: z = [T_s] (仅观测传感器温度)

    雅可比矩阵 F = df/dx，用于EKF的状态预测协方差更新
    """
    T_env_K = T_env + 273.15
    T_b_K = T_b + 273.15

    F = np.zeros((3, 3))

    F[0, 0] = 1.0 - theta_1 * dt
    F[0, 1] = theta_1 * dt
    F[0, 2] = 0.0

    F[1, 0] = theta_2 * dt
    F[1, 1] = (
        1.0
        - (
            theta_3
            + theta_5
            + theta_6
            + 4.0 * theta_7 * T_b_K**3
            + v_f * c_p * theta_9
        )
        * dt
    )
    F[1, 2] = -theta_3 * dt

    F[2, 0] = 0.0
    F[2, 1] = theta_4 * dt
    F[2, 2] = 1.0 - theta_4 * dt

    return F


@jit(nopython=True, cache=True, fastmath=False)
def _ekf_predict(
    x,
    P,
    power,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    Q,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
):
    """
    EKF预测步骤

    使用模型预测状态和协方差矩阵

    参数:
        x: 状态向量 [T_h, T_b, T_s]
        P: 状态协方差矩阵 (3x3)
        power: 输入功率 (W)
        dt: 时间步长 (s)
        T_env, T_cold, v_f, T_filament: 环境参数
        Q: 过程噪声协方差矩阵 (3x3)
        theta_*: 模型参数
        c_p: 耗材热容

    返回:
        x_pred: 预测状态
        P_pred: 预测协方差
    """
    T_h, T_b, T_s = x[0], x[1], x[2]

    T_h_new, T_b_new, T_s_new = _numba_model_step(
        T_h,
        T_b,
        T_s,
        power,
        dt,
        T_env,
        T_cold,
        v_f,
        T_filament,
        theta_1,
        theta_2,
        theta_3,
        theta_4,
        theta_5,
        theta_6,
        theta_7,
        theta_8,
        theta_9,
        c_p,
    )

    x_pred = np.array([T_h_new, T_b_new, T_s_new])

    F = _ekf_model_jacobian(
        T_h,
        T_b,
        T_s,
        power,
        dt,
        T_env,
        T_cold,
        v_f,
        T_filament,
        theta_1,
        theta_2,
        theta_3,
        theta_4,
        theta_5,
        theta_6,
        theta_7,
        theta_8,
        theta_9,
        c_p,
    )

    P_pred = F @ P @ F.T + Q

    return x_pred, P_pred


@jit(nopython=True, cache=True, fastmath=False)
def _ekf_update(x_pred, P_pred, z_meas, R):
    """
    EKF更新步骤

    使用观测值校正状态估计

    参数:
        x_pred: 预测状态 [T_h, T_b, T_s]
        P_pred: 预测协方差 (3x3)
        z_meas: 观测值 (仅T_s)
        R: 观测噪声方差

    返回:
        x_upd: 更新后的状态
        P_upd: 更新后的协方差
    """
    H = np.zeros((1, 3))
    H[0, 2] = 1.0

    z_pred = x_pred[2]
    y = z_meas - z_pred

    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T / S[0, 0]

    x_upd = x_pred + K.flatten() * y

    I = np.eye(3)
    P_upd = (I - K @ H) @ P_pred

    for i in range(3):
        x_upd[i] = max(-50.0, min(600.0, x_upd[i]))

    return x_upd, P_upd


@jit(nopython=True, cache=True, fastmath=False)
def _ekf_step(
    x,
    P,
    z_meas,
    power,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    Q,
    R,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
):
    """
    完整的EKF步骤：预测 + 更新

    参数:
        x: 当前状态估计 [T_h, T_b, T_s]
        P: 当前协方差矩阵 (3x3)
        z_meas: 传感器温度观测值
        power: 输入功率
        dt: 时间步长
        其他参数: 模型和噪声参数

    返回:
        x_new: 更新后的状态估计
        P_new: 更新后的协方差矩阵
    """
    x_pred, P_pred = _ekf_predict(
        x,
        P,
        power,
        dt,
        T_env,
        T_cold,
        v_f,
        T_filament,
        Q,
        theta_1,
        theta_2,
        theta_3,
        theta_4,
        theta_5,
        theta_6,
        theta_7,
        theta_8,
        theta_9,
        c_p,
    )

    x_new, P_new = _ekf_update(x_pred, P_pred, z_meas, R)

    return x_new, P_new


# =============================================================================
# 投影梯度下降法 MPC求解器 - Numba优化实现
# Projected Gradient Descent MPC Solver - Numba Optimized Implementation
# =============================================================================


@jit(nopython=True, cache=True, fastmath=True)
def _compute_jacobian_list(
    T_h_init,
    T_b_init,
    T_s_init,
    u,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
    Np,
):
    F_list = np.zeros((Np, 3, 3))
    T_h = T_h_init
    T_b = T_b_init
    T_s = T_s_init
    for k in range(Np):
        F = _ekf_model_jacobian(
            T_h,
            T_b,
            T_s,
            u[min(k, len(u) - 1)],
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
        )
        F_list[k] = F
        T_h, T_b, T_s = _numba_model_step(
            T_h,
            T_b,
            T_s,
            u[min(k, len(u) - 1)],
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
        )
    return F_list


@jit(nopython=True, cache=True, fastmath=True)
def _adjoint_backward_pass(T_s_pred, setpoint, F_list, Np, w_t, w_terminal):
    lambda_list = np.zeros((Np + 1, 3))
    error_Np = T_s_pred[Np] - setpoint
    lambda_list[Np, 2] = 2.0 * (w_t + w_terminal) * error_Np
    for k in range(Np - 1, -1, -1):
        F_k = F_list[k]
        lambda_next = np.array(
            [
                lambda_list[k + 1, 0],
                lambda_list[k + 1, 1],
                lambda_list[k + 1, 2],
            ]
        )
        dJ_dx = np.zeros(3)
        error = T_s_pred[k + 1] - setpoint
        dJ_dx[2] = 2.0 * w_t * error
        lambda_curr = np.array(
            [
                F_k[0, 0] * lambda_next[0]
                + F_k[1, 0] * lambda_next[1]
                + F_k[2, 0] * lambda_next[2]
                + dJ_dx[0],
                F_k[0, 1] * lambda_next[0]
                + F_k[1, 1] * lambda_next[1]
                + F_k[2, 1] * lambda_next[2]
                + dJ_dx[1],
                F_k[0, 2] * lambda_next[0]
                + F_k[1, 2] * lambda_next[1]
                + F_k[2, 2] * lambda_next[2]
                + dJ_dx[2],
            ]
        )
        lambda_list[k] = lambda_curr
    return lambda_list


@jit(nopython=True, cache=True, fastmath=True)
def _compute_gradient_adjoint(
    T_h,
    T_b,
    T_s,
    u,
    setpoint,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
    Np,
    Nc,
    w_t,
    w_terminal,
    w_r,
    last_control,
):
    T_h_pred = np.zeros(Np + 1)
    T_b_pred = np.zeros(Np + 1)
    T_s_pred = np.zeros(Np + 1)
    T_h_pred[0] = T_h
    T_b_pred[0] = T_b
    T_s_pred[0] = T_s
    for k in range(Np):
        T_h, T_b, T_s = _numba_model_step(
            T_h,
            T_b,
            T_s,
            u[min(k, len(u) - 1)],
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
        )
        T_h_pred[k + 1] = T_h
        T_b_pred[k + 1] = T_b
        T_s_pred[k + 1] = T_s
    F_list = _compute_jacobian_list(
        T_h_pred[0],
        T_b_pred[0],
        T_s_pred[0],
        u,
        dt,
        T_env,
        T_cold,
        v_f,
        T_filament,
        theta_1,
        theta_2,
        theta_3,
        theta_4,
        theta_5,
        theta_6,
        theta_7,
        theta_8,
        theta_9,
        c_p,
        Np,
    )
    lambda_list = _adjoint_backward_pass(
        T_s_pred, setpoint, F_list, Np, w_t, w_terminal
    )
    grad = np.zeros(Nc)
    for j in range(Nc):
        dT_s_dP = np.zeros(Np + 1)
        dT_s_dP[0] = 0.0
        dT_h = 0.0
        dT_b = 0.0
        dT_s = 0.0
        for k in range(Np):
            dT_h_new = (
                theta_1 * dt * (dT_b - dT_h)
                + (1.0 - theta_1 * dt) * dT_h
                + dt * theta_1
            )
            dT_b_new = (
                theta_2 * dt * dT_h
                + (1.0 - (theta_3 + theta_5 + theta_6) * dt) * dT_b
                - theta_3 * dt * dT_s
            )
            dT_s_new = theta_4 * dt * dT_b + (1.0 - theta_4 * dt) * dT_s
            if k >= j:
                dT_h_new += dt * theta_1
            dT_s_dP[k + 1] = dT_s_new
            dT_h = dT_h_new
            dT_b = dT_b_new
            dT_s = dT_s_new
        grad[j] = np.sum(lambda_list[:, 2] * dT_s_dP)
    return grad


@jit(nopython=True, cache=True, fastmath=True)
def _compute_gradient(
    T_h,
    T_b,
    T_s,
    u,
    setpoint,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
    Np,
    Nc,
    w_t,
    w_terminal,
    w_r,
    last_control,
):
    return _compute_gradient_adjoint(
        T_h,
        T_b,
        T_s,
        u,
        setpoint,
        dt,
        T_env,
        T_cold,
        v_f,
        T_filament,
        theta_1,
        theta_2,
        theta_3,
        theta_4,
        theta_5,
        theta_6,
        theta_7,
        theta_8,
        theta_9,
        c_p,
        Np,
        Nc,
        w_t,
        w_terminal,
        w_r,
        last_control,
    )


@jit(nopython=True, cache=True, fastmath=True)
def _project_to_bounds(u, min_power, max_power):
    """
    将控制序列投影到可行域

    参数:
        u: 控制序列
        min_power: 最小功率
        max_power: 最大功率

    返回:
        u_proj: 投影后的控制序列
    """
    u_proj = u.copy()
    for i in range(len(u_proj)):
        if u_proj[i] < min_power:
            u_proj[i] = min_power
        elif u_proj[i] > max_power:
            u_proj[i] = max_power
    return u_proj


@jit(nopython=True, cache=True, fastmath=True)
def _line_search(
    T_h,
    T_b,
    T_s,
    u,
    grad,
    setpoint,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
    Np,
    Nc,
    w_t,
    w_terminal,
    w_r,
    last_control,
    alpha_init,
    rho,
    c1,
):
    """
    回溯线搜索

    使用Armijo条件确定步长

    参数:
        u: 当前控制序列
        grad: 梯度方向
        alpha_init: 初始步长
        rho: 步长衰减因子
        c1: Armijo条件参数

    返回:
        alpha: 满足条件的步长
    """
    alpha = alpha_init

    J0 = _numba_mpc_objective(
        u,
        T_h,
        T_b,
        T_s,
        setpoint,
        dt,
        T_env,
        T_cold,
        v_f,
        T_filament,
        theta_1,
        theta_2,
        theta_3,
        theta_4,
        theta_5,
        theta_6,
        theta_7,
        theta_8,
        theta_9,
        c_p,
        Np,
        Nc,
        w_t,
        w_terminal,
        w_r,
        last_control,
    )

    grad_norm_sq = 0.0
    for i in range(Nc):
        grad_norm_sq += grad[i] * grad[i]

    max_line_search_iter = 20
    for _ in range(max_line_search_iter):
        u_new = u - alpha * grad

        J_new = _numba_mpc_objective(
            u_new,
            T_h,
            T_b,
            T_s,
            setpoint,
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
            Np,
            Nc,
            w_t,
            w_terminal,
            w_r,
            last_control,
        )

        if J_new <= J0 - c1 * alpha * grad_norm_sq:
            return alpha

        alpha *= rho

    return alpha


@jit(nopython=True, cache=True, fastmath=True)
def _solve_mpc_pgd(
    T_h,
    T_b,
    T_s,
    setpoint,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
    Np,
    Nc,
    w_t,
    w_terminal,
    w_r,
    last_control,
    max_power,
    min_power,
    max_iter,
    tol,
    u_init,
):
    """
    投影梯度下降法求解MPC问题

    使用回溯线搜索和投影操作求解带约束的MPC优化问题
    结合最优解追踪策略，确保返回历史最优解

    参数:
        T_h, T_b, T_s: 初始状态
        setpoint: 目标温度
        max_power, min_power: 功率约束
        max_iter: 最大迭代次数
        tol: 收敛容差
        u_init: 初始控制序列

    返回:
        u_opt: 最优控制序列
        converged: 是否收敛
        iterations: 实际迭代次数
    """
    u = u_init.copy()

    alpha_init = 1.0
    rho = 0.5
    c1 = 1e-4

    converged = False
    iterations = 0

    best_cost = _numba_mpc_objective(
        u,
        T_h,
        T_b,
        T_s,
        setpoint,
        dt,
        T_env,
        T_cold,
        v_f,
        T_filament,
        theta_1,
        theta_2,
        theta_3,
        theta_4,
        theta_5,
        theta_6,
        theta_7,
        theta_8,
        theta_9,
        c_p,
        Np,
        Nc,
        w_t,
        w_terminal,
        w_r,
        last_control,
    )
    best_u = u.copy()

    for it in range(max_iter):
        iterations = it + 1

        grad = _compute_gradient(
            T_h,
            T_b,
            T_s,
            u,
            setpoint,
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
            Np,
            Nc,
            w_t,
            w_terminal,
            w_r,
            last_control,
        )

        grad_norm = 0.0
        for i in range(Nc):
            grad_norm += grad[i] * grad[i]
        grad_norm = np.sqrt(grad_norm)

        if grad_norm < tol:
            converged = True
            break

        alpha = _line_search(
            T_h,
            T_b,
            T_s,
            u,
            grad,
            setpoint,
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
            Np,
            Nc,
            w_t,
            w_terminal,
            w_r,
            last_control,
            alpha_init,
            rho,
            c1,
        )

        u = u - alpha * grad

        u = _project_to_bounds(u, min_power, max_power)

        cost = _numba_mpc_objective(
            u,
            T_h,
            T_b,
            T_s,
            setpoint,
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
            Np,
            Nc,
            w_t,
            w_terminal,
            w_r,
            last_control,
        )

        if cost < best_cost:
            improvement = best_cost - cost
            best_cost = cost
            best_u = u.copy()

            if improvement < tol:
                converged = True
                break

    return best_u, converged, iterations


# =============================================================================
# 批量预测函数 - 用于MPC优化
# =============================================================================


@jit(nopython=True, cache=True, fastmath=True, parallel=True)
def _batch_predict_trajectories(
    T_h,
    T_b,
    T_s,
    u_candidates,
    dt,
    T_env,
    T_cold,
    v_f,
    T_filament,
    theta_1,
    theta_2,
    theta_3,
    theta_4,
    theta_5,
    theta_6,
    theta_7,
    theta_8,
    theta_9,
    c_p,
    Np,
):
    """
    批量预测多条轨迹

    用于并行评估多个候选控制序列

    参数:
        u_candidates: 候选控制序列数组 (n_candidates, Np)

    返回:
        T_s_final: 每条轨迹的最终传感器温度 (n_candidates,)
    """
    n_candidates = u_candidates.shape[0]
    T_s_final = np.zeros(n_candidates)

    for k in prange(n_candidates):
        T_h_cur = T_h
        T_b_cur = T_b
        T_s_cur = T_s

        for i in range(Np):
            T_h_cur, T_b_cur, T_s_cur = _numba_model_step(
                T_h_cur,
                T_b_cur,
                T_s_cur,
                u_candidates[k, i],
                dt,
                T_env,
                T_cold,
                v_f,
                T_filament,
                theta_1,
                theta_2,
                theta_3,
                theta_4,
                theta_5,
                theta_6,
                theta_7,
                theta_8,
                theta_9,
                c_p,
            )

        T_s_final[k] = T_s_cur

    return T_s_final


class ControlMPC:
    def __init__(self, profile, heater, load_clean=False, register=True):
        self.profile = profile
        self._load_profile()
        self.heater = heater
        self.heater_max_power = heater.get_max_power() * self.const_heater_power

        self.want_ambient_refresh = self.ambient_sensor is not None
        self.state_block_temp = (
            AMBIENT_TEMP if load_clean else self._heater_temp()
        )
        self.state_sensor_temp = self.state_block_temp
        self.state_ambient_temp = AMBIENT_TEMP

        self.last_power = 0.0
        self.last_loss_ambient = 0.0
        self.last_loss_filament = 0.0
        self.last_time = 0.0
        self.last_temp_time = 0.0

        self.printer = heater.printer
        self.toolhead = None

        if not register:
            return

        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "MPC_CALIBRATE",
            "HEATER",
            heater.get_name(),
            self.cmd_MPC_CALIBRATE,
            desc=self.cmd_MPC_CALIBRATE_help,
        )
        gcode.register_mux_command(
            "MPC_SET",
            "HEATER",
            heater.get_name(),
            self.cmd_MPC_SET,
            desc=self.cmd_MPC_SET_help,
        )

    cmd_MPC_SET_help = "Set MPC parameter"

    def cmd_MPC_SET(self, gcmd):
        self.const_filament_diameter = gcmd.get_float(
            "FILAMENT_DIAMETER", self.const_filament_diameter
        )
        self.const_filament_density = gcmd.get_float(
            "FILAMENT_DENSITY", self.const_filament_density
        )
        self.const_filament_heat_capacity = gcmd.get_float(
            "FILAMENT_HEAT_CAPACITY", self.const_filament_heat_capacity
        )

        self.const_block_heat_capacity = gcmd.get_float(
            "BLOCK_HEAT_CAPACITY", self.const_block_heat_capacity
        )
        self.const_sensor_responsiveness = gcmd.get_float(
            "SENSOR_RESPONSIVENESS", self.const_sensor_responsiveness
        )
        self.const_ambient_transfer = gcmd.get_float(
            "AMBIENT_TRANSFER", self.const_ambient_transfer
        )

        if gcmd.get("FAN_AMBIENT_TRANSFER", None):
            try:
                self.const_fan_ambient_transfer = [
                    float(v)
                    for v in gcmd.get("FAN_AMBIENT_TRANSFER").split(",")
                ]
            except ValueError:
                raise gcmd.error(
                    f"Error on '{gcmd._commandline}': unable to parse FAN_AMBIENT_TRANSFER\n"
                    "Must be a comma-separated list of values ('0.05,0.07,0.08')"
                )

        temp = gcmd.get("FILAMENT_TEMP", None)
        if temp is not None:
            temp = temp.lower().strip()
            if temp == "sensor":
                self.filament_temp_src = (FILAMENT_TEMP_SRC_SENSOR,)
            elif temp == "ambient":
                self.filament_temp_src = (FILAMENT_TEMP_SRC_AMBIENT,)
            else:
                try:
                    value = float(temp)
                except ValueError:
                    raise gcmd.error(
                        f"Error on '{gcmd._commandline}': unable to parse FILAMENT_TEMP\n"
                        "Valid options are 'sensor', 'ambient', or number."
                    )
                self.filament_temp_src = (FILAMENT_TEMP_SRC_FIXED, value)

        self._update_filament_const()

    cmd_MPC_CALIBRATE_help = "Run MPC calibration"

    def cmd_MPC_CALIBRATE(self, gcmd):
        cal = MpcCalibrate(self.printer, self.heater, self)
        cal.run(gcmd)

    # Helpers

    def _heater_temp(self):
        return self.heater.get_temp(self.heater.reactor.monotonic())[0]

    def _load_profile(self):
        self.const_block_heat_capacity = self.profile["block_heat_capacity"]
        self.const_ambient_transfer = self.profile["ambient_transfer"]
        self.const_target_reach_time = self.profile["target_reach_time"]
        self.const_heater_power = self.profile["heater_power"]
        self.const_smoothing = self.profile["smoothing"]
        self.const_sensor_responsiveness = self.profile["sensor_responsiveness"]
        self.const_min_ambient_change = self.profile["min_ambient_change"]
        self.const_steady_state_rate = self.profile["steady_state_rate"]
        self.const_filament_diameter = self.profile["filament_diameter"]
        self.const_filament_density = self.profile["filament_density"]
        self.const_filament_heat_capacity = self.profile[
            "filament_heat_capacity"
        ]
        self.const_maximum_retract = self.profile["maximum_retract"]
        self.filament_temp_src = self.profile["filament_temp_src"]
        self._update_filament_const()
        self.ambient_sensor = self.profile["ambient_temp_sensor"]
        self.cooling_fan = self.profile["cooling_fan"]
        self.const_fan_ambient_transfer = self.profile["fan_ambient_transfer"]

    def is_valid(self):
        return (
            self.const_block_heat_capacity is not None
            and self.const_ambient_transfer is not None
            and self.const_sensor_responsiveness is not None
        )

    def check_valid(self):
        if self.is_valid():
            return
        name = self.heater.get_name()
        raise self.printer.command_error(
            f"Cannot activate '{name}' as MPC control is not fully configured.\n\n"
            f"Run 'MPC_CALIBRATE' or ensure 'block_heat_capacity', 'sensor_responsiveness', and "
            f"'ambient_transfer' settings are defined for '{name}'."
        )

    def _update_filament_const(self):
        radius = self.const_filament_diameter / 2.0
        self.const_filament_cross_section_heat_capacity = (
            (radius * radius)  # mm^2
            * math.pi  # 1
            / 1000.0  # mm^3 => cm^3
            * self.const_filament_density  # g/cm^3
            * self.const_filament_heat_capacity  # J/g/K
        )

    # Control interface

    def temperature_update(self, read_time, temp, target_temp):
        if not self.is_valid():
            self.heater.set_pwm(read_time, 0.0)
            return

        dt = read_time - self.last_temp_time
        if self.last_temp_time == 0.0 or dt < 0.0 or dt > 1.0:
            dt = 0.1

        # Extruder position
        extrude_speed_prev = 0.0
        extrude_speed_next = 0.0
        if target_temp != 0.0:
            if self.toolhead is None:
                self.toolhead = self.printer.lookup_object("toolhead")
            if self.toolhead is not None:
                extruder = self.toolhead.get_extruder()
                if (
                    hasattr(extruder, "find_past_position")
                    and extruder.get_heater() == self.heater
                ):
                    pos = extruder.find_past_position(read_time)

                    pos_prev = extruder.find_past_position(read_time - dt)
                    pos_moved = max(-self.const_maximum_retract, pos - pos_prev)
                    extrude_speed_prev = pos_moved / dt

                    pos_next = extruder.find_past_position(read_time + dt)
                    pos_move = max(-self.const_maximum_retract, pos_next - pos)
                    extrude_speed_next = pos_move / dt

        # Modulate ambient transfer coefficient with fan speed
        ambient_transfer = self.const_ambient_transfer
        if self.cooling_fan and len(self.const_fan_ambient_transfer) > 1:
            fan_speed = max(
                0.0, min(1.0, self.cooling_fan.get_status(read_time)["speed"])
            )
            fan_break = fan_speed * (len(self.const_fan_ambient_transfer) - 1)
            below = self.const_fan_ambient_transfer[math.floor(fan_break)]
            above = self.const_fan_ambient_transfer[math.ceil(fan_break)]
            if below != above:
                frac = fan_break % 1.0
                ambient_transfer = below * (1 - frac) + frac * above
            else:
                ambient_transfer = below

        # Simulate

        # Expected power by heating at last power setting
        expected_heating = self.last_power
        # Expected power from block to ambient
        block_ambient_delta = self.state_block_temp - self.state_ambient_temp
        expected_ambient_transfer = block_ambient_delta * ambient_transfer
        expected_filament_transfer = (
            block_ambient_delta
            * extrude_speed_prev
            * self.const_filament_cross_section_heat_capacity
        )

        # Expected block dT since last period
        expected_block_dT = (
            (
                expected_heating
                - expected_ambient_transfer
                - expected_filament_transfer
            )
            * dt
            / self.const_block_heat_capacity
        )
        self.state_block_temp += expected_block_dT

        # Expected sensor dT since last period
        expected_sensor_dT = (
            (self.state_block_temp - self.state_sensor_temp)
            * self.const_sensor_responsiveness
            * dt
        )
        self.state_sensor_temp += expected_sensor_dT

        # Correct

        smoothing = 1 - (1 - self.const_smoothing) ** dt
        adjustment_dT = (temp - self.state_sensor_temp) * smoothing
        self.state_block_temp += adjustment_dT
        self.state_sensor_temp += adjustment_dT

        if self.want_ambient_refresh:
            temp = self.ambient_sensor.get_temp(read_time)[0]
            if temp != 0.0:
                self.state_ambient_temp = temp
                self.want_ambient_refresh = False
        if (self.last_power > 0 and self.last_power < 1.0) or abs(
            expected_block_dT + adjustment_dT
        ) < self.const_steady_state_rate * dt:
            if adjustment_dT > 0.0:
                ambient_delta = max(
                    adjustment_dT, self.const_min_ambient_change * dt
                )
            else:
                ambient_delta = min(
                    adjustment_dT, -self.const_min_ambient_change * dt
                )
            self.state_ambient_temp += ambient_delta

        # Output

        # Amount of power needed to reach the target temperature in the desired time

        heating_power = (
            (target_temp - self.state_block_temp)
            * self.const_block_heat_capacity
            / self.const_target_reach_time
        )
        # Losses (+ = lost from block, - = gained to block)
        block_ambient_delta = self.state_block_temp - self.state_ambient_temp
        loss_ambient = block_ambient_delta * ambient_transfer
        block_filament_delta = self.state_block_temp - self.filament_temp(
            read_time, self.state_ambient_temp
        )
        loss_filament = (
            block_filament_delta
            * extrude_speed_next
            * self.const_filament_cross_section_heat_capacity
        )

        if target_temp != 0.0:
            # The required power is the desired heating power + compensation for all the losses
            power = max(
                0.0,
                min(
                    self.heater_max_power,
                    heating_power + loss_ambient + loss_filament,
                ),
            )
        else:
            power = 0

        duty = power / self.const_heater_power

        # logging.info(
        #     "mpc: [%.3f/%.3f] %.2f => %.2f / %.2f / %.2f = %.2f[%.2f+%.2f+%.2f] / %.2f, dT %.2f, E %.2f=>%.2f",
        #     dt,
        #     smoothing,
        #     temp,
        #     self.state_block_temp,
        #     self.state_sensor_temp,
        #     self.state_ambient_temp,
        #     power,
        #     heating_power,
        #     loss_ambient,
        #     loss_filament,
        #     duty,
        #     adjustment_dT,
        #     extrude_speed_prev,
        #     extrude_speed_next,
        # )

        self.last_power = power
        self.last_loss_ambient = loss_ambient
        self.last_loss_filament = loss_filament
        self.last_temp_time = read_time
        self.heater.set_pwm(read_time, duty)

    def filament_temp(self, read_time, ambient_temp):
        src = self.filament_temp_src
        if src[0] == FILAMENT_TEMP_SRC_FIXED:
            return src[1]
        elif (
            src[0] == FILAMENT_TEMP_SRC_SENSOR
            and self.ambient_sensor is not None
        ):
            return self.ambient_sensor.get_temp(read_time)[0]
        else:
            return ambient_temp

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return abs(target_temp - smoothed_temp) > 1.0

    def update_smooth_time(self):
        pass

    def get_profile(self):
        return self.profile

    def get_type(self):
        return "mpc"

    def get_status(self, eventtime):
        return {
            "temp_block": self.state_block_temp,
            "temp_sensor": self.state_sensor_temp,
            "temp_ambient": self.state_ambient_temp,
            "power": self.last_power,
            "loss_ambient": self.last_loss_ambient,
            "loss_filament": self.last_loss_filament,
            "filament_temp": self.filament_temp_src,
            "filament_heat_capacity": self.const_filament_heat_capacity,
            "filament_density": self.const_filament_density,
        }


class MpcCalibrate:
    def __init__(self, printer, heater, orig_control):
        self.printer = printer
        self.heater = heater
        self.orig_control = orig_control

    def save_heatup_data(self, gcmd, samples):
        """
        Save heatup test data to CSV file
        """
        import os
        import time

        # Get home directory and create data path
        home_dir = os.path.expanduser("~")
        data_dir = os.path.join(home_dir, "printer_data", "data")

        # Create directory if it doesn't exist (including parent directories)
        os.makedirs(data_dir, exist_ok=True)

        # Generate filename with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"nenocontrol_{self.heater.get_name()}_{timestamp}.csv"
        file_path = os.path.join(data_dir, filename)

        try:
            # Write data to CSV
            with open(file_path, "w") as f:
                # Write header
                f.write("time,temperature\n")

                # Write data - convert to relative time if we have samples
                if samples:
                    start_time = samples[0][0]
                    for t, temp in samples:
                        rel_time = t - start_time
                        f.write(f"{rel_time:.3f},{temp:.2f}\n")
                else:
                    logging.warning("No samples to save in save_heatup_data")

            # Log and respond to user
            logging.info(f"Heatup data saved to: {file_path}")
            gcmd.respond_info(f"Heatup test data saved to {file_path}")
        except Exception as e:
            logging.error(f"Failed to save heatup data: {e}")
            gcmd.respond_info(f"Warning: Failed to save heatup data: {e}")

    def run(self, gcmd):
        use_analytic = gcmd.get("USE_DELTA", None) is not None
        ambient_max_measure_time = gcmd.get_float(
            "AMBIENT_MAX_MEASURE_TIME", 20.0, above=0.0
        )
        ambient_measure_sample_time = gcmd.get_float(
            "AMBIENT_MEASURE_SAMPLE_TIME", 5.0, below=ambient_max_measure_time
        )
        fan_breakpoints = gcmd.get_int("FAN_BREAKPOINTS", 3, minval=2)
        default_target_temp = (
            90.0 if self.heater.get_name() == "heater_bed" else 200.0
        )
        target_temp = gcmd.get_float("TARGET", default_target_temp, minval=60.0)
        threshold_temp = gcmd.get_float(
            "THRESHOLD", max(50.0, min(100, target_temp - 100.0))
        )

        control = TuningControl(self.heater)
        old_control = self.heater.set_control(control)
        try:
            ambient_temp = self.await_ambient(gcmd, control, threshold_temp)
            samples = self.heatup_test(gcmd, target_temp, control)
            self.save_heatup_data(gcmd, samples)
            first_res = self.process_first_pass(
                samples,
                self.orig_control.heater_max_power,
                ambient_temp,
                threshold_temp,
                use_analytic,
            )
            logging.info("First pass: %s", first_res)

            profile = dict(self.orig_control.profile)
            for key in [
                "block_heat_capacity",
                "ambient_transfer",
                "sensor_responsiveness",
            ]:
                profile[key] = first_res[key]
            new_control = ControlMPC(profile, self.heater, False, False)
            new_control.state_block_temp = first_res["post_block_temp"]
            new_control.state_sensor_temp = first_res["post_sensor_temp"]
            new_control.state_ambient_temp = ambient_temp
            self.heater.set_control(new_control)

            transfer_res = self.transfer_test(
                gcmd,
                ambient_max_measure_time,
                ambient_measure_sample_time,
                fan_breakpoints,
                first_res,
            )
            second_res = self.process_second_pass(
                first_res,
                transfer_res,
                ambient_temp,
                self.orig_control.heater_max_power,
            )
            logging.info("Second pass: %s", second_res)

            block_heat_capacity = (
                second_res["block_heat_capacity"]
                if use_analytic
                else first_res["block_heat_capacity"]
            )
            sensor_responsiveness = (
                second_res["sensor_responsiveness"]
                if use_analytic
                else first_res["sensor_responsiveness"]
            )
            ambient_transfer = second_res["ambient_transfer"]
            fan_ambient_transfer = ", ".join(
                [f"{p:.6g}" for p in second_res["fan_ambient_transfer"]]
            )

            cfgname = self.heater.get_name()
            gcmd.respond_info(
                f"Finished MPC calibration of heater '{cfgname}'\n"
                "Measured:\n "
                f"  block_heat_capacity={block_heat_capacity:#.6g} [J/K]\n"
                f"  sensor_responsiveness={sensor_responsiveness:#.6g} [K/s/K]\n"
                f"  ambient_transfer={ambient_transfer:#.6g} [W/K]\n"
                f"  fan_ambient_transfer={fan_ambient_transfer} [W/K]\n"
            )

            configfile = self.heater.printer.lookup_object("configfile")
            configfile.set(cfgname, "control", "mpc")
            configfile.set(
                cfgname, "block_heat_capacity", f"{block_heat_capacity:#.6g}"
            )
            configfile.set(
                cfgname,
                "sensor_responsiveness",
                f"{sensor_responsiveness:#.6g}",
            )
            configfile.set(
                cfgname, "ambient_transfer", f"{ambient_transfer:#.6g}"
            )
            configfile.set(
                cfgname,
                "fan_ambient_transfer",
                fan_ambient_transfer,
            )

        except self.printer.command_error as e:
            raise gcmd.error("%s failed: %s" % (gcmd.get_command(), e))
        finally:
            self.heater.set_control(old_control)
            self.heater.alter_target(0.0)

    def wait_stable(self, cycles=5):
        """
        We wait for the extruder to cycle x amount of times above and below the target
        doing this should ensure the temperature is stable enough to give a good result
        as a fallback if it stays within 0.1 degree for ~30 seconds it is also accepted
        """

        below_target = True
        above_target = 0
        on_target = 0
        starttime = self.printer.reactor.monotonic()

        def process(eventtime):
            nonlocal below_target, above_target, on_target
            temp, target = self.heater.get_temp(eventtime)
            if below_target and temp > target + 0.015:
                above_target += 1
                below_target = False
            elif not below_target and temp < target - 0.015:
                below_target = True
            if (
                above_target >= cycles
                and (self.printer.reactor.monotonic() - starttime) > 30.0
            ):
                return False
            if above_target > 0 and abs(target - temp) < 0.1:
                on_target += 1
            else:
                on_target = 0
            if on_target >= 150:  # in case the heating is super consistent
                return False
            return True

        self.printer.wait_while(process, True, 0.2)

    def wait_settle(self, max_rate):
        last_temp = None
        next_check = None
        samples = []

        def process(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            samples.append((eventtime, temp))
            while samples[0][0] < eventtime - 10.0:
                samples.pop(0)
            dT = samples[-1][1] - samples[0][1]
            dt = samples[-1][0] - samples[0][0]
            if dt < 8.0:
                return True
            rate = abs(dT / dt)
            return not rate < max_rate

        self.printer.wait_while(process)
        return samples[-1][1]

    def await_ambient(self, gcmd, control, minimum_temp):
        self.heater.alter_target(1.0)  # Turn on fan to increase settling speed
        if self.orig_control.ambient_sensor is not None:
            # If we have an ambient sensor we won't waste time waiting for ambient.
            # We do however need to wait for sub minimum_temp(we pick -5 C relative).
            reported = [False]
            target = minimum_temp - 5

            def process(eventtime):
                temp, _ = self.heater.get_temp(eventtime)
                ret = temp > target
                if ret and not reported[0]:
                    gcmd.respond_info(
                        f"Waiting for heater to drop below {target} degrees Celsius"
                    )
                    reported[0] = True
                return ret

            self.printer.wait_while(process)
            self.heater.alter_target(0.0)
            return self.orig_control.ambient_sensor.get_temp(
                self.heater.reactor.monotonic()
            )[0]

        gcmd.respond_info("Waiting for heater to settle at ambient temperature")
        ambient_temp = self.wait_settle(0.01)
        self.heater.alter_target(0.0)
        return ambient_temp

    def heatup_test(self, gcmd, target_temp, control):
        gcmd.respond_info(
            "Performing heatup test, target is %.1f degrees" % (target_temp,)
        )
        control.set_output(self.heater.get_max_power(), target_temp)

        control.logging = True

        def process(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            return temp < target_temp

        self.printer.wait_while(process)
        control.logging = False
        self.heater.alter_target(0.0)

        log = control.log
        control.log = []
        return log

    def transfer_test(
        self,
        gcmd,
        ambient_max_measure_time,
        ambient_measure_sample_time,
        fan_breakpoints,
        first_pass_results,
    ):
        target_temp = round(first_pass_results["post_block_temp"])
        self.heater.set_temp(target_temp)
        gcmd.respond_info(
            "Performing ambient transfer tests, target is %.1f degrees"
            % (target_temp,)
        )

        self.wait_stable(5)

        fan = self.orig_control.cooling_fan

        fan_powers = []
        if fan is None:
            power_base = self.measure_power(
                ambient_max_measure_time, ambient_measure_sample_time
            )
            gcmd.respond_info(f"Average stable power: {power_base} W")
        else:
            for idx in range(0, fan_breakpoints):
                speed = idx / (fan_breakpoints - 1)
                curtime = self.heater.reactor.monotonic()
                fan.set_speed(speed)
                gcmd.respond_info("Waiting for temperature to stabilize")
                self.wait_stable(3)
                gcmd.respond_info(
                    f"Temperature stable, measuring power usage with {speed * 100.0:.0f}% fan speed"
                )
                power = self.measure_power(
                    ambient_max_measure_time, ambient_measure_sample_time
                )
                gcmd.respond_info(
                    f"{speed * 100.0:.0f}% fan average power: {power:.2f} W"
                )
                fan_powers.append((speed, power))
            curtime = self.heater.reactor.monotonic()
            fan.set_speed(0.0)
            power_base = fan_powers[0][1]

        return {
            "target_temp": target_temp,
            "base_power": power_base,
            "fan_powers": fan_powers,
        }

    def measure_power(self, max_time, sample_time):
        samples = []
        time = [0]
        last_time = [None]

        def process(eventtime):
            dt = eventtime - (
                last_time[0] if last_time[0] is not None else eventtime
            )
            last_time[0] = eventtime
            status = self.heater.get_status(eventtime)
            samples.append((dt, status["control_stats"]["power"] * dt))
            time[0] += dt
            return time[0] < max_time

        self.printer.wait_while(process)

        total_energy = 0
        total_time = 0
        for dt, energy in reversed(samples):
            total_energy += energy
            total_time += dt
            if total_time > sample_time:
                break

        return total_energy / total_time

    def fastest_rate(self, samples):
        best = [-1, 0, 0]
        base_t = samples[0][0]
        for idx in range(2, len(samples)):
            dT = samples[idx][1] - samples[idx - 2][1]
            dt = samples[idx][0] - samples[idx - 2][0]
            rate = dT / dt
            if rate > best[0]:
                sample = samples[idx - 1]
                best = [sample[0] - base_t, sample[1], rate]
        return best

    def process_first_pass(
        self,
        all_samples,
        heater_power,
        ambient_temp,
        threshold_temp,
        use_analytic,
    ):
        # Find a continous segment of samples that all lie in the threshold.. range
        best_lower = None
        for idx in range(0, len(all_samples)):
            if all_samples[idx][1] > threshold_temp and best_lower is None:
                best_lower = idx
            elif all_samples[idx][1] < threshold_temp:
                best_lower = None

        t1_time = all_samples[best_lower][0] - all_samples[0][0]

        samples = all_samples[best_lower:]
        pitch = math.floor((len(samples) - 1) / 2)
        # We pick samples 0, pitch, and 2pitch, ensuring matching time spacing
        dt = samples[pitch][0] - samples[0][0]
        t1 = samples[0][1]
        t2 = samples[pitch][1]
        t3 = samples[2 * pitch][1]

        asymp_T = (t2 * t2 - t1 * t3) / (2.0 * t2 - t1 - t3)
        block_responsiveness = -math.log((t2 - asymp_T) / (t1 - asymp_T)) / dt
        ambient_transfer = heater_power / (asymp_T - ambient_temp)

        block_heat_capacity = -1.0
        sensor_responsiveness = -1.0
        start_temp = all_samples[0][1]

        # Asymptotic method
        if use_analytic:
            block_heat_capacity = ambient_transfer / block_responsiveness
            sensor_responsiveness = block_responsiveness / (
                1.0
                - (start_temp - asymp_T)
                * math.exp(-block_responsiveness * t1_time)
                / (t1 - asymp_T)
            )

        # Differential method
        if (
            not use_analytic
            or block_heat_capacity < 0
            or sensor_responsiveness < 0
        ):
            fastest_rate = self.fastest_rate(samples)
            block_heat_capacity = heater_power / fastest_rate[2]
            sensor_responsiveness = fastest_rate[2] / (
                fastest_rate[2] * fastest_rate[0]
                + ambient_temp
                - fastest_rate[0]
            )

        heat_time = all_samples[-1][0] - all_samples[0][0]
        post_block_temp = asymp_T + (start_temp - asymp_T) * math.exp(
            -block_responsiveness * heat_time
        )
        post_sensor_temp = all_samples[-1][1]

        return {
            "post_block_temp": post_block_temp,
            "post_sensor_temp": post_sensor_temp,
            "block_responsiveness": block_responsiveness,
            "ambient_transfer": ambient_transfer,
            "block_heat_capacity": block_heat_capacity,
            "sensor_responsiveness": sensor_responsiveness,
            "asymp_temp": asymp_T,
            "t1": t1,
            "t1_time": t1_time,
            "t2": t2,
            "start_temp": start_temp,
            "dt": dt,
        }

    def process_second_pass(
        self, first_res, transfer_res, ambient_temp, heater_power
    ):
        target_ambient_temp = transfer_res["target_temp"] - ambient_temp
        ambient_transfer = transfer_res["base_power"] / target_ambient_temp
        asymp_T = ambient_temp + heater_power / ambient_transfer
        block_responsiveness = (
            -math.log((first_res["t2"] - asymp_T) / (first_res["t1"] - asymp_T))
            / first_res["dt"]
        )
        block_heat_capacity = ambient_transfer / block_responsiveness
        sensor_responsiveness = block_responsiveness / (
            1.0
            - (first_res["start_temp"] - asymp_T)
            * math.exp(-block_responsiveness * first_res["t1_time"])
            / (first_res["t1"] - asymp_T)
        )

        fan_ambient_transfer = [
            power / target_ambient_temp
            for (_speed, power) in transfer_res["fan_powers"]
        ]

        return {
            "ambient_transfer": ambient_transfer,
            "block_responsiveness": block_responsiveness,
            "block_heat_capacity": block_heat_capacity,
            "sensor_responsiveness": sensor_responsiveness,
            "asymp_temp": asymp_T,
            "fan_ambient_transfer": fan_ambient_transfer,
        }


class TuningControl:
    def __init__(self, heater):
        self.value = 0.0
        self.target = None
        self.heater = heater
        self.log = []
        self.logging = False

    def temperature_update(self, read_time, temp, target_temp):
        if self.logging:
            self.log.append((read_time, temp))
        self.heater.set_pwm(read_time, self.value)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return self.value != 0.0 or self.target != 0

    def set_output(self, value, target):
        self.value = value
        self.target = target
        self.heater.set_temp(target)

    def get_profile(self):
        return {"name": "tuning"}

    def get_type(self):
        return "tuning"


class ControlMPCV2:
    """
    MPC控制器V2 - 基于三节点热力学等价参数模型的完整MPC实现

    三节点模型说明:
        - T_h: 加热器温度 (Heater temperature)
        - T_b: 加热块温度 (Block temperature)
        - T_s: 传感器温度 (Sensor temperature)

    模型方程:
        dT_h/dt = theta_8 * P_in - theta_1 * (T_h - T_b)
        dT_b/dt = theta_2 * (T_h - T_b) - theta_3 * (T_b - T_s)
                  - theta_5 * (T_b - T_env) - theta_6 * (T_b - T_c)
                  - theta_7 * (T_b^4 - T_env^4) - v_f * c_p * theta_9 * (T_b - T_f)
        dT_s/dt = theta_4 * (T_b - T_s)

    参数说明:
        theta_1: 加热器到加热块的热传导系数 (1/s)
        theta_2: 加热块从加热器获得热量的系数 (1/s)
        theta_3: 加热块到传感器的热传导系数 (1/s)
        theta_4: 传感器响应系数 (1/s)
        theta_5: 加热块到环境的对流换热系数 (W/K)
        theta_6: 加热块到冷端的传导系数 (W/K)
        theta_7: 加热块辐射换热系数 (W/K^4)
        theta_8: 加热器功率转换系数 (K/(W*s))
        theta_9: 挤出损耗系数 = theta_8 * theta_2 / theta_1

    MPC算法流程:
        1. 预测: 使用模型预测未来Np步的温度轨迹
        2. 优化: 最小化目标函数 J = w_t * sum((T_s - T_target)^2) + w_c * sum(u^2) + w_r * sum((du)^2)
        3. 控制: 应用优化后的第一个控制量
    """

    def __init__(self, profile, heater, load_clean=False, register=True):
        """
        初始化MPC V2控制器

        参数:
            profile: 配置参数字典，包含所有theta参数和MPC配置
            heater: 加热器对象，用于控制PWM输出
            load_clean: 是否以清洁状态加载（使用环境温度初始化）
            register: 是否注册GCode命令
        """
        self.profile_v2 = profile
        self._load_profile_v2()
        self.heater_v2 = heater
        self.heater_max_power_v2 = (
            heater.get_max_power() * self.const_heater_power_v2
        )

        # 状态变量初始化
        # 是否需要刷新环境温度（当有环境温度传感器时）
        self.want_ambient_refresh_v2 = self.ambient_sensor_v2 is not None

        # 三节点温度状态
        self.state_heater_temp_v2 = (
            AMBIENT_TEMP if load_clean else self._heater_temp_v2()
        )
        self.state_block_temp_v2 = self.state_heater_temp_v2
        self.state_sensor_temp_v2 = self.state_heater_temp_v2
        self.state_ambient_temp_v2 = AMBIENT_TEMP
        self.state_cold_temp_v2 = AMBIENT_TEMP

        # 功率和损耗记录
        self.last_power_v2 = 0.0
        self.last_loss_ambient_v2 = 0.0
        self.last_loss_filament_v2 = 0.0
        self.last_loss_cold_v2 = 0.0
        self.last_loss_radiation_v2 = 0.0
        self.last_time_v2 = 0.0
        self.last_temp_time_v2 = 0.0

        # 打印机对象引用
        self.printer_v2 = heater.printer
        self.toolhead_v2 = None

        # MPC控制历史
        self.last_control_v2 = 0.0
        self.control_history_v2 = []

        # 时间记录 - 用于性能监控
        self.timing_model_step_v2 = 0.0  # 单步模型预测时间 (ms)
        self.timing_predict_v2 = 0.0  # 轨迹预测时间 (ms)
        self.timing_optimize_v2 = 0.0  # MPC优化求解时间 (ms)
        self.timing_total_v2 = 0.0  # 总控制周期时间 (ms)
        self.timing_max_v2 = 0.0  # 最大周期时间 (ms)
        self.timing_avg_v2 = 0.0  # 平均周期时间 (ms)
        self.timing_count_v2 = 0  # 计数器

        # Numba加速可用性
        self.numba_enabled_v2 = NUMBA_AVAILABLE

        # =====================================================================
        # EKF状态估计器初始化
        # Extended Kalman Filter Initialization
        # =====================================================================

        # EKF状态向量: x = [T_h, T_b, T_s]
        self.ekf_state_v2 = np.array(
            [
                self.state_heater_temp_v2,
                self.state_block_temp_v2,
                self.state_sensor_temp_v2,
            ]
        )

        # EKF协方差矩阵初始化
        # P_0: 初始协方差，表示初始状态的不确定性
        self.ekf_P_v2 = np.eye(3) * 10.0  # 初始方差10°C²

        # EKF噪声参数
        # Q: 过程噪声协方差矩阵，反映模型不确定性
        # R: 观测噪声方差，反映传感器精度
        self.ekf_Q_v2 = np.eye(3) * 0.1  # 过程噪声
        self.ekf_R_v2 = np.array([[0.25]])  # 观测噪声 (0.5°C标准差)

        # EKF配置参数
        self.ekf_enabled_v2 = True  # EKF使能标志
        self.ekf_Q_scale_v2 = 0.1  # 过程噪声缩放因子
        self.ekf_R_scale_v2 = 0.25  # 观测噪声缩放因子

        # =====================================================================
        # 内存预分配 - 热启动优化
        # Memory Pre-allocation for Hot Start Optimization
        # =====================================================================

        # MPC控制序列缓存 (控制时域)
        Nc = self.profile_v2.get("control_horizon", 10)
        Np = self.profile_v2.get("prediction_horizon", 30)

        # 预分配控制序列数组
        self._u_cache_v2 = np.zeros(Nc)  # 控制序列缓存
        self._u_prev_v2 = np.zeros(Nc)  # 上一次优化结果
        self._power_sequence_v2 = np.zeros(Np)  # 功率序列

        # 预分配状态轨迹数组
        self._T_h_traj_v2 = np.zeros(Np + 1)
        self._T_b_traj_v2 = np.zeros(Np + 1)
        self._T_s_traj_v2 = np.zeros(Np + 1)

        # 预分配梯度数组
        self._grad_cache_v2 = np.zeros(Nc)

        # 热启动标志
        self._hot_start_available_v2 = False

        # =====================================================================
        # 性能统计
        # Performance Statistics
        # =====================================================================

        self.timing_ekf_v2 = 0.0  # EKF计算时间 (ms)
        self.timing_gradient_v2 = 0.0  # 梯度计算时间 (ms)
        self.timing_projection_v2 = 0.0  # 投影操作时间 (ms)
        self.pgd_iterations_v2 = 0  # PGD迭代次数
        self.pgd_converged_v2 = False  # PGD收敛标志

        if self.numba_enabled_v2:
            self._warmup_numba()

        if not register:
            return

        # 注册GCode命令
        gcode = self.printer_v2.lookup_object("gcode")
        gcode.register_mux_command(
            "MPC_SET_V2",
            "HEATER",
            heater.get_name(),
            self.cmd_MPC_SET_V2,
            desc=self.cmd_MPC_SET_V2_help,
        )

    cmd_MPC_SET_V2_help = "设置MPC V2参数"

    def cmd_MPC_SET_V2(self, gcmd):
        """
        处理MPC_SET_V2 GCode命令

        可设置的参数:
            FILAMENT_DIAMETER: 耗材直径 (mm)
            FILAMENT_DENSITY: 耗材密度 (g/cm³)
            FILAMENT_HEAT_CAPACITY: 耗材比热容 (J/g/K)
            THETA_1 ~ THETA_8: 系统辨识参数
            FILAMENT_TEMP: 耗材温度来源 ('sensor', 'ambient', 或数值)
        """
        # 耗材参数设置
        self.const_filament_diameter_v2 = gcmd.get_float(
            "FILAMENT_DIAMETER", self.const_filament_diameter_v2
        )
        self.const_filament_density_v2 = gcmd.get_float(
            "FILAMENT_DENSITY", self.const_filament_density_v2
        )
        self.const_filament_heat_capacity_v2 = gcmd.get_float(
            "FILAMENT_HEAT_CAPACITY", self.const_filament_heat_capacity_v2
        )

        # 系统辨识参数设置
        self.const_theta_1_v2 = gcmd.get_float("THETA_1", self.const_theta_1_v2)
        self.const_theta_2_v2 = gcmd.get_float("THETA_2", self.const_theta_2_v2)
        self.const_theta_3_v2 = gcmd.get_float("THETA_3", self.const_theta_3_v2)
        self.const_theta_4_v2 = gcmd.get_float("THETA_4", self.const_theta_4_v2)
        self.const_theta_5_v2 = gcmd.get_float("THETA_5", self.const_theta_5_v2)
        self.const_theta_6_v2 = gcmd.get_float("THETA_6", self.const_theta_6_v2)
        self.const_theta_7_v2 = gcmd.get_float("THETA_7", self.const_theta_7_v2)
        self.const_theta_8_v2 = gcmd.get_float("THETA_8", self.const_theta_8_v2)
        self._update_theta_9_v2()

        # 耗材温度来源设置
        temp = gcmd.get("FILAMENT_TEMP", None)
        if temp is not None:
            temp = temp.lower().strip()
            if temp == "sensor":
                self.filament_temp_src_v2 = (FILAMENT_TEMP_SRC_SENSOR,)
            elif temp == "ambient":
                self.filament_temp_src_v2 = (FILAMENT_TEMP_SRC_AMBIENT,)
            else:
                try:
                    value = float(temp)
                except ValueError:
                    raise gcmd.error(
                        f"Error on '{gcmd._commandline}': unable to parse FILAMENT_TEMP\n"
                        "Valid options are 'sensor', 'ambient', or number."
                    )
                self.filament_temp_src_v2 = (FILAMENT_TEMP_SRC_FIXED, value)

        self._update_filament_const_v2()

    def _heater_temp_v2(self):
        """获取当前加热器温度"""
        return self.heater_v2.get_temp(self.heater_v2.reactor.monotonic())[0]

    def _load_profile_v2(self):
        """
        从配置文件加载MPC V2参数

        加载的参数包括:
            - 系统辨识参数 (theta_1 ~ theta_8)
            - MPC配置参数 (预测时域、控制时域、权重)
            - 耗材参数 (直径、密度、比热容)
            - 传感器配置 (环境温度传感器、冷端温度传感器)
        """
        # 系统辨识参数 - 三节点热力学模型
        self.const_theta_1_v2 = self.profile_v2.get("theta_1", 5.029312e-02)
        self.const_theta_2_v2 = self.profile_v2.get("theta_2", 2.806417e-01)
        self.const_theta_3_v2 = self.profile_v2.get("theta_3", 1.065468e-02)
        self.const_theta_4_v2 = self.profile_v2.get("theta_4", 1.370236e-01)
        self.const_theta_5_v2 = self.profile_v2.get("theta_5", 3.195262e-03)
        self.const_theta_6_v2 = self.profile_v2.get("theta_6", 2.327857e-02)
        self.const_theta_7_v2 = self.profile_v2.get("theta_7", 2.571527e-11)
        self.const_theta_8_v2 = self.profile_v2.get("theta_8", 8.000314e-02)
        self._update_theta_9_v2()

        # 加热器基本参数
        self.const_heater_power_v2 = self.profile_v2.get("heater_power", 40.0)
        self.const_smoothing_v2 = self.profile_v2.get("smoothing", 0.83)
        self.const_min_ambient_change_v2 = self.profile_v2.get(
            "min_ambient_change", 1.0
        )
        self.const_steady_state_rate_v2 = self.profile_v2.get(
            "steady_state_rate", 0.5
        )

        # MPC算法参数
        self.const_prediction_horizon_v2 = self.profile_v2.get(
            "prediction_horizon", 30
        )
        self.const_control_horizon_v2 = self.profile_v2.get(
            "control_horizon", 10
        )
        self.const_weight_tracking_v2 = self.profile_v2.get(
            "weight_tracking", 10.0
        )
        self.const_weight_terminal_v2 = self.profile_v2.get(
            "weight_terminal", 50.0
        )
        self.const_weight_rate_v2 = self.profile_v2.get("weight_rate", 0.1)
        self.const_max_iterations_v2 = self.profile_v2.get(
            "max_iterations", 200
        )
        self.const_tolerance_v2 = self.profile_v2.get("tolerance", 1e-5)

        # 耗材参数
        self.const_filament_diameter_v2 = self.profile_v2.get(
            "filament_diameter", 1.75
        )
        self.const_filament_density_v2 = self.profile_v2.get(
            "filament_density", 1.2
        )
        self.const_filament_heat_capacity_v2 = self.profile_v2.get(
            "filament_heat_capacity", 1.8
        )
        self.const_maximum_retract_v2 = self.profile_v2.get(
            "maximum_retract", 2.0
        )
        self.filament_temp_src_v2 = self.profile_v2.get(
            "filament_temp_src", (FILAMENT_TEMP_SRC_AMBIENT,)
        )
        self._update_filament_const_v2()

        # 传感器配置
        self.ambient_sensor_v2 = self.profile_v2.get(
            "ambient_temp_sensor", None
        )
        self.cold_temp_sensor_v2 = self.profile_v2.get("cold_temp_sensor", None)
        self.cooling_fan_v2 = self.profile_v2.get("cooling_fan", None)
        self.const_fan_ambient_transfer_v2 = self.profile_v2.get(
            "fan_ambient_transfer", []
        )

        self.const_T_cold_v2 = self.profile_v2.get("T_cold", 25.0)

        # EKF参数配置
        self.ekf_enabled_v2 = self.profile_v2.get("ekf_enabled", True)
        self.ekf_Q_scale_v2 = self.profile_v2.get("ekf_Q_scale", 0.1)
        self.ekf_R_scale_v2 = self.profile_v2.get("ekf_R_scale", 0.25)

        # PGD求解器参数
        self.pgd_max_iterations_v2 = self.profile_v2.get(
            "pgd_max_iterations", 100
        )
        self.pgd_tolerance_v2 = self.profile_v2.get("pgd_tolerance", 1e-3)
        self.pgd_line_search_alpha_v2 = self.profile_v2.get(
            "pgd_line_search_alpha", 1.0
        )
        self.pgd_line_search_rho_v2 = self.profile_v2.get(
            "pgd_line_search_rho", 0.5
        )

    def _update_theta_9_v2(self):
        """
        计算theta_9参数

        theta_9 = theta_8 * theta_2 / theta_1
        该参数用于计算挤出过程中的热量损耗
        """
        if abs(self.const_theta_1_v2) > 1e-10:
            self.const_theta_9_v2 = (
                self.const_theta_8_v2
                * self.const_theta_2_v2
                / self.const_theta_1_v2
            )
        else:
            self.const_theta_9_v2 = 0.0

    def _update_filament_const_v2(self):
        """
        计算耗材截面热容

        计算公式: 截面积 * 密度 * 比热容
        用于计算挤出过程中的热量损耗
        """
        radius = self.const_filament_diameter_v2 / 2.0
        self.const_filament_cross_section_heat_capacity_v2 = (
            (radius * radius)  # mm^2 (截面积)
            * math.pi  # 圆周率
            / 1000.0  # mm^3 => cm^3
            * self.const_filament_density_v2  # g/cm^3
            * self.const_filament_heat_capacity_v2  # J/g/K
        )

    def _warmup_numba(self):
        """
        Numba 预热 - 触发 JIT 编译以避免首次调用的延迟

        在控制器初始化时调用，确保所有 Numba 加速函数已编译完成，
        避免在首次控制循环时因 JIT 编译导致控制延迟。
        """
        _numba_model_step(
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            25.0,
            25.0,
            0.0,
            25.0,
            5.029312e-02,
            2.806417e-01,
            1.065468e-02,
            1.370236e-01,
            3.195262e-03,
            2.327857e-02,
            2.571527e-11,
            8.000314e-02,
            4.458e-01,
            0.00259,
        )

    def is_valid_v2(self):
        """检查所有必需的theta参数是否已配置"""
        return (
            self.const_theta_1_v2 is not None
            and self.const_theta_2_v2 is not None
            and self.const_theta_3_v2 is not None
            and self.const_theta_4_v2 is not None
            and self.const_theta_5_v2 is not None
            and self.const_theta_6_v2 is not None
            and self.const_theta_7_v2 is not None
            and self.const_theta_8_v2 is not None
        )

    def check_valid(self):
        """验证配置有效性，如果无效则抛出错误"""
        if self.is_valid_v2():
            return
        name = self.heater_v2.get_name()
        raise self.printer_v2.command_error(
            f"Cannot activate '{name}' as MPC V2 control is not fully configured.\n\n"
            f"Ensure all theta parameters are defined for '{name}'."
        )

    def _model_step_v2(
        self, T_h, T_b, T_s, power, dt, T_env, T_cold, v_f, T_filament
    ):
        """
        单步模型仿真 - 使用前向欧拉法

        基于三节点热力学模型计算一个时间步长后的温度变化
        优先使用Numba加速版本，回退到纯Python版本

        参数:
            T_h: 当前加热器温度 (°C)
            T_b: 当前加热块温度 (°C)
            T_s: 当前传感器温度 (°C)
            power: 加热功率 (W)
            dt: 时间步长 (s)
            T_env: 环境温度 (°C)
            T_cold: 冷端温度 (°C)
            v_f: 挤出速率 (mm³/s)
            T_filament: 耗材温度 (°C)

        返回:
            (T_h_new, T_b_new, T_s_new): 更新后的温度元组
        """
        t_start = time.perf_counter()

        if self.numba_enabled_v2:
            T_h_new, T_b_new, T_s_new = _numba_model_step(
                T_h,
                T_b,
                T_s,
                power,
                dt,
                T_env,
                T_cold,
                v_f,
                T_filament,
                self.const_theta_1_v2,
                self.const_theta_2_v2,
                self.const_theta_3_v2,
                self.const_theta_4_v2,
                self.const_theta_5_v2,
                self.const_theta_6_v2,
                self.const_theta_7_v2,
                self.const_theta_8_v2,
                self.const_theta_9_v2,
                self.const_filament_cross_section_heat_capacity_v2,
            )
        else:
            T_env_K = T_env + 273.15
            T_b_K = T_b + 273.15
            c_p = self.const_filament_cross_section_heat_capacity_v2

            dT_h = self.const_theta_8_v2 * power - self.const_theta_1_v2 * (
                T_h - T_b
            )

            q_hb = self.const_theta_2_v2 * (T_h - T_b)
            q_bs = self.const_theta_3_v2 * (T_b - T_s)
            P_conv = self.const_theta_5_v2 * (T_b - T_env)
            P_cond = self.const_theta_6_v2 * (T_b - T_cold)
            P_rad = self.const_theta_7_v2 * (T_b_K**4 - T_env_K**4)
            P_extrusion = v_f * c_p * self.const_theta_9_v2 * (T_b - T_filament)

            dT_b = q_hb - q_bs - P_conv - P_cond - P_rad - P_extrusion
            dT_s = self.const_theta_4_v2 * (T_b - T_s)

            T_h_new = T_h + dT_h * dt
            T_b_new = T_b + dT_b * dt
            T_s_new = T_s + dT_s * dt

            T_h_new = max(-50.0, min(600.0, T_h_new))
            T_b_new = max(-50.0, min(600.0, T_b_new))
            T_s_new = max(-50.0, min(600.0, T_s_new))

        self.timing_model_step_v2 = (time.perf_counter() - t_start) * 1000.0

        return T_h_new, T_b_new, T_s_new

    def _predict_trajectory_v2(
        self, initial_state, power_sequence, dt, T_env, T_cold, v_f, T_filament
    ):
        """
        多步预测 - 预测未来温度轨迹

        使用模型迭代计算未来N步的温度变化，用于MPC优化
        优先使用Numba加速版本，回退到纯Python版本

        参数:
            initial_state: 初始状态字典 {'T_h', 'T_b', 'T_s'}
            power_sequence: 功率序列数组 (W)，长度为预测步数
            dt: 时间步长 (s)
            T_env: 环境温度 (°C)
            T_cold: 冷端温度 (°C)
            v_f: 挤出速率 (mm³/s)
            T_filament: 耗材温度 (°C)

        返回:
            (T_h_arr, T_b_arr, T_s_arr): 温度轨迹数组，长度为 n_steps + 1
                - T_h_arr: 加热器温度轨迹
                - T_b_arr: 加热块温度轨迹
                - T_s_arr: 传感器温度轨迹
        """
        t_start = time.perf_counter()

        if self.numba_enabled_v2:
            T_h_arr, T_b_arr, T_s_arr = _numba_predict_trajectory(
                initial_state["T_h"],
                initial_state["T_b"],
                initial_state["T_s"],
                power_sequence,
                dt,
                T_env,
                T_cold,
                v_f,
                T_filament,
                self.const_theta_1_v2,
                self.const_theta_2_v2,
                self.const_theta_3_v2,
                self.const_theta_4_v2,
                self.const_theta_5_v2,
                self.const_theta_6_v2,
                self.const_theta_7_v2,
                self.const_theta_8_v2,
                self.const_theta_9_v2,
                self.const_filament_cross_section_heat_capacity_v2,
            )
        else:
            n_steps = len(power_sequence)
            T_h_arr = np.zeros(n_steps + 1)
            T_b_arr = np.zeros(n_steps + 1)
            T_s_arr = np.zeros(n_steps + 1)

            T_h = initial_state["T_h"]
            T_b = initial_state["T_b"]
            T_s = initial_state["T_s"]

            T_h_arr[0] = T_h
            T_b_arr[0] = T_b
            T_s_arr[0] = T_s

            for i in range(n_steps):
                T_h, T_b, T_s = self._model_step_v2(
                    T_h,
                    T_b,
                    T_s,
                    power_sequence[i],
                    dt,
                    T_env,
                    T_cold,
                    v_f,
                    T_filament,
                )
                T_h_arr[i + 1] = T_h
                T_b_arr[i + 1] = T_b
                T_s_arr[i + 1] = T_s

        self.timing_predict_v2 = (time.perf_counter() - t_start) * 1000.0

        return T_h_arr, T_b_arr, T_s_arr

    def _mpc_objective_v2(
        self, u, initial_state, setpoint, dt, T_env, T_cold, v_f, T_filament
    ):
        """
        MPC目标函数

        计算给定控制序列下的目标函数值，用于优化求解
        优先使用Numba加速版本，回退到纯Python版本

        目标函数组成:
            J = w_t * sum((T_s - T_target)^2)    # 跟踪误差
              + w_c * sum(u^2)                    # 控制量惩罚
              + w_r * sum((du)^2)                 # 控制变化率惩罚

        参数:
            u: 控制序列 (控制时域内的控制量，长度为Nc)
            initial_state: 初始状态字典 {'T_h', 'T_b', 'T_s'}
            setpoint: 目标温度 (°C)
            dt: 时间步长 (s)
            T_env: 环境温度 (°C)
            T_cold: 冷端温度 (°C)
            v_f: 挤出速率 (mm³/s)
            T_filament: 耗材温度 (°C)

        返回:
            目标函数值 (float)
        """
        Np = self.const_prediction_horizon_v2
        Nc = self.const_control_horizon_v2

        if self.numba_enabled_v2:
            return _numba_mpc_objective(
                u,
                initial_state["T_h"],
                initial_state["T_b"],
                initial_state["T_s"],
                setpoint,
                dt,
                T_env,
                T_cold,
                v_f,
                T_filament,
                self.const_theta_1_v2,
                self.const_theta_2_v2,
                self.const_theta_3_v2,
                self.const_theta_4_v2,
                self.const_theta_5_v2,
                self.const_theta_6_v2,
                self.const_theta_7_v2,
                self.const_theta_8_v2,
                self.const_theta_9_v2,
                self.const_filament_cross_section_heat_capacity_v2,
                Np,
                Nc,
                self.const_weight_tracking_v2,
                self.const_weight_terminal_v2,
                self.const_weight_rate_v2,
                self.last_control_v2,
            )

        u_full = np.zeros(Np)
        for i in range(Np):
            u_full[i] = u[min(i, Nc - 1)]

        _, _, T_s_pred = self._predict_trajectory_v2(
            initial_state, u_full, dt, T_env, T_cold, v_f, T_filament
        )

        tracking_error = np.sum(
            self.const_weight_tracking_v2 * (T_s_pred[1:] - setpoint) ** 2
        )
        terminal_cost = (
            self.const_weight_terminal_v2 * (T_s_pred[Np] - setpoint) ** 2
        )

        rate_penalty = (
            self.const_weight_rate_v2 * (u[0] - self.last_control_v2) ** 2
        )
        if Nc > 1:
            rate_penalty += np.sum(
                self.const_weight_rate_v2 * np.diff(u[:Nc]) ** 2
            )

        total_cost = tracking_error + terminal_cost + rate_penalty

        return total_cost

    def _solve_mpc_v2(
        self, initial_state, setpoint, dt, T_env, T_cold, v_f, T_filament
    ):
        """
        求解MPC优化问题

        使用L-BFGS-B算法求解带约束的优化问题，找到最优控制序列

        参数:
            initial_state: 初始状态字典 {'T_h', 'T_b', 'T_s'}
            setpoint: 目标温度 (°C)
            dt: 时间步长 (s)
            T_env: 环境温度 (°C)
            T_cold: 冷端温度 (°C)
            v_f: 挤出速率 (mm³/s)
            T_filament: 耗材温度 (°C)

        返回:
            最优控制量 (功率 W)
        """
        t_start = time.perf_counter()

        from scipy.optimize import minimize

        Nc = self.const_control_horizon_v2
        max_power = self.heater_max_power_v2
        min_power = 0.0

        T_s = initial_state["T_s"]
        T_b = initial_state["T_b"]
        temp_error = setpoint - T_s

        # logging.debug(
        #    f"MPC V2: setpoint={setpoint:.1f}, T_s={T_s:.1f}, T_b={T_b:.1f}, "
        #    f"temp_error={temp_error:.1f}, max_power={max_power:.1f}"
        # )

        # 简单回退控制：当温度误差很大时使用简单比例控制
        if temp_error > 100:
            logging.debug("MPC V2: Large temp error, using full power")
            return max_power
        elif temp_error < -50:
            logging.debug("MPC V2: Temp overshoot, turning off heater")
            return 0.0

        # 根据温度误差确定初始猜测值
        if temp_error > 50:
            u0 = np.ones(Nc) * max_power * 0.8
        elif temp_error > 20:
            u0 = np.ones(Nc) * max_power * 0.5
        elif temp_error > 5:
            u0 = np.ones(Nc) * max_power * 0.3
        elif temp_error < -5:
            u0 = np.ones(Nc) * min_power
        else:
            u0 = np.ones(Nc) * max(self.last_control_v2, max_power * 0.1)

        bounds = [(min_power, max_power)] * Nc

        try:
            result = minimize(
                self._mpc_objective_v2,
                u0,
                args=(
                    initial_state,
                    setpoint,
                    dt,
                    T_env,
                    T_cold,
                    v_f,
                    T_filament,
                ),
                method="L-BFGS-B",
                bounds=bounds,
                options={
                    "maxiter": self.const_max_iterations_v2,
                    "ftol": self.const_tolerance_v2,
                },
            )
            optimal_control = result.x[0]
            logging.debug(
                f"MPC V2 optimization: success={result.success}, "
                f"optimal_power={optimal_control:.2f}W, iterations={result.nit}"
            )
        except Exception as e:
            logging.warning(
                f"MPC V2 optimization failed: {e}, using fallback control"
            )
            # 回退到简单比例控制
            Kp = 0.5  # 简单的比例系数
            optimal_control = min(
                max_power, max(0.0, temp_error * Kp * max_power / 50.0)
            )
            logging.debug(f"MPC V2 fallback: power={optimal_control:.2f}W")

        optimal_control = np.clip(optimal_control, min_power, max_power)

        self.last_control_v2 = optimal_control
        self.control_history_v2.append(optimal_control)

        self.timing_optimize_v2 = (time.perf_counter() - t_start) * 1000.0

        return optimal_control

    def temperature_update(self, read_time, temp, target_temp):
        """
        温度更新主循环 - 每个控制周期调用一次

        执行以下步骤:
            1. 验证配置有效性
            2. 计算时间步长
            3. 获取挤出速率
            4. 更新传感器温度
            5. EKF状态估计（替代简单的模型预测+校正）
            6. MPC优化求解（使用投影梯度下降法）
            7. 输出控制信号

        参数:
            read_time: 当前时间 (s)
            temp: 当前测量的传感器温度 (°C)
            target_temp: 目标温度 (°C)
        """
        t_total_start = time.perf_counter()

        if not self.is_valid_v2():
            self.heater_v2.set_pwm(read_time, 0.0)
            return

        dt = read_time - self.last_temp_time_v2
        if self.last_temp_time_v2 == 0.0 or dt < 0.0 or dt > 1.0:
            dt = 0.1

        extrude_speed = 0.0
        if target_temp != 0.0:
            if self.toolhead_v2 is None:
                self.toolhead_v2 = self.printer_v2.lookup_object("toolhead")
            if self.toolhead_v2 is not None:
                extruder = self.toolhead_v2.get_extruder()
                if (
                    hasattr(extruder, "find_past_position")
                    and extruder.get_heater() == self.heater_v2
                ):
                    pos = extruder.find_past_position(read_time)
                    pos_prev = extruder.find_past_position(read_time - dt)
                    pos_moved = max(
                        -self.const_maximum_retract_v2, pos - pos_prev
                    )
                    extrude_speed = pos_moved / dt

        if self.want_ambient_refresh_v2 and self.ambient_sensor_v2 is not None:
            temp_amb = self.ambient_sensor_v2.get_temp(read_time)[0]
            if temp_amb != 0.0:
                self.state_ambient_temp_v2 = temp_amb
                self.want_ambient_refresh_v2 = False

        if self.cold_temp_sensor_v2 is not None:
            temp_cold = self.cold_temp_sensor_v2.get_temp(read_time)[0]
            if temp_cold != 0.0:
                self.state_cold_temp_v2 = temp_cold

        T_env = self.state_ambient_temp_v2
        T_cold = self.state_cold_temp_v2
        T_filament = self.filament_temp_v2(read_time, T_env)

        # =====================================================================
        # EKF状态估计
        # Extended Kalman Filter State Estimation
        # =====================================================================
        t_ekf_start = time.perf_counter()
        t_model_step_start = time.perf_counter()

        if self.ekf_enabled_v2 and self.numba_enabled_v2:
            self.ekf_Q_v2 = np.eye(3) * self.ekf_Q_scale_v2
            self.ekf_R_v2 = np.array([[self.ekf_R_scale_v2]])

            x_new, P_new = _ekf_step(
                self.ekf_state_v2,
                self.ekf_P_v2,
                temp,
                self.last_power_v2,
                dt,
                T_env,
                T_cold,
                extrude_speed,
                T_filament,
                self.ekf_Q_v2,
                self.ekf_R_v2,
                self.const_theta_1_v2,
                self.const_theta_2_v2,
                self.const_theta_3_v2,
                self.const_theta_4_v2,
                self.const_theta_5_v2,
                self.const_theta_6_v2,
                self.const_theta_7_v2,
                self.const_theta_8_v2,
                self.const_theta_9_v2,
                self.const_filament_cross_section_heat_capacity_v2,
            )

            self.timing_model_step_v2 = (
                time.perf_counter() - t_model_step_start
            ) * 1000.0

            self.ekf_state_v2 = x_new
            self.ekf_P_v2 = P_new

            self.state_heater_temp_v2 = x_new[0]
            self.state_block_temp_v2 = x_new[1]
            self.state_sensor_temp_v2 = x_new[2]
        else:
            T_h = self.state_heater_temp_v2
            T_b = self.state_block_temp_v2
            T_s = self.state_sensor_temp_v2

            power = self.last_power_v2
            T_h_new, T_b_new, T_s_new = self._model_step_v2(
                T_h,
                T_b,
                T_s,
                power,
                dt,
                T_env,
                T_cold,
                extrude_speed,
                T_filament,
            )

            self.state_heater_temp_v2 = T_h_new
            self.state_block_temp_v2 = T_b_new
            self.state_sensor_temp_v2 = T_s_new

            smoothing = 1 - (1 - self.const_smoothing_v2) ** dt
            adjustment_dT = (temp - self.state_sensor_temp_v2) * smoothing
            self.state_heater_temp_v2 += adjustment_dT
            self.state_block_temp_v2 += adjustment_dT
            self.state_sensor_temp_v2 += adjustment_dT

            if (
                self.last_power_v2 > 0
                and self.last_power_v2 < self.heater_max_power_v2
            ) or abs(adjustment_dT) < self.const_steady_state_rate_v2 * dt:
                if adjustment_dT > 0.0:
                    ambient_delta = max(
                        adjustment_dT, self.const_min_ambient_change_v2 * dt
                    )
                else:
                    ambient_delta = min(
                        adjustment_dT, -self.const_min_ambient_change_v2 * dt
                    )
                self.state_ambient_temp_v2 += ambient_delta

        self.timing_ekf_v2 = (time.perf_counter() - t_ekf_start) * 1000.0

        # =====================================================================
        # MPC优化求解 - 投影梯度下降法
        # MPC Optimization - Projected Gradient Descent
        # =====================================================================

        if target_temp != 0.0:
            t_optimize_start = time.perf_counter()
            t_predict_start = time.perf_counter()

            if self._hot_start_available_v2:
                u_init = self._u_prev_v2.copy()
                for i in range(len(u_init) - 1):
                    u_init[i] = u_init[i + 1]
                u_init[-1] = u_init[-2]
            else:
                u_init = np.full(
                    self.const_control_horizon_v2,
                    self.last_power_v2 if self.last_power_v2 > 0 else 0.0,
                )

            if self.numba_enabled_v2:
                u_opt, converged, iterations = _solve_mpc_pgd(
                    self.state_heater_temp_v2,
                    self.state_block_temp_v2,
                    self.state_sensor_temp_v2,
                    target_temp,
                    dt,
                    self.state_ambient_temp_v2,
                    self.state_cold_temp_v2,
                    extrude_speed,
                    T_filament,
                    self.const_theta_1_v2,
                    self.const_theta_2_v2,
                    self.const_theta_3_v2,
                    self.const_theta_4_v2,
                    self.const_theta_5_v2,
                    self.const_theta_6_v2,
                    self.const_theta_7_v2,
                    self.const_theta_8_v2,
                    self.const_theta_9_v2,
                    self.const_filament_cross_section_heat_capacity_v2,
                    self.const_prediction_horizon_v2,
                    self.const_control_horizon_v2,
                    self.const_weight_tracking_v2,
                    self.const_weight_terminal_v2,
                    self.const_weight_rate_v2,
                    self.last_power_v2,
                    self.heater_max_power_v2,
                    0.0,
                    self.pgd_max_iterations_v2,
                    self.pgd_tolerance_v2,
                    u_init,
                )

                self.timing_predict_v2 = (
                    time.perf_counter() - t_predict_start
                ) * 1000.0
                self.timing_optimize_v2 = (
                    time.perf_counter() - t_optimize_start
                ) * 1000.0

                power = u_opt[0]
                self._u_prev_v2 = u_opt.copy()
                self._hot_start_available_v2 = True
                self.pgd_iterations_v2 = iterations
                self.pgd_converged_v2 = converged
            else:
                initial_state = {
                    "T_h": self.state_heater_temp_v2,
                    "T_b": self.state_block_temp_v2,
                    "T_s": self.state_sensor_temp_v2,
                }

                power = self._solve_mpc_v2(
                    initial_state,
                    target_temp,
                    dt,
                    T_env,
                    T_cold,
                    extrude_speed,
                    T_filament,
                )
        else:
            power = 0.0
            self._hot_start_available_v2 = False

        duty = power / self.const_heater_power_v2

        logging.debug(
            f"MPC V2 output: power={power:.2f}W, duty={duty:.3f}, "
            f"heater_power={self.const_heater_power_v2:.1f}W, "
            f"target={target_temp:.1f}, temp={temp:.1f}"
        )

        T_b = self.state_block_temp_v2
        T_env = self.state_ambient_temp_v2
        T_cold = self.state_cold_temp_v2
        T_b_K = T_b + 273.15
        T_env_K = T_env + 273.15

        self.last_loss_ambient_v2 = self.const_theta_5_v2 * (T_b - T_env)
        self.last_loss_cold_v2 = self.const_theta_6_v2 * (T_b - T_cold)
        self.last_loss_radiation_v2 = self.const_theta_7_v2 * (
            T_b_K**4 - T_env_K**4
        )
        self.last_loss_filament_v2 = (
            extrude_speed
            * self.const_filament_cross_section_heat_capacity_v2
            * self.const_theta_9_v2
            * (T_b - T_filament)
        )

        self.last_power_v2 = power
        self.last_temp_time_v2 = read_time
        self.heater_v2.set_pwm(read_time, duty)

        self.timing_total_v2 = (time.perf_counter() - t_total_start) * 1000.0
        self.timing_count_v2 += 1

        if self.timing_total_v2 > self.timing_max_v2:
            self.timing_max_v2 = self.timing_total_v2

        alpha = 0.1
        self.timing_avg_v2 = (
            alpha * self.timing_total_v2 + (1 - alpha) * self.timing_avg_v2
        )

    def filament_temp_v2(self, read_time, ambient_temp):
        """
        获取耗材温度

        根据配置的耗材温度来源返回温度值:
            - 'fixed': 使用固定温度值
            - 'sensor': 使用传感器读数
            - 'ambient': 使用环境温度

        参数:
            read_time: 当前时间 (s)
            ambient_temp: 环境温度 (°C)

        返回:
            耗材温度 (°C)
        """
        src = self.filament_temp_src_v2
        if src[0] == FILAMENT_TEMP_SRC_FIXED:
            return src[1]
        elif (
            src[0] == FILAMENT_TEMP_SRC_SENSOR
            and self.ambient_sensor_v2 is not None
        ):
            return self.ambient_sensor_v2.get_temp(read_time)[0]
        else:
            return ambient_temp

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        """
        检查加热器是否仍在加热中

        当温度与目标相差超过1°C时返回True

        参数:
            eventtime: 当前事件时间
            smoothed_temp: 平滑后的温度
            target_temp: 目标温度

        返回:
            bool: 是否仍在加热中
        """
        return abs(target_temp - smoothed_temp) > 1.0

    def update_smooth_time(self):
        """更新平滑时间（MPC不需要此功能）"""
        pass

    def get_profile(self):
        """获取当前配置参数字典"""
        return self.profile_v2

    def get_type(self):
        """获取控制器类型标识"""
        return "mpc_v2"

    def get_status(self, eventtime):
        """
        获取控制器状态信息

        返回包含以下信息的状态字典:
            - 温度状态: temp_heater, temp_block, temp_sensor, temp_ambient, temp_cold
            - 控制状态: power
            - 热损耗: loss_ambient, loss_filament, loss_cold, loss_radiation
            - 耗材信息: filament_temp, filament_heat_capacity, filament_density
            - MPC参数: theta_1 ~ theta_9, prediction_horizon, control_horizon, weights
            - EKF状态: ekf_enabled, ekf_T_h, ekf_T_b, ekf_T_s, ekf_P_diag
            - PGD状态: pgd_iterations, pgd_converged
            - 性能统计: timing_model_step, timing_predict, timing_optimize, timing_total, timing_max, timing_avg
            - Numba状态: numba_enabled

        参数:
            eventtime: 当前事件时间

        返回:
            dict: 状态信息字典
        """
        return {
            "temp_heater": self.state_heater_temp_v2,
            "temp_block": self.state_block_temp_v2,
            "temp_sensor": self.state_sensor_temp_v2,
            "temp_ambient": self.state_ambient_temp_v2,
            "temp_cold": self.state_cold_temp_v2,
            "power": self.last_power_v2,
            "loss_ambient": self.last_loss_ambient_v2,
            "loss_filament": self.last_loss_filament_v2,
            "loss_cold": self.last_loss_cold_v2,
            "loss_radiation": self.last_loss_radiation_v2,
            "filament_temp": self.filament_temp_src_v2,
            "filament_heat_capacity": self.const_filament_heat_capacity_v2,
            "filament_density": self.const_filament_density_v2,
            "theta_1": self.const_theta_1_v2,
            "theta_2": self.const_theta_2_v2,
            "theta_3": self.const_theta_3_v2,
            "theta_4": self.const_theta_4_v2,
            "theta_5": self.const_theta_5_v2,
            "theta_6": self.const_theta_6_v2,
            "theta_7": self.const_theta_7_v2,
            "theta_8": self.const_theta_8_v2,
            "theta_9": self.const_theta_9_v2,
            "prediction_horizon": self.const_prediction_horizon_v2,
            "control_horizon": self.const_control_horizon_v2,
            "weight_tracking": self.const_weight_tracking_v2,
            "weight_terminal": self.const_weight_terminal_v2,
            "weight_rate": self.const_weight_rate_v2,
            "ekf_enabled": self.ekf_enabled_v2,
            "ekf_T_h": float(self.ekf_state_v2[0]),
            "ekf_T_b": float(self.ekf_state_v2[1]),
            "ekf_T_s": float(self.ekf_state_v2[2]),
            "ekf_P_diag": [
                float(self.ekf_P_v2[0, 0]),
                float(self.ekf_P_v2[1, 1]),
                float(self.ekf_P_v2[2, 2]),
            ],
            "ekf_Q_scale": self.ekf_Q_scale_v2,
            "ekf_R_scale": self.ekf_R_scale_v2,
            "pgd_iterations": self.pgd_iterations_v2,
            "pgd_converged": self.pgd_converged_v2,
            "hot_start_available": self._hot_start_available_v2,
            "timing_model_step": self.timing_model_step_v2,
            "timing_predict": self.timing_predict_v2,
            "timing_optimize": self.timing_optimize_v2,
            "timing_ekf": self.timing_ekf_v2,
            "timing_total": self.timing_total_v2,
            "timing_max": self.timing_max_v2,
            "timing_avg": self.timing_avg_v2,
            "timing_count": self.timing_count_v2,
            "numba_enabled": self.numba_enabled_v2,
        }


# =============================================================================
# 性能基准测试函数
# Performance Benchmark Functions
# =============================================================================


def run_performance_benchmark(n_iterations=100):
    """
    运行性能基准测试

    比较Numba优化前后的计算性能

    参数:
        n_iterations: 测试迭代次数

    返回:
        dict: 包含各函数执行时间的字典
    """
    import time as time_module

    results = {}

    theta_1, theta_2, theta_3 = 5.029312e-02, 2.806417e-01, 1.065468e-02
    theta_4, theta_5, theta_6 = 1.370236e-01, 3.195262e-03, 2.327857e-02
    theta_7, theta_8, theta_9 = 2.571527e-11, 8.000314e-02, 4.458e-01
    c_p = 0.00259

    T_h, T_b, T_s = 200.0, 195.0, 190.0
    power = 30.0
    dt = 0.1
    T_env, T_cold = 25.0, 25.0
    v_f, T_filament = 0.0, 25.0

    Np, Nc = 30, 10
    w_t, w_c, w_r = 10.0, 0.001, 0.1
    last_control = 30.0
    max_power = 100.0

    if NUMBA_AVAILABLE:
        _numba_model_step(
            T_h,
            T_b,
            T_s,
            power,
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
        )

        t_start = time_module.perf_counter()
        for _ in range(n_iterations):
            _numba_model_step(
                T_h,
                T_b,
                T_s,
                power,
                dt,
                T_env,
                T_cold,
                v_f,
                T_filament,
                theta_1,
                theta_2,
                theta_3,
                theta_4,
                theta_5,
                theta_6,
                theta_7,
                theta_8,
                theta_9,
                c_p,
            )
        t_end = time_module.perf_counter()
        results["model_step_numba"] = (t_end - t_start) / n_iterations * 1000

        u_test = np.full(Nc, 30.0)

        _numba_mpc_objective(
            u_test,
            T_h,
            T_b,
            T_s,
            200.0,
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
            Np,
            Nc,
            w_t,
            w_c,
            w_r,
            last_control,
        )

        t_start = time_module.perf_counter()
        for _ in range(n_iterations):
            _numba_mpc_objective(
                u_test,
                T_h,
                T_b,
                T_s,
                200.0,
                dt,
                T_env,
                T_cold,
                v_f,
                T_filament,
                theta_1,
                theta_2,
                theta_3,
                theta_4,
                theta_5,
                theta_6,
                theta_7,
                theta_8,
                theta_9,
                c_p,
                Np,
                Nc,
                w_t,
                w_c,
                w_r,
                last_control,
            )
        t_end = time_module.perf_counter()
        results["mpc_objective_numba"] = (t_end - t_start) / n_iterations * 1000

        x_ekf = np.array([T_h, T_b, T_s])
        P_ekf = np.eye(3) * 10.0
        Q_ekf = np.eye(3) * 0.1
        R_ekf = np.array([[0.25]])

        _ekf_step(
            x_ekf,
            P_ekf,
            T_s,
            power,
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            Q_ekf,
            R_ekf,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
        )

        t_start = time_module.perf_counter()
        for _ in range(n_iterations):
            _ekf_step(
                x_ekf,
                P_ekf,
                T_s,
                power,
                dt,
                T_env,
                T_cold,
                v_f,
                T_filament,
                Q_ekf,
                R_ekf,
                theta_1,
                theta_2,
                theta_3,
                theta_4,
                theta_5,
                theta_6,
                theta_7,
                theta_8,
                theta_9,
                c_p,
            )
        t_end = time_module.perf_counter()
        results["ekf_step_numba"] = (t_end - t_start) / n_iterations * 1000

        u_init = np.full(Nc, 30.0)

        t_start = time_module.perf_counter()
        u_opt, converged, iterations = _solve_mpc_pgd(
            T_h,
            T_b,
            T_s,
            200.0,
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
            Np,
            Nc,
            w_t,
            w_c,
            w_r,
            last_control,
            max_power,
            0.0,
            100,
            1e-3,
            u_init,
        )
        t_end = time_module.perf_counter()
        results["mpc_solve_pgd_numba"] = (t_end - t_start) * 1000
        results["pgd_iterations"] = iterations
        results["pgd_converged"] = converged

        t_start = time_module.perf_counter()
        u_opt2, converged2, iterations2 = _solve_mpc_pgd(
            T_h,
            T_b,
            T_s,
            200.0,
            dt,
            T_env,
            T_cold,
            v_f,
            T_filament,
            theta_1,
            theta_2,
            theta_3,
            theta_4,
            theta_5,
            theta_6,
            theta_7,
            theta_8,
            theta_9,
            c_p,
            Np,
            Nc,
            w_t,
            w_c,
            w_r,
            last_control,
            max_power,
            0.0,
            100,
            1e-3,
            u_opt,
        )
        t_end = time_module.perf_counter()
        results["mpc_solve_hotstart_numba"] = (t_end - t_start) * 1000
        results["pgd_iterations_hotstart"] = iterations2
        results["pgd_converged_hotstart"] = converged2

        results["numba_available"] = True
    else:
        results["numba_available"] = False
        results["error"] = "Numba not available for benchmarking"

    return results


def print_benchmark_results():
    """打印性能基准测试结果"""
    print("=" * 60)
    print("MPC V2 性能基准测试结果")
    print("=" * 60)

    results = run_performance_benchmark(n_iterations=100)

    if not results.get("numba_available", False):
        print("Numba 不可用，无法运行基准测试")
        return

    print(
        f"\n单步模型预测 (model_step):     {results['model_step_numba']:.4f} ms"
    )
    print(
        f"MPC目标函数 (mpc_objective):   {results['mpc_objective_numba']:.4f} ms"
    )
    print(f"EKF状态更新 (ekf_step):        {results['ekf_step_numba']:.4f} ms")
    print(
        f"\nMPC求解 (冷启动):              {results['mpc_solve_pgd_numba']:.4f} ms"
    )
    print(f"  - 迭代次数: {results['pgd_iterations']}")
    print(f"  - 收敛状态: {'是' if results['pgd_converged'] else '否'}")
    print(
        f"\nMPC求解 (热启动):              {results['mpc_solve_hotstart_numba']:.4f} ms"
    )
    print(f"  - 迭代次数: {results['pgd_iterations_hotstart']}")
    print(
        f"  - 收敛状态: {'是' if results['pgd_converged_hotstart'] else '否'}"
    )

    print("\n" + "=" * 60)
    print("总控制周期预估时间:")
    total_cold = results["ekf_step_numba"] + results["mpc_solve_pgd_numba"]
    total_hot = results["ekf_step_numba"] + results["mpc_solve_hotstart_numba"]
    print(f"  冷启动: EKF + MPC求解 = {total_cold:.4f} ms")
    print(f"  热启动: EKF + MPC求解 = {total_hot:.4f} ms")
    print("=" * 60)


if __name__ == "__main__":
    print_benchmark_results()

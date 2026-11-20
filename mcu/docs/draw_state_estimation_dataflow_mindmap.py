from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


OUT = Path(__file__).with_name("state_estimation_dataflow_mindmap.png")


def choose_cjk_font():
    preferred = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Source Han Sans SC"]
    installed = {font.name for font in font_manager.fontManager.ttflist}
    for name in preferred:
        if name in installed:
            return name
    return "DejaVu Sans"


plt.rcParams["font.family"] = [choose_cjk_font(), "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

fig, ax = plt.subplots(figsize=(20, 15), dpi=160)
fig.patch.set_facecolor("#f4f7fb")
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.axis("off")

COLORS = {
    "wheel": {"edge": "#1570ef", "face": "#eff6ff", "title": "#175cd3"},
    "imu": {"edge": "#7a5af8", "face": "#f4f3ff", "title": "#6941c6"},
    "laser": {"edge": "#039855", "face": "#ecfdf3", "title": "#027a48"},
    "auto": {"edge": "#dc6803", "face": "#fffaeb", "title": "#b54708"},
    "output": {"edge": "#344054", "face": "#f2f4f7", "title": "#1d2939"},
}


def box(x, y, w, h, title, body, kind, title_size=12.2, body_size=10.4):
    c = COLORS[kind]
    patch = FancyBboxPatch((x, y), w, h,
                           boxstyle="round,pad=0.3,rounding_size=1.1",
                           linewidth=1.8, edgecolor=c["edge"], facecolor=c["face"], zorder=2)
    ax.add_patch(patch)
    ax.text(x + w / 2, y + h - 1.35, title, ha="center", va="top",
            fontsize=title_size, fontweight="bold", color=c["title"], zorder=3)
    ax.text(x + w / 2, y + h - 3.35, body, ha="center", va="top",
            fontsize=body_size, color="#263449", linespacing=1.45, zorder=3)
    return {"x": x, "y": y, "w": w, "h": h,
            "top": (x + w / 2, y + h), "bottom": (x + w / 2, y),
            "left": (x, y + h / 2), "right": (x + w, y + h / 2)}


def arrow(start, end, color="#667085", style="-|>", connectionstyle="arc3,rad=0"):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=15,
                                linewidth=1.7, color=color, connectionstyle=connectionstyle,
                                zorder=1))


def lane_header(x, text, kind):
    c = COLORS[kind]
    ax.text(x + 15, 82.7, text, ha="center", va="center", fontsize=16,
            fontweight="bold", color=c["title"],
            bbox=dict(boxstyle="round,pad=0.4", facecolor=c["face"], edgecolor=c["edge"]))


ax.text(50, 97.2, "当前状态估计数据流与核心公式", ha="center", va="center",
        fontsize=27, fontweight="bold", color="#172033")
ax.text(50, 94.5, "思维导图式链路：轮速位置 × IMU 航向 × 激光横移｜对应当前未提交的纯 IMU 航向环代码",
        ha="center", va="center", fontsize=13, color="#667085")

schedule = box(4, 86.5, 92, 5.8, "调度入口：entry_loop() → app_runtime",
               "主循环／底盘反馈／Auto 控制 500 Hz（名义 Δt=0.002 s）　·　odom.process() 250 Hz　·　遥控 100 Hz",
               "output", title_size=12.5, body_size=11)

lane_header(3, "① 轮速里程计：位置与路线进度", "wheel")
lane_header(35, "② BMI088：独立姿态与航向反馈", "imu")
lane_header(67, "③ 四路激光：横移、就位与安全", "laser")

wheel_src = box(3, 72.5, 30, 7.5, "八路电机反馈 → 底盘观测",
                "4 舵向角 θᵢ＋4 驱动轮速 ωᵢ\nchassis_refresh_observation()\n输出：ChassisObservation", "wheel")
wheel_fk = box(3, 55.5, 30, 14.5, "swheel.fk()：四轮正运动学",
               "vᵢₓ=rωᵢcosθᵢ，vᵢᵧ=rωᵢsinθᵢ\nvx=Σvᵢₓ/4，vy=Σvᵢᵧ/4\nwz=Σ(−yᵢvᵢₓ+xᵢvᵢᵧ)/Σ(xᵢ²+yᵢ²)\n经验偏置：b=k_vx·vx−k_vy·vy\nk_vx=−0.069，k_vy=0.03\n反馈 wz=wz_FK+b；逆解 wz_IK=wz_cmd−b", "wheel", body_size=9.9)
wheel_odom = box(3, 36.5, 30, 16.5, "odom_process_wheels()：梯形积分",
                 "Δψ=(wz_prev+wz)Δt/2，ψ_mid=ψ+Δψ/2\nΔx_b=(vx_prev+vx)Δt/2\nΔy_b=(vy_prev+vy)Δt/2\nx+=cosψ_mid·Δx_b−sinψ_mid·Δy_b\ny+=sinψ_mid·Δx_b+cosψ_mid·Δy_b\nψ=wrap(ψ+Δψ)\n输出：odom.get_odom()/get_angle()/get_velocity()", "wheel", body_size=9.9)

imu_src = box(35, 73.5, 30, 6.5, "SPI2 BMI088 → imu.update()",
              "输入：三轴加速度、三轴陀螺、温度\n原始轴直接赋值 x/y/z，无安装旋转矩阵", "imu", body_size=10.2)
imu_cal = box(35, 62.5, 30, 8.5, "启动校准＋零偏／温漂补偿",
              "2000 个静止样本，检查方差；静止时学习残差\nω_corr=ω_raw−(b_startup+b_temp+b_residual)\nz 轴温漂系数 0.000038；输出修正角速度", "imu", body_size=10.1)
imu_mahony = box(35, 44.5, 30, 15.5, "imu_attitude_update_mahony()：六轴姿态",
                 "a=a_raw/|a_raw|，e=a×g_est(q)\ngx=ωcorr_x+kp·e_x，gy=ωcorr_y+kp·e_y\ngz=ωcorr_z；kp=2，ki=ki_z=0\nq+=0.5·q⊗[0,(gx,gy,gz)]·Δt\n随后归一化 q，并转 roll/pitch/yaw\n加速度仅校正 roll/pitch；无磁力计绝对 yaw", "imu", body_size=10.0)
imu_out = box(35, 36.5, 30, 5.5, "imu.get_angle()／imu_get_gyro_corrected()",
              "输出 ψ_IMU；imu_wz=(ω_y·sin roll+ω_z·cos roll)/cos pitch", "imu", body_size=9.9)

laser_src = box(67, 72.5, 30, 7.5, "CAN3 标准帧 0x601 → 四束距离",
                "assemble_laser_receive()/process()\n8 字节大端 uint16：distance1~4=F/B/L/R\n输出 LaserOdomSample，单位 mm", "laser")
laser_gate = box(67, 59.5, 30, 10.5, "laser_odom_receive()＋路线门控",
                 "有效量程 0~5000 mm；CAN 在线超时 250 ms\n控制帧龄≤120 ms\nMAP_ROUTE_LASER_GUIDANCE_ENABLED=1\n双束窗口：F+B=50~125 mm\n01 段先等左束≤105 mm，再锁存本段侧束引导", "laser", body_size=9.5)
laser_ctrl = box(67, 35.5, 30, 21.5, "route_lateral_target()/route_front_target()",
                 "死区：e=d−target；|e|≤3 mm 时 e=0，否则减去 3 mm\n双束横跨：corr_x=0.5(e_REAR−e_FRONT)×1.0×0.001\n目标 F=70±3 mm／B=45±3 mm；限幅 0.06 m/s\n侧束换沟：corr_y=s·e×2.5×0.001；限幅 0.09 m/s\nRIGHT s=+1；LEFT s=−1；目标 45±3 mm\n仅 01：F 单独 corr_x=−e_FRONT×1.0×0.001\n04/06/08/10/12/14：轮速 X＋侧束 Y，F 不做 70 mm 就位\n所需侧束及 01 前束无效时等待，不允许跳过换段\n纠偏增速限幅 0.30 m/s²；双通道独立限增速\n逃离增益 2.5、限幅 0.09 m/s，保持即时响应\n前束安全 55 mm；其余指定束 30 mm\n输出只叠加 vx/vy，不修改里程计坐标", "laser", body_size=8.2)

arrow(schedule["bottom"], wheel_src["top"], COLORS["wheel"]["edge"], connectionstyle="arc3,rad=0.13")
arrow(schedule["bottom"], imu_src["top"], COLORS["imu"]["edge"])
arrow(schedule["bottom"], laser_src["top"], COLORS["laser"]["edge"], connectionstyle="arc3,rad=-0.13")
arrow(wheel_src["bottom"], wheel_fk["top"], COLORS["wheel"]["edge"])
arrow(wheel_fk["bottom"], wheel_odom["top"], COLORS["wheel"]["edge"])
arrow(imu_src["bottom"], imu_cal["top"], COLORS["imu"]["edge"])
arrow(imu_cal["bottom"], imu_mahony["top"], COLORS["imu"]["edge"])
arrow(imu_mahony["bottom"], imu_out["top"], COLORS["imu"]["edge"])
arrow(laser_src["bottom"], laser_gate["top"], COLORS["laser"]["edge"])
arrow(laser_gate["bottom"], laser_ctrl["top"], COLORS["laser"]["edge"])

auto = box(13, 20.5, 74, 13.5, "Auto 汇合：app_odom_test_process() → route_send_command()",
           "轮速：位置、速度、轮速 ψ 与段进度｜IMU：ψ_IMU、imu_wz｜激光：横移／就位／安全逃离\n"
           "自主首帧有效 IMU：chassis_yaw_hold_set_target(ψ_IMU)，整趟锁存，不逐段重置\n"
           "eψ=wrap(ψ_ref−ψ_IMU)；wz=sat(8·eψ−0.25·imu_wz, ±0.55 rad/s)，路线原始 wz_route=0\n"
           "轮速起始 ψ 仅建立路线坐标；路线速度用 IMU ψ 转回车体坐标；两平移轴均≤0.01 m/s 时航向纠偏为 0",
           "auto", title_size=13.0, body_size=10.7)
arrow(wheel_odom["bottom"], (26, 34.2), COLORS["wheel"]["edge"])
arrow((26, 34.2), auto["top"], COLORS["wheel"]["edge"])
arrow(imu_out["bottom"], auto["top"], COLORS["imu"]["edge"])
arrow(laser_ctrl["bottom"], (74, 34.2), COLORS["laser"]["edge"])
arrow((74, 34.2), auto["top"], COLORS["laser"]["edge"])

output = box(13, 5.0, 74, 12.5, "底盘输出：chassis.set_velocity(vx,vy,wz) → swheel.ik()",
             "每个舵轮：vᵢₓ=vx−wz·yᵢ，vᵢᵧ=vy+wz·xᵢ\n"
             "轮速 ωᵢ=hypot(vᵢₓ,vᵢᵧ)/r；舵角 θᵢ=atan2(vᵢᵧ,vᵢₓ)\n"
             "输出四轮舵角与轮速指令，再经电机反馈进入下一轮正解",
             "output", title_size=13.0, body_size=11.2)
arrow(auto["bottom"], output["top"], COLORS["auto"]["edge"])

ax.text(50, 2.0,
        "当前边界：wheel_only=true 仅表示位姿由轮速维护；IMU 仍独立采样并供 Auto 航向环。Kalman 保留但不运行；IMU／激光均不回写轮速位置。",
        ha="center", va="center", fontsize=11.5, color="#b42318", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.45", facecolor="#fff1f0", edgecolor="#fda29b"))

plt.savefig(OUT, bbox_inches="tight", facecolor=fig.get_facecolor())
print(OUT.resolve())

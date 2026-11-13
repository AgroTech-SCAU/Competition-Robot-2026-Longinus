const BASELINE = "0059547cff11c99a675f6e17420e2803164e078b";
const DRAFT_KEY = "longinus-workbench-calibration-v1";
const RADIANS = Math.PI / 180;
const WHEELS = [
  { code: "FL", name: "左前" },
  { code: "FR", name: "右前" },
  { code: "RR", name: "右后" },
  { code: "RL", name: "左后" },
];
const AXES = ["X", "Y", "Z"];
const BEAMS = [
  { code: "front", name: "前" },
  { code: "rear", name: "后" },
  { code: "left", name: "左" },
  { code: "right", name: "右" },
];

const finite = Number.isFinite;
const toRadians = (degrees) => finite(degrees) ? degrees * RADIANS : null;
const toMetres = (millimetres) => finite(millimetres) ? millimetres / 1000 : null;
const angleWrap = (angle) => Math.atan2(Math.sin(angle), Math.cos(angle));

export function fitSteering(raw) {
  const { zeroRad, positive90Rad, negative90Rad } = raw;
  if (![zeroRad, positive90Rad, negative90Rad].every(finite)) return null;
  const plus = positive90Rad - zeroRad;
  const minus = negative90Rad - zeroRad;
  const rawPerMechanicalRad = (plus - minus) / Math.PI;
  if (plus * minus >= 0 || Math.abs(rawPerMechanicalRad) < 1e-6) return null;
  const sign = Math.sign(rawPerMechanicalRad);
  const radiansPerRawRadian = 1 / Math.abs(rawPerMechanicalRad);
  const residualRad = [
    angleWrap(sign * radiansPerRawRadian * plus - Math.PI / 2),
    angleWrap(sign * radiansPerRawRadian * minus + Math.PI / 2),
  ];
  return {
    zeroRad, sign, radiansPerRawRadian,
    residualRad, maxAbsResidualRad: Math.max(...residualRad.map(Math.abs)),
  };
}

export function fitRadius(samples, driveSign, encoderKind, driveToWheelRatio) {
  if (!["drive_output", "wheel_axis"].includes(encoderKind)) return null;
  const direction = encoderKind === "drive_output" ? driveSign : 1;
  const ratio = encoderKind === "drive_output" ? driveToWheelRatio : 1;
  if ((direction !== 1 && direction !== -1) || !finite(ratio) || ratio <= 0) return null;
  const filled = samples.filter(({ travelM, encoderDeltaRad }) => travelM !== null || encoderDeltaRad !== null);
  const usable = filled.map(({ travelM, encoderDeltaRad }) => ({
    travelM, wheelDeltaRad: direction * encoderDeltaRad / ratio,
    valid: finite(travelM) && finite(encoderDeltaRad),
  }));
  if (usable.length === 0 || usable.some(({ valid, travelM, wheelDeltaRad }) =>
    !valid || !finite(wheelDeltaRad) || Math.abs(wheelDeltaRad) < 1e-6 || travelM * wheelDeltaRad <= 0)) {
    return null;
  }
  const squareSum = usable.reduce((sum, { wheelDeltaRad }) => sum + wheelDeltaRad ** 2, 0);
  const crossSum = usable.reduce((sum, { travelM, wheelDeltaRad }) => sum + travelM * wheelDeltaRad, 0);
  const radiusM = crossSum / squareSum;
  if (!finite(radiusM) || radiusM <= 0) return null;
  const residualM = usable.map(({ travelM, wheelDeltaRad }) => travelM - radiusM * wheelDeltaRad);
  return {
    radiusM, sampleCount: usable.length,
    hasBothDirections: usable.some(({ travelM }) => travelM > 0) && usable.some(({ travelM }) => travelM < 0),
    residualM, maxAbsResidualM: Math.max(...residualM.map(Math.abs)),
  };
}

export function fitFieldFromOdom(odom, field) {
  if (![odom.xM, odom.yM, odom.yawRad, field.xM, field.yM, field.yawRad].every(finite)) return null;
  const yawRad = angleWrap(field.yawRad - odom.yawRad);
  const cos = Math.cos(yawRad);
  const sin = Math.sin(yawRad);
  return {
    xM: field.xM - (cos * odom.xM - sin * odom.yM),
    yM: field.yM - (sin * odom.xM + cos * odom.yM),
    yawRad,
  };
}

export function checkFieldFromOdom(transform, odom, field) {
  if (!transform || ![odom.xM, odom.yM, odom.yawRad, field.xM, field.yM, field.yawRad].every(finite)) return null;
  const cos = Math.cos(transform.yawRad);
  const sin = Math.sin(transform.yawRad);
  const dxM = transform.xM + cos * odom.xM - sin * odom.yM - field.xM;
  const dyM = transform.yM + sin * odom.xM + cos * odom.yM - field.yM;
  return {
    dxM, dyM, positionErrorM: Math.hypot(dxM, dyM),
    yawErrorRad: angleWrap(transform.yawRad + odom.yawRad - field.yawRad),
  };
}

export function registrationFromCalibration(draft) {
  if (draft?.schema !== "longinus.calibration.draft" || draft.schemaVersion !== 1) return null;
  const transform = draft.coordinates?.fieldFromOdom?.transform;
  if (!transform || ![transform.xM, transform.yM, transform.yawRad].every(finite)) return null;
  return { xM: transform.xM, yM: transform.yM, yawRad: transform.yawRad };
}

function imuAxisMatrix(mapping) {
  if (AXES.some((axis) => !/^[+-][XYZ]$/.test(mapping[axis] ?? ""))) return null;
  const rows = AXES.map((axis) => AXES.map((sensor) =>
    mapping[axis][1] === sensor ? (mapping[axis][0] === "+" ? 1 : -1) : 0));
  const [a, b, c] = rows;
  const determinant =
    a[0] * (b[1] * c[2] - b[2] * c[1])
    - a[1] * (b[0] * c[2] - b[2] * c[0])
    + a[2] * (b[0] * c[1] - b[1] * c[0]);
  return determinant === 1 ? rows : null;
}

function numberField(name, label, extra = "") {
  return '<label>' + label + '<input name="' + name + '" type="number" step="any" ' + extra + '></label>';
}

function textField(name, label, extra = "") {
  return '<label>' + label + '<input name="' + name + '" type="text" maxlength="120" ' + extra + '></label>';
}

function signField(name) {
  return '<label>原始驱动正号对应机械前向<select name="' + name + '">'
    + '<option value="">未核验</option><option value="1">同向 (+1)</option><option value="-1">反向 (−1)</option></select></label>';
}

function wheelMarkup({ code, name }, index) {
  const field = (suffix, label, extra) => numberField(code + suffix, label, extra);
  const radiusRun = (number, direction) =>
    '<div class="cal-fields cal-pair">'
    + field("Travel" + number, direction + "实测位移 (m)")
    + field("Encoder" + number, direction + "原始角位移 (rad)")
    + '</div>';
  return '<details class="panel cal-wheel"' + (index === 0 ? ' open' : '') + '>'
    + '<summary><span>' + code + ' · ' + name + '</span><span data-summary="' + code + '">待实测</span></summary>'
    + '<div class="cal-body">'
    + '<h3>电机与轴心</h3><div class="cal-fields">'
    + field("SteerId", "CAN1 DM 节点 ID（十进制）", 'min="1" max="2047"')
    + field("DriveId", "CAN2 DJI 配置 ID", 'min="1" max="8"')
    + signField(code + "DriveSign")
    + field("XMm", "舵轴 X (mm)") + field("YMm", "舵轴 Y (mm)") + '</div>'
    + '<h3>机械零位与方向</h3><div class="cal-fields">'
    + field("Zero", "指向机械前方的反馈 (rad)")
    + field("Plus", "机械 +90° 的反馈 (rad)")
    + field("Minus", "机械 −90° 的反馈 (rad)") + '</div>'
    + '<output class="cal-result" data-result="' + code + '-steer"></output>'
    + '<h3>负载下有效轮径</h3><div class="cal-fields">'
    + '<label>原始反馈角位移类型<select name="' + code + 'EncoderKind">'
    + '<option value="drive_output">减速后电机输出轴 rad</option><option value="wheel_axis">轮轴 rad（向前为正）</option></select></label>'
    + field("Ratio", "实测电机输出轴角 / 轮角", 'min="0"') + '</div>'
    + radiusRun(1, "正向") + radiusRun(2, "反向")
    + '<output class="cal-result" data-result="' + code + '-radius"></output>'
    + '<label class="cal-check"><input name="' + code + 'Verified" type="checkbox">我已用独立实测复核这只轮</label>'
    + '</div></details>';
}

function poseFields(prefix, caption) {
  return '<fieldset class="cal-pose"><legend>' + caption + '</legend><div class="cal-fields">'
    + numberField(prefix + "X", "X (m)") + numberField(prefix + "Y", "Y (m)")
    + numberField(prefix + "Yaw", "航向 (°)") + '</div></fieldset>';
}

function imuFields() {
  const options = AXES.flatMap((axis) => ["+", "-"].map((sign) =>
    '<option value="' + sign + axis + '">' + sign + axis + '</option>')).join("");
  return AXES.map((axis) => '<label>沿车体 +' + axis + ' 测得的 IMU 轴<select name="imu' + axis + '">'
    + '<option value="">未核验</option>' + options + '</select></label>').join("");
}

function laserFields({ code, name }) {
  return '<fieldset class="cal-pose"><legend>' + name + '向束</legend><div class="cal-fields">'
    + numberField(code + "LaserX", "原点 X (mm)")
    + numberField(code + "LaserY", "原点 Y (mm)")
    + numberField(code + "LaserZ", "高度 Z (mm)")
    + numberField(code + "LaserYaw", "水平偏航 (°)")
    + numberField(code + "LaserBias", "距离偏置 (mm)") + '</div></fieldset>';
}

const formMarkup = '<style>'
  + '.cal-form{display:grid;gap:12px}.cal-form .cal-card{display:grid;gap:11px;padding:15px}'
  + '.cal-form .cal-fields{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,145px),1fr));gap:10px}'
  + '.cal-form .cal-fields label{display:grid;gap:5px;min-width:0;font-size:.82rem;color:var(--muted)}'
  + '.cal-form .cal-fields input,.cal-form .cal-fields select{width:100%;min-width:0}'
  + '.cal-form .cal-wheels{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;align-items:start}'
  + '.cal-form .cal-wheel summary{display:flex;align-items:center;justify-content:space-between;gap:10px;cursor:pointer;padding:12px 15px;font-weight:650}'
  + '.cal-form .cal-wheel summary span:first-child::before{content:"▸ ";color:var(--accent)}'
  + '.cal-form .cal-wheel[open] summary span:first-child::before{content:"▾ "}'
  + '.cal-form .cal-wheel summary span:last-child{font-size:.78rem;font-weight:400;color:var(--muted)}'
  + '.cal-form .cal-body{display:grid;gap:10px;border-top:1px solid var(--line);padding:14px 15px}'
  + '.cal-form .cal-check{display:flex;align-items:center;gap:8px;color:var(--muted);font-size:.85rem}'
  + '.cal-form .cal-check input{min-height:0;accent-color:var(--mint)}'
  + '.cal-form .cal-pose{min-width:0;border:1px solid var(--line);border-radius:7px;margin:0;padding:8px 11px 11px}'
  + '.cal-form legend{padding:0 4px;font-size:.84rem;color:var(--text)}'
  + '.cal-form .cal-result{display:block;color:var(--accent);font-size:.82rem;overflow-wrap:anywhere}'
  + '.cal-form .cal-pair{grid-template-columns:repeat(2,minmax(0,1fr))}'
  + '.cal-form ol{color:var(--muted);font-size:.88rem;padding-left:1.3rem;line-height:1.55;margin:0}'
  + '.cal-form .cal-actions{display:flex;gap:10px;flex-wrap:wrap;align-items:center}'
  + '@media(max-width:750px){.cal-form .cal-wheels{grid-template-columns:1fr}}'
  + '</style><form class="cal-form">'
  + '<section class="panel cal-card"><h2>实测档案</h2><div class="cal-fields">'
  + textField("device", "设备编号") + textField("revision", "标定版本") + textField("operator", "测量人")
  + '<label>测量日期<input name="date" type="date"></label>'
  + textField("conditions", "地面、载荷及胎况") + textField("truthSource", "外部真值来源") + '</div>'
  + '<ol><li>断电测量舵轴 X/Y、接地点偏移和整车收拢尺寸。用四个舵轴几何中心定义车体原点，贴出机械前向基准线。</li>'
  + '<li>架空、限制输出，逐轮核对节点 ID、实际转向与驱动方向。对齐 0°、+90°、−90° 并记录同一连续角度域的 DM 反馈；有回绕先展开。</li>'
  + '<li>载荷落地、四轮沿直线，分别正反走已测距离，每轮记录连续编码器角位移。位移沿车体前向取正。用电机输出轴反馈时保留原始正负号；直接测轮轴时向前滚动取正。减速后输出轴已经包含 M2006 内部 36:1。</li>'
  + '<li>用未参与拟合的直行、横移和双向原地旋转复核。记录配对姿态来自同一次试跑、同一时刻、同一车体参考点。</li></ol></section>'
  + '<div class="cal-wheels">' + WHEELS.map(wheelMarkup).join("") + '</div>'
  + '<section class="panel cal-card"><h2>坐标轴与 IMU</h2>'
  + '<p>车体 +X 前、+Y 左、+Z 上，俯视逆时针为正航向。沿车体三个正轴分别做已知方向转动，从原始陀螺读取占主导的有符号 IMU 轴。此步骤验证正交轴映射，精密安装角和动态尺度另用外部真值复核。</p>'
  + '<div class="cal-fields">' + imuFields() + textField("imuNotes", "IMU 安装与转圈实测备注") + '</div>'
  + '<output class="cal-result" data-result="imu"></output>'
  + '<label class="cal-check"><input name="imuVerified" type="checkbox">已用独立顺／逆转圈检查 IMU 航向符号和尺度</label></section>'
  + '<section class="panel cal-card"><h2>场地到里程计登记</h2>'
  + '<p>场地原点为入口侧左角，+X 横跨 3 m 指向场地右侧，+Y 沿 4.6 m 向场内。输入同时测得的一对车体中心位姿，计算里程计坐标到场地坐标的刚体变换。换一次里程计起始原点后需重测；第二对位置或航向应不同，用于独立检查。</p>'
  + '<div class="cal-fields">' + poseFields("fitOdom", "拟合 · 里程计位姿")
  + poseFields("fitField", "拟合 · 场地实测位姿") + '</div>'
  + '<div class="cal-fields">' + poseFields("checkOdom", "复核 · 里程计位姿")
  + poseFields("checkField", "复核 · 场地实测位姿") + '</div>'
  + '<output class="cal-result" data-result="field"></output>'
  + '<label class="cal-check"><input name="fieldVerified" type="checkbox">已用不同位置或航向的独立位姿复核</label></section>'
  + '<details class="panel cal-wheel"><summary><span>四向激光外参记录</span><span>选填</span></summary><div class="cal-body">'
  + '<p>以车体中心为原点，按机械轴测发射点，水平偏航从 +X 朝 +Y 取正。偏置定义为原始距离减实测距离。用多个实测墙距复核，空值表示未测。</p>'
  + '<div class="cal-fields">' + BEAMS.map(laserFields).join("") + '</div></div></details>'
  + '<section class="panel cal-card"><div class="cal-actions"><button class="primary" type="submit">导出标定草案</button>'
  + '<output class="cal-result" data-result="draft" aria-live="polite"></output></div>'
  + '<p>草案保留原始读数、拟合和复核残差，以及仍缺的项目。工作台只计算候选值，由操作者判断是否可用。</p></section></form>';

function readSavedDraft(form) {
  try {
    const saved = JSON.parse(localStorage.getItem(DRAFT_KEY) ?? "{}");
    for (const element of form.elements) {
      if (!element.name || !Object.hasOwn(saved, element.name)) continue;
      const value = saved[element.name];
      if (element.type === "checkbox") element.checked = value === true;
      else if (typeof value === "string") element.value = value;
    }
  } catch {
    // Drafts can still be exported when browser storage is unavailable.
  }
}

function saveDraft(form) {
  try {
    const entries = Array.from(form.elements).filter((element) => element.name)
      .map((element) => [element.name, element.type === "checkbox" ? element.checked : element.value]);
    localStorage.setItem(DRAFT_KEY, JSON.stringify(Object.fromEntries(entries)));
  } catch {
    // Export remains available if storage is unavailable or full.
  }
}

function buildDraft(form) {
  const field = (name) => form.elements.namedItem(name);
  const string = (name) => field(name).value.trim() || null;
  const numeric = (name) => {
    const value = string(name);
    if (value === null) return null;
    const parsed = Number(value);
    return finite(parsed) ? parsed : null;
  };
  const marked = (name) => field(name).checked;
  const pose = (prefix) => ({
    xM: numeric(prefix + "X"),
    yM: numeric(prefix + "Y"),
    yawRad: toRadians(numeric(prefix + "Yaw")),
  });
  const missing = [];
  const issues = [];
  const metadata = {
    device: string("device"), revision: string("revision"), operator: string("operator"),
    date: string("date"), conditions: string("conditions"), truthSource: string("truthSource"),
  };
  for (const [name, value] of Object.entries(metadata)) {
    if (value === null) missing.push("metadata." + name);
  }

  const wheels = WHEELS.map(({ code }) => {
    const steerCanId = numeric(code + "SteerId");
    const driveCanId = numeric(code + "DriveId");
    const driveSign = numeric(code + "DriveSign");
    const steering = {
      zeroRad: numeric(code + "Zero"), positive90Rad: numeric(code + "Plus"),
      negative90Rad: numeric(code + "Minus"),
    };
    const geometry = { xM: toMetres(numeric(code + "XMm")), yM: toMetres(numeric(code + "YMm")) };
    const rolling = {
      encoderKind: string(code + "EncoderKind"), driveToWheelRatio: numeric(code + "Ratio"),
      samples: [1, 2].map((number) => ({
        travelM: numeric(code + "Travel" + number), encoderDeltaRad: numeric(code + "Encoder" + number),
      })),
    };
    const steeringFit = fitSteering(steering);
    const rollingFit = fitRadius(rolling.samples, driveSign, rolling.encoderKind, rolling.driveToWheelRatio);
    const operatorChecked = marked(code + "Verified");
    const require = (value, name) => {
      if (value === null || value === false) missing.push("wheels." + code + "." + name);
    };
    require(steerCanId, "steerCanId");
    require(driveCanId, "driveCanId");
    require(driveSign, "driveSign");
    require(geometry.xM, "steerAxis.xM");
    require(geometry.yM, "steerAxis.yM");
    require(steeringFit, "steering.fit");
    require(rollingFit?.hasBothDirections ?? null, "rolling.bidirectionalFit");
    require(operatorChecked, "operatorChecked");
    if (steerCanId !== null && (!Number.isInteger(steerCanId) || steerCanId < 1 || steerCanId > 2047)) {
      issues.push(code + " 的 CAN1 节点 ID 应为 1–2047 的整数");
    }
    if (driveCanId !== null && (!Number.isInteger(driveCanId) || driveCanId < 1 || driveCanId > 8)) {
      issues.push(code + " 的 CAN2 DJI 配置 ID 应为 1–8 的整数");
    }
    return {
      wheel: code, steerCanId, driveCanId, driveSign, steerAxis: geometry,
      steering: { observations: steering, fit: steeringFit },
      rolling: { ...rolling, fit: rollingFit }, operatorChecked,
    };
  });

  for (const [key, bus] of [["steerCanId", "CAN1"], ["driveCanId", "CAN2"]]) {
    const used = wheels.map((wheel) => wheel[key]).filter(finite);
    if (new Set(used).size !== used.length) issues.push(bus + " 节点 ID 重复");
  }

  const imuMapping = Object.fromEntries(AXES.map((axis) => [axis, string("imu" + axis)]));
  const imuMappingMatrix = imuAxisMatrix(imuMapping);
  const imuChecked = marked("imuVerified");
  if (!imuMappingMatrix) missing.push("coordinates.imu.baseFromImuAxisMatrix");
  if (!imuChecked) missing.push("coordinates.imu.operatorChecked");
  const fitting = { odom: pose("fitOdom"), field: pose("fitField") };
  const checking = { odom: pose("checkOdom"), field: pose("checkField") };
  const transform = fitFieldFromOdom(fitting.odom, fitting.field);
  const sameOdom = transform && Object.values(checking.odom).every(finite)
    && Math.hypot(fitting.odom.xM - checking.odom.xM, fitting.odom.yM - checking.odom.yM) < 1e-6
    && Math.abs(angleWrap(fitting.odom.yawRad - checking.odom.yawRad)) < 1e-6;
  const checkResidual = sameOdom ? null : checkFieldFromOdom(transform, checking.odom, checking.field);
  const fieldChecked = marked("fieldVerified");
  if (!transform) missing.push("coordinates.fieldFromOdom.transform");
  if (!checkResidual) missing.push("coordinates.fieldFromOdom.checkResidual");
  if (!fieldChecked) missing.push("coordinates.fieldFromOdom.operatorChecked");
  if (sameOdom) issues.push("场地复核需使用不同的里程计位姿");

  const optionalMissing = [];
  const laser = Object.fromEntries(BEAMS.map(({ code }) => {
    const beam = {
      xM: toMetres(numeric(code + "LaserX")), yM: toMetres(numeric(code + "LaserY")),
      zM: toMetres(numeric(code + "LaserZ")), yawRad: toRadians(numeric(code + "LaserYaw")),
      rawMinusTruthBiasM: toMetres(numeric(code + "LaserBias")),
    };
    for (const [name, value] of Object.entries(beam)) {
      if (value === null) optionalMissing.push("coordinates.laser." + code + "." + name);
    }
    return [code, beam];
  }));

  return {
    schema: "longinus.calibration.draft", schemaVersion: 1, status: "draft",
    baseFirmware: BASELINE, metadata, wheels,
    coordinates: {
      base: {
        origin: "geometric center of the four steering axes",
        axes: ["forward", "left", "up"], positiveYaw: "ccw", wheelOrder: WHEELS.map(({ code }) => code),
      },
      field: { origin: "left corner on the entrance side", axes: ["right across the 3 m field", "into the 4.6 m field", "up"], positiveYaw: "ccw" },
      imu: { signedAxisCheck: imuMapping, baseFromImuAxisMatrix: imuMappingMatrix, notes: string("imuNotes"), operatorChecked: imuChecked },
      fieldFromOdom: { transform, fitting, checking, checkResidual, operatorChecked: fieldChecked },
      laser,
    },
    missing, optionalMissing, issues,
  };
}

function setResult(form, name, value) {
  form.querySelector('[data-result="' + name + '"]').textContent = value;
}

function updateRatioFields(form) {
  for (const { code } of WHEELS) {
    const kind = form.elements.namedItem(code + "EncoderKind").value;
    form.elements.namedItem(code + "Ratio").disabled = kind === "wheel_axis";
  }
}

function resetCheckForChangedObservation(form, target) {
  const wheel = WHEELS.find(({ code }) => target.name.startsWith(code));
  let check = null;
  if (wheel) check = wheel.code + "Verified";
  else if (["imuX", "imuY", "imuZ"].includes(target.name)) check = "imuVerified";
  else if (/^(fit|check)(Odom|Field)/.test(target.name)) check = "fieldVerified";
  if (check && target.name !== check) form.elements.namedItem(check).checked = false;
}

function showResults(form, draft) {
  for (const wheel of draft.wheels) {
    const { fit: steering } = wheel.steering;
    const { fit: rolling, samples, encoderKind, driveToWheelRatio } = wheel.rolling;
    const steerObservations = Object.values(wheel.steering.observations);
    const hasSteerData = steerObservations.every(finite);
    const hasRollData = samples.some(({ travelM, encoderDeltaRad }) => travelM !== null || encoderDeltaRad !== null);
    setResult(form, wheel.wheel + "-steer", steering
      ? "候选零位 " + steering.zeroRad.toFixed(5) + " rad · 方向 " + (steering.sign === 1 ? "+" : "−")
        + " · 机械角/反馈角 " + steering.radiansPerRawRadian.toFixed(5)
        + " · 最大角残差 " + (steering.maxAbsResidualRad / RADIANS).toFixed(2) + "°"
      : hasSteerData ? "无法拟合：检查 ±90° 的反馈是否在零位两侧，并检查角度回绕。" : "待填三处实测舵角。");
    let radiusText = "待填有符号位移与角位移。";
    if (rolling) {
      radiusText = "候选半径 " + (rolling.radiusM * 1000).toFixed(2) + " mm · "
        + rolling.sampleCount + " 次测量 · 最大位移残差 " + (rolling.maxAbsResidualM * 1000).toFixed(2) + " mm"
        + (rolling.hasBothDirections ? "" : " · 缺反向或正向复核");
    } else if (hasRollData) {
      radiusText = encoderKind === "drive_output" && wheel.driveSign === null ? "请先核验原始驱动方向。"
        : encoderKind === "drive_output" && !(driveToWheelRatio > 0) ? "请填实测外部传动比（直连核验后填 1）。"
          : "无法拟合：检查每次位移与角位移是否成对、非零，按驱动方向转换后符号是否一致。";
    }
    setResult(form, wheel.wheel + "-radius", radiusText);
    form.querySelector('[data-summary="' + wheel.wheel + '"]').textContent = steering && rolling
      ? "已有候选值" : steering || rolling ? "部分记录" : "待实测";
  }

  const { baseFromImuAxisMatrix, signedAxisCheck } = draft.coordinates.imu;
  setResult(form, "imu", baseFromImuAxisMatrix
    ? "得到右手轴映射。精细安装角与动态尺度仍需验证。"
    : Object.values(signedAxisCheck).every(Boolean) ? "轴映射无效：检查重复轴和左右手约定。" : "待核验三个正轴。");
  const { transform, checkResidual } = draft.coordinates.fieldFromOdom;
  const residualText = checkResidual
    ? " · 复核位置差 " + (checkResidual.positionErrorM * 1000).toFixed(1)
      + " mm，航向差 " + (checkResidual.yawErrorRad / RADIANS).toFixed(2) + "°"
    : " · 缺独立复核位姿";
  setResult(form, "field", transform
    ? "候选平移 (" + transform.xM.toFixed(3) + ", " + transform.yM.toFixed(3)
      + ") m · 偏航 " + (transform.yawRad / RADIANS).toFixed(2) + "°" + residualText
    : "待填拟合位姿对。");
  setResult(form, "draft", "待补 " + draft.missing.length + " 项"
    + (draft.issues.length ? " · " + draft.issues.join("，") : ""));
}

export function mountCalibration(element, { download }) {
  element.innerHTML = formMarkup;
  const form = element.querySelector("form");
  readSavedDraft(form);
  function update() {
    updateRatioFields(form);
    const draft = buildDraft(form);
    showResults(form, draft);
    return draft;
  }
  form.addEventListener("input", (event) => {
    resetCheckForChangedObservation(form, event.target);
    update();
    saveDraft(form);
  });
  form.addEventListener("submit", (event) => {
    event.preventDefault();
    const draft = update();
    const revision = draft.metadata.revision?.replace(/[^a-zA-Z0-9_-]+/g, "-").replace(/^-+|-+$/g, "");
    download("longinus-calibration-" + (revision || "draft") + ".json", {
      ...draft, exportedAt: new Date().toISOString(),
    });
    setResult(form, "draft", "已导出草案，待补 " + draft.missing.length + " 项"
      + (draft.issues.length ? " · " + draft.issues.join("，") : ""));
  });
  update();
}

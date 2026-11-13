const SCHEMA = "longinus.run";
const VERSION = 1;
const FIRMWARE = "0059547cff11c99a675f6e17420e2803164e078b";
const SDK = "71b9fbf1bc2ec876fb899cc63cfde0c888515c96";
const LANE_CENTERS = [0.2, 0.9, 1.6, 2.3, 3.0, 3.7, 4.4];
const DEMO_FOOTPRINT_RADIUS = 0.13;

function routePoints() {
    const points = [{x: 0.2, y: -0.5, stage: "启动区"}];
    LANE_CENTERS.forEach((y, index) => {
        const rightbound = index % 2 === 0;
        points.push(
            {x: rightbound ? 0.2 : 2.8, y, lane: index + 1, stage: index ? "换沟" : "入场"},
            {x: rightbound ? 2.8 : 0.2, y, lane: index + 1, stage: "沟内"},
        );
    });
    points.push(
        {x: 0.2, y: 4.4, stage: "返程"},
        {x: 0.2, y: -0.5, stage: "停车"},
    );
    return points;
}

export const FIELD = {
    width: 3,
    length: 4.6,
    laneCenters: LANE_CENTERS,
    ridges: Array.from({length: 6}, (_, index) => {
        const short = index === 3;
        return {x: short ? 0.6 : 0.4, y: 0.4 + 0.7 * index, width: short ? 1.8 : 2.2, height: 0.3};
    }),
    door: {x: 0, y: 0, width: 0.4},
    startup: {x: 0, y: -1, width: 0.4, height: 1},
    route: routePoints(),
};

function motionProfile(distance, limit) {
    const acceleration = 1.05;
    const deceleration = 1.45;
    const peak = Math.min(limit, Math.sqrt(2 * distance / (1 / acceleration + 1 / deceleration)));
    const accelerateFor = peak / acceleration;
    const brakeFor = peak / deceleration;
    const accelerateDistance = peak * accelerateFor / 2;
    const brakeDistance = peak * brakeFor / 2;
    const cruiseFor = Math.max(0, (distance - accelerateDistance - brakeDistance) / peak);
    const brakeAt = accelerateFor + cruiseFor;
    const duration = brakeAt + brakeFor;
    return {
        duration,
        state(t) {
            if (t < accelerateFor) return [acceleration * t * t / 2, acceleration * t];
            if (t < brakeAt) return [accelerateDistance + peak * (t - accelerateFor), peak];
            const left = Math.max(0, duration - t);
            return [distance - deceleration * left * left / 2, deceleration * left];
        },
    };
}

function demoLegs() {
    const legs = [];
    for (const [index, point] of FIELD.route.entries()) {
        if (index === 0) continue;
        const start = FIELD.route[index - 1];
        if (point.stage === "沟内") {
            const station = {x: (start.x + point.x) / 2, y: point.y, lane: point.lane, stage: "沟内"};
            const scored = point.lane % 2 === 1;
            if (scored) legs.push({start, end: station, act: true});
            legs.push({start: scored ? station : start, end: point, cover: true});
        } else if (point.stage === "停车") {
            const nearDoor = {x: 0.2, y: 0.34, stage: "返程"};
            legs.push({start, end: nearDoor});
            legs.push({start: nearDoor, end: point});
        } else {
            legs.push({start, end: point});
        }
    }
    return legs;
}

function rectangleDistance(x, y, rectangle) {
    const dx = Math.max(rectangle.x - x, x - rectangle.x - rectangle.width, 0);
    const dy = Math.max(rectangle.y - y, y - rectangle.y - rectangle.height, 0);
    return Math.hypot(dx, dy);
}

const DEMO_OBSTACLES = [
    ...FIELD.ridges,
    {x: 0, y: 0, width: 0, height: FIELD.length},
    {x: FIELD.width, y: 0, width: 0, height: FIELD.length},
    {x: 0, y: FIELD.length, width: FIELD.width, height: 0},
    {x: FIELD.door.width, y: 0, width: FIELD.width - FIELD.door.width, height: 0},
];

function simulatedClearance(x, y) {
    const nearest = Math.min(...DEMO_OBSTACLES.map((rect) => rectangleDistance(x, y, rect)));
    return Math.max(0, nearest - DEMO_FOOTPRINT_RADIUS);
}

function timeAtDistance(motion, distance) {
    let lower = 0;
    let upper = motion.duration;
    for (let iteration = 0; iteration < 20; iteration++) {
        const middle = (lower + upper) / 2;
        if (motion.state(middle)[0] < distance) lower = middle;
        else upper = middle;
    }
    return upper;
}

const DEMO_PROFILES = [
    {id: "sim-a", name: "基准 A", straightSpeed: 0.86, linkSpeed: 0.46, settle: 0.34, act: 1.7, lateral: 0.009, yaw: 0.017},
    {id: "sim-b", name: "竞速 B", straightSpeed: 1.12, linkSpeed: 0.53, settle: 0.24, act: 1.4, lateral: 0.017, yaw: 0.029},
];

function makeDemoRun(profile) {
    const samples = [];
    const events = [{t: 0, kind: "start", label: "模拟起跑"}];
    let elapsed = 0;
    let lastLane = null;
    const heading = Math.PI / 2;
    const legs = demoLegs();
    let remaining = legs.reduce((distance, leg) => distance + Math.hypot(leg.end.x - leg.start.x, leg.end.y - leg.start.y), 0);

    function addSample(t, xPlan, yPlan, dx, dy, vCmd, distanceLeft, lane, stage) {
        const sampleTime = Math.round(t * 1000) / 1000;
        if (sampleTime <= (samples.at(-1)?.t ?? -1)) return;
        const progress = Math.min(1, vCmd / 0.3);
        const lateralOffset = profile.lateral * progress * Math.sin(1.15 * t);
        const xTruth = xPlan - dy * lateralOffset;
        const yTruth = yPlan + dx * lateralOffset;
        const yaw = heading + profile.yaw * progress * Math.sin(2.2 * t);
        const correctedSpeed = vCmd * (0.985 + 0.026 * Math.sin(2.6 * t));
        const yawRate = 2.2 * profile.yaw * progress * Math.cos(2.2 * t);
        samples.push({
            t: sampleTime, epoch: 0, xPlan, yPlan, xTruth, yTruth,
            x: xTruth + 0.002 * progress * Math.sin(3 * t),
            y: yTruth + 0.002 * progress * Math.cos(2.4 * t),
            yaw, yawRef: heading,
            vxCmd: vCmd * dy, vyCmd: -vCmd * dx,
            vxFk: correctedSpeed * (dx * Math.cos(yaw) + dy * Math.sin(yaw)),
            vyFk: correctedSpeed * (-dx * Math.sin(yaw) + dy * Math.cos(yaw)),
            wzFk: yawRate * 1.04,
            gz: yawRate,
            vx: correctedSpeed * (dx * Math.cos(yaw) + dy * Math.sin(yaw)) * 0.995,
            vy: correctedSpeed * (-dx * Math.sin(yaw) + dy * Math.cos(yaw)) * 0.995,
            wz: yawRate * 0.985,
            remaining: Math.max(0, distanceLeft),
            clearance: simulatedClearance(xTruth, yTruth),
            lane, stage, active: stage !== "已停车",
        });
    }

    function wait(point, duration, stage) {
        const count = Math.ceil(duration * 12);
        for (let step = 1; step <= count; step++) {
            addSample(elapsed + duration * step / count, point.x, point.y, 0, 0, 0, remaining, point.lane, stage);
        }
        elapsed += duration;
    }

    for (const leg of legs) {
        const {start, end} = leg;
        const distance = Math.hypot(end.x - start.x, end.y - start.y);
        const dx = (end.x - start.x) / distance;
        const dy = (end.y - start.y) / distance;
        const lane = end.stage === "沟内" ? end.lane : undefined;
        if (lane && lane !== lastLane) {
            lastLane = lane;
            events.push({t: elapsed, kind: "lane_enter", lane, label: `进入第 ${lane} 沟`});
        }
        const speed = end.stage === "停车" ? 0.36
            : end.stage === "入场" || end.stage === "换沟" ? profile.linkSpeed
            : profile.straightSpeed;
        const motion = motionProfile(distance, speed);
        const count = Math.ceil(motion.duration * 12);
        for (let step = 0; step <= count; step++) {
            const t = motion.duration * step / count;
            const [progress, speedCmd] = motion.state(t);
            addSample(elapsed + t, start.x + dx * progress, start.y + dy * progress,
                dx, dy, speedCmd, remaining - progress, lane, end.stage);
        }
        if (end.stage === "入场") {
            const crossing = timeAtDistance(motion, -start.y - DEMO_FOOTPRINT_RADIUS);
            events.push({t: elapsed + crossing, kind: "timing_start", label: "模拟车头跨过起跑线"});
        } else if (end.stage === "停车") {
            const crossing = timeAtDistance(motion, start.y + DEMO_FOOTPRINT_RADIUS);
            events.push({t: elapsed + crossing, kind: "field_exit", label: "模拟整车出门"});
        }
        elapsed += motion.duration;
        remaining -= distance;
        if (leg.cover) events.push({t: elapsed, kind: "lane_complete", lane, label: `第 ${lane} 沟走完`});
        if (leg.act) {
            events.push({t: elapsed, kind: "work_stop", lane, label: `第 ${lane} 沟作业停靠`});
            wait(end, profile.act, "作业停靠");
            events.push({t: elapsed, kind: "action_ok", lane, label: "模拟摘除并收拢"});
        } else if (end.stage !== "停车") {
            wait(end, profile.settle, "对舵等待");
        }
    }

    wait(FIELD.route.at(-1), 0.12, "已停车");
    events.push({t: elapsed, kind: "parked", label: "模拟整车停入启动区"});
    return {
        id: profile.id,
        name: profile.name,
        source: "simulated",
        createdAt: "2026-11-13T00:00:00.000Z",
        manifest: {
            runId: profile.id, bootId: "simulated",
            baselineFirmware: FIRMWARE, standardCommit: SDK,
            mapRevision: "attachment-6-nominal", calibrationRevision: null,
            parameterRevision: profile.id === "sim-a" ? "sim-baseline" : "sim-candidate",
            surface: "模拟场地", payload: "模拟载荷",
            status: "complete", penaltySeconds: 0, completed: true,
            simulationFootprintRadiusM: DEMO_FOOTPRINT_RADIUS,
        },
        samples, events, warnings: [], raw: [], frame: "field",
    };
}

export function makeDemoRuns() {
    return DEMO_PROFILES.map(makeDemoRun);
}

function finite(value) {
    return typeof value === "number" && Number.isFinite(value);
}

export function summarizeRun(run) {
    const samples = run.samples;
    const first = samples[0];
    const last = samples.at(-1);
    const hasFusedSpeed = samples.some((sample) => sample.firmwareStatus !== "mismatch" && finite(sample.vx) && finite(sample.vy));
    let distance = 0;
    let positionSteps = 0;
    let peakSpeed = null;
    let squaredSpeedError = 0;
    let pairedSpeeds = 0;
    let minClearance = null;
    const headingErrors = [];

    for (const [index, sample] of samples.entries()) {
        if (sample.firmwareStatus === "mismatch") continue;
        const previous = samples[index - 1];
        if (previous && !sample.breakBefore && previous.epoch === sample.epoch &&
            previous.firmwareStatus !== "mismatch" &&
            sample.poseValid !== false && previous.poseValid !== false &&
            [sample.x, sample.y, previous.x, previous.y].every(finite)) {
            positionSteps++;
            distance += Math.hypot(sample.x - previous.x, sample.y - previous.y);
        }
        const vx = hasFusedSpeed ? sample.vx : sample.vxFk;
        const vy = hasFusedSpeed ? sample.vy : sample.vyFk;
        if (finite(vx) && finite(vy)) {
            const speed = Math.hypot(vx, vy);
            peakSpeed = Math.max(peakSpeed ?? 0, speed);
            if (finite(sample.vxCmd) && finite(sample.vyCmd)) {
                squaredSpeedError += (speed - Math.hypot(sample.vxCmd, sample.vyCmd)) ** 2;
                pairedSpeeds++;
            }
        }
        if (finite(sample.yaw) && finite(sample.yawRef) && sample.poseValid !== false) {
            const difference = sample.yaw - sample.yawRef;
            headingErrors.push(Math.abs(Math.atan2(Math.sin(difference), Math.cos(difference))));
        }
        if (finite(sample.clearance)) minClearance = Math.min(minClearance ?? Infinity, sample.clearance);
    }

    headingErrors.sort((a, b) => a - b);
    const completedLaneEvents = run.events.filter((event) => event.kind === "lane_complete");
    const completedLanes = new Set(completedLaneEvents.map((event) => event.lane)
        .filter((lane) => Number.isInteger(lane) && lane >= 1 && lane <= FIELD.laneCenters.length));
    const exit = run.events.find((event) => event.kind === "field_exit" && finite(event.t));
    const start = run.events.find((event) => event.kind === "timing_start" && finite(event.t));
    const completed = typeof run.manifest.completed === "boolean" ? run.manifest.completed
        : run.events.some((event) => event.kind === "race_complete") ? true : null;

    return {
        duration: samples.length > 1 ? last.t - first.t : null,
        raceTime: start && exit && exit.t >= start.t ? exit.t - start.t : null,
        distance: positionSteps ? distance : null,
        peakSpeed,
        speedSource: peakSpeed === null ? null : hasFusedSpeed ? "fusion" : "fk",
        rmsSpeedError: pairedSpeeds ? Math.sqrt(squaredSpeedError / pairedSpeeds) : null,
        p95HeadingError: headingErrors.length ? headingErrors[Math.ceil(headingErrors.length * 0.95) - 1] : null,
        lanesCovered: completedLanes.size || null,
        penaltySeconds: finite(run.manifest.penaltySeconds) ? run.manifest.penaltySeconds : null,
        minClearance,
        completed,
    };
}

function objectLike(value) {
    return value !== null && typeof value === "object" && !Array.isArray(value);
}

function cloneJson(value, depth = 0) {
    if (depth > 24) throw new Error("归档字段嵌套过深。");
    if (value === null || typeof value === "string" || typeof value === "boolean" || finite(value)) return value;
    if (Array.isArray(value)) return value.map((entry) => cloneJson(entry ?? null, depth + 1));
    if (!objectLike(value)) throw new Error("归档字段含不支持的值。");
    const entries = [];
    for (const [key, entry] of Object.entries(value)) {
        if (key === "__proto__" || key === "constructor" || key === "prototype" || entry === undefined) continue;
        entries.push([key, cloneJson(entry, depth + 1)]);
    }
    return Object.fromEntries(entries);
}

function readSample(sample, index, previousTime) {
    if (!objectLike(sample) || !finite(sample.t) || sample.t < 0 || sample.t < previousTime) {
        throw new Error(`归档第 ${index + 1} 个样本的时间无效或顺序错误。`);
    }
    const numeric = ["x", "y", "yaw", "yawRef", "vxCmd", "vyCmd", "vxFk", "vyFk", "wzFk", "gz", "vx", "vy", "wz",
        "remaining", "xTruth", "yTruth", "xPlan", "yPlan", "clearance", "sourceTimeMs"];
    for (const field of numeric) {
        if (sample[field] != null && !finite(sample[field])) throw new Error(`归档样本字段 ${field} 需要有限数值。`);
    }
    if (sample.epoch != null && (!Number.isInteger(sample.epoch) || sample.epoch < 0)) throw new Error("归档样本时钟段无效。");
    for (const field of ["active", "breakBefore", "poseValid"]) {
        if (sample[field] != null && typeof sample[field] !== "boolean") throw new Error(`归档样本字段 ${field} 需要布尔值。`);
    }
    return cloneJson(sample);
}

export function makeArchive(run) {
    return {schema: SCHEMA, version: VERSION, run: normalizeRun(run)};
}

export function parseArchive(object) {
    if (!objectLike(object) || object.schema !== SCHEMA) throw new Error("文件不是 Longinus 试跑归档。");
    if (object.version !== VERSION) throw new Error(`不支持的试跑归档版本 ${String(object.version)}。`);
    return normalizeRun(object.run);
}

function normalizeRun(run) {
    if (!objectLike(run) || typeof run.id !== "string" || !run.id.trim() ||
        typeof run.name !== "string" || !run.name.trim() ||
        !["imported", "simulated"].includes(run.source) ||
        !["field", "odom", "unknown"].includes(run.frame) ||
        typeof run.createdAt !== "string" || !Number.isFinite(Date.parse(run.createdAt)) ||
        !objectLike(run.manifest) || !Array.isArray(run.samples) || !Array.isArray(run.events) ||
        !Array.isArray(run.raw) || !Array.isArray(run.warnings)) {
        throw new Error("试跑归档缺少有效的标识、来源、坐标、时间或数据列表。");
    }
    let previousTime = -1;
    const samples = run.samples.map((sample, index) => {
        const normalized = readSample(sample, index, previousTime);
        previousTime = normalized.t;
        return normalized;
    });
    const events = run.events.map((event) => {
        if (!objectLike(event) || !(event.t === null || (finite(event.t) && event.t >= 0)) ||
            typeof event.kind !== "string" || typeof event.label !== "string") {
            throw new Error("试跑归档的事件格式无效。");
        }
        return cloneJson(event);
    });
    const raw = run.raw.map((record) => {
        if (!objectLike(record) || typeof record.filename !== "string" ||
            typeof record.format !== "string" || typeof record.text !== "string") {
            throw new Error("试跑归档的原始文件条目无效。");
        }
        return {filename: record.filename, format: record.format, text: record.text};
    });
    if (run.warnings.some((warning) => typeof warning !== "string")) throw new Error("试跑归档的提示格式无效。");

    const normalized = {
        id: run.id, name: run.name, source: run.source, frame: run.frame, createdAt: run.createdAt,
        manifest: cloneJson(run.manifest), samples, events, raw, warnings: [...run.warnings],
    };
    if (run.registration != null) {
        const {tx, ty, yawRad} = run.registration;
        if (![tx, ty, yawRad].every(finite)) throw new Error("场地登记需要有限的 tx、ty 和 yawRad。");
        normalized.registration = {tx, ty, yawRad};
    }
    if (run.availability != null) normalized.availability = cloneJson(run.availability);
    if (run.untimedSamples != null) {
        if (!Array.isArray(run.untimedSamples) || !run.untimedSamples.every(objectLike)) {
            throw new Error("归档的无时戳样本格式无效。");
        }
        normalized.untimedSamples = cloneJson(run.untimedSamples);
    }
    if (run.timeSources != null) {
        if (!objectLike(run.timeSources) || !Object.values(run.timeSources).every((count) => Number.isInteger(count) && count >= 0)) {
            throw new Error("归档的时间来源格式无效。");
        }
        normalized.timeSources = cloneJson(run.timeSources);
    }
    return normalized;
}

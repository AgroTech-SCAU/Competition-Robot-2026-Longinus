import {FIELD} from "./runs.js";

const colors = {muted: "#a3b3ac", grid: "#32433c", ridge: "#30433b", a: "#62d9ab", b: "#f0b879", cursor: "#edf2f1"};
const degrees = 180 / Math.PI;
export const finite = Number.isFinite;
export const hasPose = (s) => Boolean(s && s.poseValid !== false && finite(s.x) && finite(s.y));
export const speed = (s, command = false, source) => {
    if (!s) return undefined;
    const pair = command ? [s.vxCmd, s.vyCmd]
        : source === "fusion" || (!source && [s.vx, s.vy].every(finite)) ? [s.vx, s.vy] : [s.vxFk, s.vyFk];
    return pair.every(finite) ? Math.hypot(...pair) : undefined;
};

export function heading(sample, run, reference = false) {
    if (!sample || sample.poseValid === false) return undefined;
    const yaw = reference ? sample.yawRef : sample.yaw;
    if (!finite(yaw)) return undefined;
    const angle = yaw + (run.frame === "field" ? 0 : run.registration?.yawRad ?? 0);
    return Math.atan2(Math.sin(angle), Math.cos(angle)) * degrees;
}

export function nearest(samples, t) {
    let left = 0;
    let right = samples.length;
    while (left < right) {
        const mid = (left + right) >>> 1;
        if (samples[mid].t < t) left = mid + 1;
        else right = mid;
    }
    const before = samples[left - 1];
    const after = samples[left];
    if (!before || !after) {
        const edge = before ?? after;
        return edge && Math.abs(edge.t - t) < 0.03 ? edge : undefined;
    }
    if (after.t === t) return after;
    if (after.breakBefore || before.epoch !== after.epoch) return undefined;
    return t - before.t <= after.t - t ? before : after;
}

function surface(canvas) {
    const {width, height} = canvas.getBoundingClientRect();
    if (!width || !height) return undefined;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const pixelsX = Math.round(width * dpr);
    const pixelsY = Math.round(height * dpr);
    if (canvas.width !== pixelsX || canvas.height !== pixelsY) {
        canvas.width = pixelsX;
        canvas.height = pixelsY;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.font = '12px "Microsoft YaHei UI", sans-serif';
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    return {ctx, width, height};
}

function trace(ctx, points, projection, color, dash = [], width = 1.7) {
    ctx.beginPath();
    let started = false;
    for (const point of points) {
        if (!point) {
            started = false;
            continue;
        }
        const [x, y] = projection(point);
        if (!started) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
        started = true;
    }
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash);
    ctx.stroke();
    ctx.setLineDash([]);
}

function pathPoints(samples, transform, circular = false) {
    const step = Math.max(1, Math.ceil(samples.length / 2400));
    const points = [];
    let previous;
    let final;
    for (const [index, sample] of samples.entries()) {
        const point = transform(sample);
        const boundary = sample.breakBefore || (previous && sample.epoch !== previous.sample.epoch) ||
            (circular && previous && point && Math.abs(point[1] - previous.point[1]) > 180);
        if ((!point || boundary) && previous) {
            if (final !== previous.point) points.push(previous.point);
            points.push(null);
            final = null;
        }
        if (point && (!previous || boundary || index % step === 0 || index === samples.length - 1)) {
            points.push(point);
            final = point;
        }
        previous = point ? {sample, point} : undefined;
    }
    return points;
}

export function preparePlots(a, b, sourceA, sourceB, comparisonShift = 0) {
    const field = a.frame === "field" || Boolean(a.registration);
    const compareOnMap = Boolean(b && field && (b.frame === "field" || b.registration));
    const position = (sample, run) => {
        if (!hasPose(sample)) return undefined;
        const r = run.registration;
        if (!r || run.frame === "field") return [sample.x, sample.y];
        const c = Math.cos(r.yawRad);
        const s = Math.sin(r.yawRad);
        return [c * sample.x - s * sample.y + r.tx, s * sample.x + c * sample.y + r.ty];
    };
    const pathA = pathPoints(a.samples, (sample) => position(sample, a));
    const pathB = compareOnMap ? pathPoints(b.samples, (sample) => position(sample, b)) : [];
    const series = (mode) => {
        const seriesFor = (run, color, command = false, dashed = false) => {
            const value = (sample) => mode === "speed"
                ? speed(sample, command, run === a ? sourceA : sourceB) : heading(sample, run, command);
            return {color, dash: dashed ? [7, 4] : command ? [2, 4] : [], value, run,
                points: pathPoints(run.samples, (sample) => {
                    const y = value(sample);
                    return finite(y) ? [sample.t + (run === b ? comparisonShift : 0), y] : undefined;
                }, mode === "yaw")};
        };
        const result = [seriesFor(a, colors.a), seriesFor(a, colors.a, true)];
        if (b) result.push(seriesFor(b, colors.b, false, true));
        return result;
    };
    return {a, b, field, compareOnMap, position, pathA, pathB, speed: series("speed"), yaw: series("yaw")};
}

function message(ctx, width, height, text) {
    ctx.textAlign = "center";
    ctx.fillStyle = colors.muted;
    ctx.fillText(text, width / 2, height / 2);
}

function dot(ctx, x, y, color) {
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, Math.PI * 2);
    ctx.fillStyle = "#17201e";
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.stroke();
}

export function drawMap(canvas, view, currentA, currentB) {
    const size = surface(canvas);
    if (!size) return;
    const {ctx, width, height} = size;
    const {a, b, field, compareOnMap, position, pathA, pathB} = view;
    let xMin = -0.08, xMax = FIELD.width + 0.08;
    let yMin = FIELD.startup.y - 0.04, yMax = FIELD.length + 0.04;
    if (!field) {
        const points = pathA.filter(Boolean);
        if (!points.length) {
            message(ctx, width, height, "本趟日志无可用 XY");
            return;
        }
        let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
        for (const [x, y] of points) {
            xmin = Math.min(x, xmin); xmax = Math.max(x, xmax);
            ymin = Math.min(y, ymin); ymax = Math.max(y, ymax);
        }
        const pad = Math.max(0.18, (xmax - xmin) * 0.08, (ymax - ymin) * 0.08);
        xMin = xmin - pad; xMax = xmax + pad; yMin = ymin - pad; yMax = ymax + pad;
    }
    const scale = Math.min((width - 52) / (xMax - xMin), (height - 27) / (yMax - yMin));
    const left = (width - scale * (xMax - xMin)) / 2;
    const top = (height - scale * (yMax - yMin)) / 2;
    const project = ([x, y]) => [left + (x - xMin) * scale, top + (yMax - y) * scale];
    const rect = ({x, y, width: w, height: h}, fill, stroke) => {
        const [sx, sy] = project([x, y + h]);
        ctx.fillStyle = fill;
        ctx.strokeStyle = stroke;
        ctx.lineWidth = 1;
        ctx.fillRect(sx, sy, scale * w, scale * h);
        ctx.strokeRect(sx, sy, scale * w, scale * h);
    };
    if (field) {
        rect({x: 0, y: 0, width: FIELD.width, height: FIELD.length}, "#141e1a", "#60736a");
        rect(FIELD.startup, "#1e382e", "#5d8170");
        ctx.strokeStyle = "#1e382e";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(...project([0, 0]));
        ctx.lineTo(...project([FIELD.door.width, 0]));
        ctx.stroke();
        FIELD.ridges.forEach((ridge) => rect(ridge, colors.ridge, "#53685e"));
        ctx.fillStyle = colors.muted;
        ctx.textAlign = "left";
        FIELD.laneCenters.forEach((y, i) => {
            const [sx, sy] = project([FIELD.width, y]);
            ctx.fillText(String(i + 1), sx + 7, sy + 4);
        });
        const [sx, sy] = project([FIELD.startup.width, FIELD.startup.y + 0.28]);
        ctx.fillText("启动区", sx + 7, sy);
        trace(ctx, FIELD.route.map(({x, y}) => [x, y]), project, "#a4b3ab", [2, 5], 1.2);
    } else {
        rect({x: xMin, y: yMin, width: xMax - xMin, height: yMax - yMin}, "#141e1a", colors.grid);
        const hasXOrigin = xMin <= 0 && xMax >= 0;
        const hasYOrigin = yMin <= 0 && yMax >= 0;
        if (hasYOrigin) trace(ctx, [[xMin, 0], [xMax, 0]], project, colors.grid, [2, 4], 1);
        if (hasXOrigin) trace(ctx, [[0, yMin], [0, yMax]], project, colors.grid, [2, 4], 1);
        ctx.fillStyle = colors.muted;
        ctx.textAlign = "left";
        ctx.fillText("odom · m", left + 6, top + 16);
        if (hasXOrigin && hasYOrigin) {
            const [originX, originY] = project([0, 0]);
            ctx.fillText("0", originX + 4, originY - 4);
        }
    }
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, width, height);
    ctx.clip();
    trace(ctx, pathA, project, colors.a);
    if (compareOnMap) trace(ctx, pathB, project, colors.b, [5, 4]);
    function marker(sample, run, color) {
        const point = position(sample, run);
        if (!point) return;
        const [x, y] = project(point);
        dot(ctx, x, y, color);
        const angle = heading(sample, run);
        if (finite(angle)) {
            const yaw = angle / degrees;
            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(x + 12 * Math.cos(yaw), y - 12 * Math.sin(yaw));
            ctx.stroke();
        }
    }
    if (compareOnMap) marker(currentB, b, colors.b);
    marker(currentA, a, colors.a);
    ctx.restore();
}

export const plotGutters = {left: 43, right: 13};

export function drawChart(canvas, series, cursor, duration, mode, currentA, currentB) {
    const size = surface(canvas);
    if (!size) return;
    const {ctx, width, height} = size;
    const {left, right} = plotGutters;
    const top = 9;
    const bottom = 24;
    const plotWidth = width - left - right;
    const plotHeight = height - top - bottom;
    let min = mode === "speed" ? 0 : Infinity;
    let max = mode === "speed" ? 0.15 : -Infinity;
    let count = 0;
    for (const line of series) {
        for (const point of line.points) {
            if (!point) continue;
            min = Math.min(min, point[1]);
            max = Math.max(max, point[1]);
            count++;
        }
    }
    if (!count) {
        message(ctx, width, height, mode === "speed" ? "未记录可用平移速度" : "未记录可用航向");
        return;
    }
    if (mode === "speed") max = Math.ceil(max * 5) / 5;
    else {
        const padding = Math.max(1, (max - min) * 0.12);
        min -= padding;
        max += padding;
    }
    const timeMax = Math.max(duration, 0.01);
    const project = ([x, y]) => [left + x / timeMax * plotWidth, top + (max - y) / (max - min) * plotHeight];
    ctx.fillStyle = colors.muted;
    ctx.strokeStyle = colors.grid;
    ctx.lineWidth = 1;
    ctx.textAlign = "right";
    for (let i = 0; i <= 3; i++) {
        const y = top + i * plotHeight / 3;
        ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(width - right, y); ctx.stroke();
        const value = max - i * (max - min) / 3;
        ctx.fillText(value.toFixed(mode === "speed" ? 2 : 1), left - 6, y + 4);
    }
    const ticks = width < 390 ? 3 : 5;
    for (let i = 0; i <= ticks; i++) {
        const x = left + i * plotWidth / ticks;
        ctx.textAlign = i === ticks ? "right" : "center";
        ctx.fillText((i * duration / ticks).toFixed(duration < 10 ? 1 : 0) + (i === ticks ? " s" : ""), x, height - 5);
    }
    ctx.save();
    ctx.beginPath();
    ctx.rect(left, top - 4, plotWidth, plotHeight + 8);
    ctx.clip();
    for (const line of series) trace(ctx, line.points, project, line.color, line.dash);
    const [cursorX] = project([cursor, min]);
    trace(ctx, [[cursorX, top], [cursorX, top + plotHeight]], (p) => p, colors.cursor, [3, 4], 1);
    const current = [currentA, currentA, currentB];
    for (const [index, line] of series.entries()) {
        const value = line.value(current[index]);
        if (finite(value) && value >= min && value <= max) dot(ctx, ...project([cursor, value]), line.color);
    }
    ctx.restore();
}

import {parseLog} from "./importers.js";
import {makeDemoRuns, makeArchive, parseArchive, summarizeRun} from "./runs.js";
import {mountCalibration, registrationFromCalibration} from "./calibration.js";
import {drawChart, drawMap, finite, hasPose, heading, nearest, plotGutters, preparePlots, speed} from "./plots.js";

const byId = (id) => document.getElementById(id);
const make = (tag, text) => {
    const element = document.createElement(tag);
    if (text !== undefined) element.textContent = text;
    return element;
};
const runs = new Map(makeDemoRuns().map((run) => [run.id, run]));
const runA = byId("run-a");
const runB = byId("run-b");
const alignment = byId("compare-align");
const timeline = byId("timeline");
const play = byId("play");
const metadata = byId("metadata-form");
const registration = byId("registration-form");
const degrees = 180 / Math.PI;
const base = "0059547cff11c99a675f6e17420e2803164e078b";
const sdk = "71b9fbf1bc2ec876fb899cc63cfde0c888515c96";

let a;
let b;
let view;
let summaryA;
let summaryB;
let currentTime = 0;
let duration = 0;
let comparisonShift = 0;
let animation = null;
let lastFrame;
let lastPaint = 0;
let selectedView = "replay";

function notify(message, error = false) {
    const element = byId("notice");
    element.textContent = message;
    element.hidden = !message;
    element.classList.toggle("error", error);
}

function fileName(name) {
    return name.replace(/[<>:"/\\|?*\x00-\x1f]/g, "_").slice(0, 100) || "longinus";
}

function download(name, payload) {
    const blob = new Blob([JSON.stringify(payload, null, 2) + "\n"], {type: "application/json"});
    const url = URL.createObjectURL(blob);
    const link = make("a");
    link.href = url;
    link.download = fileName(name);
    document.body.append(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function source(run) {
    return run.source === "simulated" ? "模拟" : "导入";
}

function updateRunOptions(aId, bId) {
    for (const select of [runA, runB]) {
        select.replaceChildren();
        if (select === runB) select.append(new Option("关闭对比", ""));
        for (const run of runs.values()) {
            select.append(new Option(run.name + " · " + source(run), run.id));
        }
    }
    runA.value = aId;
    runB.value = bId ?? "";
    if (runA.value === runB.value) runB.value = "";
    refreshRun();
}

function format(value, digits = 2, suffix = "") {
    return finite(value) ? value.toFixed(digits) + suffix : "—";
}

function status(value) {
    return value === true ? "完成" : value === false ? "未完成" : "—";
}

function refreshMetrics() {
    const metrics = [
        ["记录时长 (s)", "duration", 2],
        ["起跑线至全车离场 (s)", "raceTime", 2],
        ["估计里程 (m)", "distance", 2],
        ["峰值平移速度 (m/s)", "peakSpeed", 2],
        ["速度幅值误差 RMS (m/s)", "rmsSpeedError", 3],
        ["航向误差 P95 (°)", "p95HeadingError", 2, degrees],
        ["最低净空 (mm)", "minClearance", 0, 1000],
        ["已确认沟数 / 7", "lanesCovered", 0],
        ["罚时 (s)", "penaltySeconds", 0],
        ["离场后停车", "completed", null],
    ];
    byId("metrics").replaceChildren(...metrics.map(([label, key, digits, scale = 1]) => {
        const row = make("tr");
        const header = make("th", label);
        header.scope = "row";
        const va = summaryA[key];
        const vb = summaryB[key];
        const formatMetric = (value) => digits === null ? status(value) : finite(value) ? format(value * scale, digits) : "—";
        const mixedSpeed = (key === "peakSpeed" || key === "rmsSpeedError") && summaryA.speedSource !== summaryB.speedSource;
        const comparable = a.source === b?.source && !mixedSpeed;
        const delta = comparable && finite(va) && finite(vb) ? format((vb - va) * scale, digits) : "—";
        row.append(header, make("td", formatMetric(va)), make("td", formatMetric(vb)),
            make("td", delta !== "—" && vb > va ? "+" + delta : delta));
        return row;
    }));
}

function refreshEvents() {
    const timed = a.events.filter((event) => finite(event.t)).length;
    const none = a.events.length - timed;
    byId("events-note").textContent = a.events.length
        ? timed + " 条可定位 · " + none + " 条无独立时间戳" : "本趟未记录事件。";
    const events = byId("events");
    events.replaceChildren(...a.events.slice(0, 400).map((event) => {
        const item = make("li");
        if (finite(event.t)) {
            const button = make("button", format(event.t, 2, " s"));
            button.type = "button";
            button.addEventListener("click", () => {
                stopPlaying();
                seek(event.t);
            });
            item.append(button);
        } else item.append(make("span", "未定时"));
        item.append(make("span", event.label));
        return item;
    }));
    if (a.events.length > 400) events.append(make("li", "其余事件可在完整归档查看。"));
}

function refreshHealth() {
    const any = (fn) => [...a.samples, ...(a.untimedSamples ?? [])].some(fn);
    const flags = [
        ["里程计 XY", any(hasPose)],
        ["航向", any((s) => finite(s.yaw))],
        ["航向参考", any((s) => finite(s.yawRef))],
        ["速度指令 vx/vy", any((s) => finite(speed(s, true)))],
        ["融合速度 vx/vy", any((s) => [s.vx, s.vy].every(finite))],
        ["轮速 FK vx/vy", any((s) => [s.vxFk, s.vyFk].every(finite))],
        ["原始轮速 FK vx/vy", any((s) => [s.vxFkRaw, s.vyFkRaw].every(finite)) || a.availability?.vxFkRaw && a.availability?.vyFkRaw],
        ["陀螺仪 Z", any((s) => finite(s.gz))],
        ["四轮独立舵角与轮速", any((s) => Array.isArray(s.wheels) && s.wheels.length === 4)],
        ["四向激光距离", a.availability?.laserRange || a.events.some((e) => e.laser)],
        ["逐束采样时间与墙段", any((s) => Array.isArray(s.laserBeams) && s.laserBeams.length === 4)],
        ["外部位置真值", any((s) => [s.xTruth, s.yTruth].every(finite))],
        ["沟覆盖、全车离场、停车", ["lane_complete", "field_exit", "parked"].every((kind) => a.events.some((event) => event.kind === kind))],
        ["循环耗时、漏拍与丢帧", a.events.some((e) => e.kind === "scheduler") || any((s) => finite(s.controlDt))],
    ];
    byId("field-status").replaceChildren(...flags.map(([name, present]) => {
        const item = make("li");
        item.append(make("strong", name), document.createTextNode(present ? " · 已记录" : " · 缺失"));
        return item;
    }));
    const warnings = [...a.warnings];
    if (b && a.source !== b.source) warnings.unshift("A/B 混有模拟和导入数据，差值列停用。");
    if (b) {
        const sa = summaryA.speedSource;
        const sb = summaryB.speedSource;
        if (sa && sb && sa !== sb) warnings.unshift("A/B 的速度来源不同，相关差值停用。");
    }
    if (a.frame !== "field" && !a.registration && a.samples.some(hasPose)) {
        warnings.push("里程计坐标未登记到场地，仅显示局部轨迹。");
    }
    if (multiplePoseEpochs(a)) warnings.push("包含多个里程计时钟段，请按单次开机拆分日志后登记场地。");
    byId("warnings").replaceChildren(...warnings.map((warning) => make("li", warning)));
}

function refreshRun(keepTime = false) {
    stopPlaying();
    a = runs.get(runA.value);
    b = runs.get(runB.value);
    summaryA = summarizeRun(a);
    summaryB = b ? summarizeRun(b) : {};
    const anchor = (run, lane) => run?.events.find((event) => event.kind === "lane_enter" && event.lane === lane && finite(event.t));
    const previousAlignment = alignment.value;
    alignment.replaceChildren(new Option("各趟起点", ""));
    const common = [];
    for (let lane = 1; lane <= 7; lane++) {
        const eventA = anchor(a, lane);
        const eventB = anchor(b, lane);
        if (eventA && eventB) {
            common.push({lane, a: eventA.t, b: eventB.t});
            alignment.append(new Option("进入第 " + lane + " 沟", String(lane)));
        }
    }
    alignment.disabled = !common.length;
    const aligned = common.find((entry) => String(entry.lane) === previousAlignment);
    if (aligned) alignment.value = previousAlignment;
    comparisonShift = aligned ? aligned.a - aligned.b : 0;
    view = preparePlots(a, b, summaryA.speedSource, summaryB.speedSource, comparisonShift);
    duration = Math.max(a.samples.at(-1)?.t ?? 0, b ? (b.samples.at(-1)?.t ?? 0) + comparisonShift : 0,
        a.events.reduce((max, event) => finite(event.t) ? Math.max(max, event.t) : max, 0));
    timeline.max = String(Math.max(duration, 0.01));
    timeline.disabled = play.disabled = duration === 0;
    const count = a.samples.length;
    const untimed = a.untimedSamples?.length ?? 0;
    byId("run-meta").textContent = source(a) + " A · " + count + " 个定时样本" +
        (untimed ? " · " + untimed + " 个未定时" : "") + " · " + a.raw.length + " 份原始文件";
    const hasA = a.samples.some(hasPose);
    byId("map-title").textContent = view.field ? "七沟场地" : "里程计局部轨迹";
    byId("map-note").textContent = view.field
        ? (hasA ? "场地米制坐标 · +X 向右，+Y 向场内" : "场地图可用 · 本趟日志缺少 XY")
        : (hasA ? "odom 坐标 · 登记外参后可叠到七沟图" : "本趟日志未包含可用 XY");
    if (b && !view.compareOnMap) byId("map-note").textContent += " · B 只参与曲线对比";
    const speedSource = (summary) => summary.speedSource === "fusion" ? "融合" : summary.speedSource === "fk" ? "FK" : "缺失";
    byId("chart-mode").textContent = (aligned ? "A/B 按进入第 " + aligned.lane + " 沟对齐"
        : b ? "A/B 按各自记录起点对齐" : "各曲线共享回放光标") +
        " · A 速度 " + speedSource(summaryA) + (b ? " / B " + speedSource(summaryB) : "") +
        (b && !view.compareOnMap ? " · 航向基准可能不同" : "");
    for (const input of metadata.querySelectorAll("input")) {
        input.value = input.name === "name" ? a.name : a.manifest[input.name] ??
            (input.name === "conditions" ? [a.manifest.surface, a.manifest.payload].filter(Boolean).join(" / ") : "");
    }
    registration.elements.tx.value = a.registration?.tx ?? "";
    registration.elements.ty.value = a.registration?.ty ?? "";
    registration.elements.heading.value = a.registration ? a.registration.yawRad * degrees : "";
    for (const input of registration.querySelectorAll("input, button")) input.disabled = a.frame === "field" || multiplePoseEpochs(a);
    byId("clear-registration").disabled = !a.registration || a.frame === "field";
    byId("attach-logs").disabled = a.source === "simulated";
    byId("attached-files").textContent = a.raw.length ? a.raw.map((record) => record.filename).join(" · ") : "无原始文件";
    refreshMetrics();
    refreshEvents();
    refreshHealth();
    seek(keepTime ? currentTime : 0);
}

function drawCursor() {
    if (selectedView !== "replay" || !view) return;
    const sa = nearest(a.samples, currentTime);
    const sb = b ? nearest(b.samples, currentTime - comparisonShift) : undefined;
    const label = sa?.lane ? "第 " + sa.lane + " 沟" : sa?.stage;
    byId("lane-label").textContent = label || (hasPose(sa) ? "当前车位" : "无当前位置");
    byId("cursor-time").value = format(currentTime, 2, " s");
    byId("duration").value = format(currentTime, 2) + " / " + format(duration, 2, " s");
    byId("speed-readout").value = "A " + format(speed(sa, false, summaryA.speedSource), 2) + " · 指令 " + format(speed(sa, true), 2) +
        (b ? " · B " + format(speed(sb, false, summaryB.speedSource), 2) : "");
    byId("yaw-readout").value = "A " + format(heading(sa, a), 1) + " · 参考 " + format(heading(sa, a, true), 1) +
        (b ? " · B " + format(heading(sb, b), 1) : "");
    drawMap(byId("map"), view, sa, sb);
    drawChart(byId("speed-chart"), view.speed, currentTime, duration, "speed", sa, sb);
    drawChart(byId("yaw-chart"), view.yaw, currentTime, duration, "yaw", sa, sb);
}

function seek(time) {
    currentTime = Math.max(0, Math.min(duration, time));
    timeline.value = String(currentTime);
    drawCursor();
}

function stopPlaying() {
    if (animation !== null) cancelAnimationFrame(animation);
    animation = null;
    lastFrame = undefined;
    play.textContent = "播放";
}

function step(now) {
    const delta = lastFrame === undefined ? 0 : (now - lastFrame) / 1000;
    lastFrame = now;
    currentTime = Math.min(duration, currentTime + delta * Number(byId("rate").value));
    if (now - lastPaint > 31 || currentTime >= duration) {
        seek(currentTime);
        lastPaint = now;
    }
    if (currentTime < duration) animation = requestAnimationFrame(step);
    else stopPlaying();
}

play.addEventListener("click", () => {
    if (animation !== null) {
        stopPlaying();
        return;
    }
    if (currentTime >= duration) seek(0);
    play.textContent = "暂停";
    animation = requestAnimationFrame(step);
});
timeline.addEventListener("input", () => {
    stopPlaying();
    seek(Number(timeline.value));
});
for (const id of ["speed-chart", "yaw-chart"]) {
    const canvas = byId(id);
    const pointer = (event) => {
        const rect = canvas.getBoundingClientRect();
        seek((event.clientX - rect.left - plotGutters.left) /
            (rect.width - plotGutters.left - plotGutters.right) * duration);
    };
    canvas.addEventListener("pointerdown", (event) => {
        if (duration === 0 || event.button !== 0) return;
        stopPlaying();
        canvas.setPointerCapture(event.pointerId);
        pointer(event);
    });
    canvas.addEventListener("pointermove", (event) => {
        if (canvas.hasPointerCapture(event.pointerId)) pointer(event);
    });
}

async function readText(file) {
    const bytes = new Uint8Array(await file.arrayBuffer());
    const utf16 = bytes[0] === 0xff && bytes[1] === 0xfe ? "utf-16le"
        : bytes[0] === 0xfe && bytes[1] === 0xff ? "utf-16be" : null;
    if (utf16) return new TextDecoder(utf16).decode(bytes);
    try {
        return new TextDecoder("utf-8", {fatal: true}).decode(bytes);
    } catch {
        return new TextDecoder("gb18030").decode(bytes);
    }
}

function readRun(text, filename) {
    const head = text.trimStart();
    if (head.startsWith("{")) {
        let archive;
        try { archive = JSON.parse(head); } catch { /* NDJSON is line-oriented. */ }
        if (archive?.schema === "longinus.run") {
            const run = parseArchive(archive);
            if (run.registration && multiplePoseEpochs(run)) {
                delete run.registration;
                run.warnings.push("已停用跨时钟段的场地登记。");
            }
            return run;
        }
    }
    const data = parseLog(text, filename);
    const id = crypto.randomUUID();
    return {
        id, name: filename.replace(/\.[^.]+$/, "") || filename, source: "imported", frame: data.frame,
        createdAt: new Date().toISOString(), samples: data.samples, events: data.events,
        warnings: data.warnings, untimedSamples: data.untimedSamples, availability: data.availability,
        timeSources: data.timeSources,
        manifest: {runId: id, bootId: null, firmware: null, parameterRevision: null, calibrationRevision: null, status: "imported"},
        raw: [{filename, format: data.format, text: data.rawText ?? text}],
    };
}

byId("import-button").addEventListener("click", () => byId("import-files").click());
byId("import-files").addEventListener("change", async (event) => {
    const files = [...event.target.files];
    event.target.value = "";
    if (!files.length) return;
    const added = [];
    const errors = [];
    for (const file of files) {
        try {
            const run = readRun(await readText(file), file.name);
            if (runs.has(run.id)) {
                const original = runs.get(run.id);
                if (JSON.stringify(makeArchive(original)) !== JSON.stringify(makeArchive(run))) {
                    throw new Error("归档 ID 已存在且内容不同。请先保存当前档案后刷新，再载入需要的版本。");
                }
            } else runs.set(run.id, run);
            added.push(run.id);
        } catch (error) {
            errors.push(file.name + " · " + error.message);
        }
    }
    if (added.length) {
        selectView("replay");
        updateRunOptions(added[0], added[1] ?? "");
    }
    notify([added.length ? "已载入 " + added.length + " 趟。" : "", ...errors].filter(Boolean).join("\n"), Boolean(errors.length));
});

byId("attach-logs").addEventListener("click", () => byId("attach-files").click());
byId("attach-files").addEventListener("change", async (event) => {
    const files = [...event.target.files];
    event.target.value = "";
    const target = a;
    if (!files.length || target.source !== "imported") return;
    const errors = [];
    let added = 0;
    for (const file of files) {
        try {
            const data = parseLog(await readText(file), file.name);
            if (target.raw.some((record) => record.filename === file.name && record.text === data.rawText)) continue;
            target.raw.push({filename: file.name, format: data.format, text: data.rawText});
            added++;
        } catch (error) {
            errors.push(file.name + " · " + error.message);
        }
    }
    if (target === a) refreshRun(true);
    notify(["已附加 " + added + " 份原始日志。回放仍使用主日志的时钟。", ...errors].join("\n"), Boolean(errors.length));
});

runA.addEventListener("change", () => updateRunOptions(runA.value, runB.value));
runB.addEventListener("change", () => updateRunOptions(runA.value, runB.value));
alignment.addEventListener("change", () => {
    refreshRun(true);
    const lane = Number(alignment.value);
    const anchor = a.events.find((event) => event.kind === "lane_enter" && event.lane === lane && finite(event.t));
    if (anchor) seek(anchor.t);
});
byId("export-run").addEventListener("click", () => {
    try {
        download(a.name + ".longinus-run.json", makeArchive(a));
        notify("已下载本趟归档，原始日志与解析结果包含在同一个文件中。");
    } catch (error) {
        notify(error.message, true);
    }
});

const toggle = byId("run-details-toggle");
toggle.addEventListener("click", () => {
    const details = byId("run-details");
    details.hidden = !details.hidden;
    toggle.setAttribute("aria-expanded", String(!details.hidden));
});

metadata.addEventListener("submit", (event) => {
    event.preventDefault();
    const values = new FormData(metadata);
    const name = values.get("name").trim();
    if (!name) {
        notify("请输入试跑名称。", true);
        return;
    }
    a.name = name;
    for (const [key, value] of values) {
        if (key !== "name") a.manifest[key] = value.trim() || null;
    }
    updateRunOptions(a.id, b?.id);
    notify("已更新档案信息。保存本趟归档即可带走。");
});

function multiplePoseEpochs(run) {
    return new Set(run.samples.filter(hasPose).map((sample) => sample.epoch ?? 0)).size > 1;
}

function applyRegistration(tx, ty, yawRad) {
    if (a.frame === "field" || multiplePoseEpochs(a) || ![tx, ty, yawRad].every(finite)) return;
    a.registration = {tx, ty, yawRad};
    refreshRun(true);
    notify("已登记场地变换，保存本趟归档即可保留。");
}

registration.addEventListener("submit", (event) => {
    event.preventDefault();
    applyRegistration(registration.elements.tx.valueAsNumber, registration.elements.ty.valueAsNumber,
        registration.elements.heading.valueAsNumber / degrees);
});
byId("clear-registration").addEventListener("click", () => {
    delete a.registration;
    refreshRun(true);
    notify("已移除场地变换。");
});
byId("load-calibration").addEventListener("click", () => byId("calibration-file").click());
byId("calibration-file").addEventListener("change", async (event) => {
    const file = event.target.files[0];
    const target = a;
    event.target.value = "";
    if (!file) return;
    try {
        const transform = registrationFromCalibration(JSON.parse(await readText(file)));
        if (!transform || ![transform.xM, transform.yM, transform.yawRad].every(finite)) {
            throw new Error("此标定草案没有可用的场地变换。");
        }
        if (a !== target) throw new Error("试跑已切换，请在目标试跑下重新选择标定文件。");
        applyRegistration(transform.xM, transform.yM, transform.yawRad);
    } catch (error) {
        notify(error.message, true);
    }
});

const fields = [
    ["cruiseMps", "长沟巡航上限 (m/s)", 0.8, 0.05, 2.11656, 0.00001, "方案初值"],
    ["linkMps", "换沟上限 (m/s)", 0.5, 0.05, 1, 0.01, "方案初值"],
    ["doorMps", "进出门上限 (m/s)", 0.35, 0.05, 0.8, 0.01, "方案初值"],
    ["accelMps2", "加速度上限 (m/s²)", 1, 0.1, 5, 0.01, "方案初值"],
    ["brakeMps2", "可用制动减速度 (m/s²)", 1.5, 0.1, 5, 0.01, "假设，待实测"],
    ["latencyMs", "端到端延迟 (ms)", 40, 1, 1000, 1, "假设，待实测"],
    ["stopMarginM", "制动额外余量 (m)", 0.08, 0, 1, 0.01, "方案初值"],
    ["yawKp", "航向 Kp (s⁻¹)", 48, 0, 100, 0.01, "基线固件"],
    ["yawKd", "航向 Kd", 2, 0, 20, 0.01, "基线固件"],
    ["laserMaxAgeMs", "赛用测距时效上限 (ms)", 80, 1, 250, 1, "方案初值，待实测"],
];

function mountTuning(element) {
    const form = make("form");
    form.className = "tuning-form";
    form.append(make("p", "用途为试跑参数对照。带方案初值或假设的项需按实测复核。"));
    for (const [key, label, value, min, max, step, note] of fields) {
        const wrapper = make("label", label);
        const input = make("input");
        Object.assign(input, {name: key, type: "number", value: String(value), min, max, step, required: true});
        wrapper.append(input, make("small", note));
        form.append(wrapper);
    }
    const actions = make("div");
    actions.className = "tuning-actions";
    const submit = make("button", "导出参数 JSON");
    submit.className = "primary";
    submit.type = "submit";
    const reset = make("button", "恢复默认草案");
    reset.type = "reset";
    const distance = make("output");
    distance.setAttribute("aria-live", "polite");
    actions.append(submit, reset, distance);
    form.append(actions);
    const values = () => Object.fromEntries(fields.map(([key]) => [key, form.elements[key].valueAsNumber]));
    function estimate() {
        const v = values();
        const consistent = v.linkMps <= v.cruiseMps && v.doorMps <= v.cruiseMps;
        form.elements.cruiseMps.setCustomValidity(consistent ? "" : "巡航上限须不小于换沟和进出门上限。");
        const stop = v.cruiseMps * v.latencyMs / 1000 + v.cruiseMps ** 2 / (2 * v.brakeMps2) + v.stopMarginM;
        distance.value = form.checkValidity() ? "理想制动距离含余量 " + format(stop, 3, " m") : "请核对输入范围和速度层级";
    }
    form.addEventListener("input", estimate);
    form.addEventListener("reset", () => setTimeout(estimate, 0));
    form.addEventListener("submit", (event) => {
        event.preventDefault();
        if (!form.reportValidity()) return;
        const parameters = values();
        const changes = Object.fromEntries(fields.filter(([key, , initial]) => parameters[key] !== initial)
            .map(([key, , initial]) => [key, {from: initial, to: parameters[key]}]));
        download("longinus-parameters.json", {
            schema: "longinus.parameters", version: 1, status: "offline-draft", exportedAt: new Date().toISOString(),
            baseline: {branch: "feature/longinus-stm32-development", commit: base, standardCommit: sdk},
            referenceRun: {id: a.id, source: a.source, profile: a.manifest.parameterRevision ?? null},
            parameters, changes,
        });
        notify("已导出参数草案。");
    });
    element.append(form);
    estimate();
}

function selectView(name) {
    stopPlaying();
    selectedView = name;
    for (const button of document.querySelectorAll("[data-view]")) {
        const selected = button.dataset.view === name;
        button.setAttribute("aria-pressed", String(selected));
        byId(button.dataset.view + "-view").hidden = !selected;
    }
    requestAnimationFrame(drawCursor);
}

for (const button of document.querySelectorAll("[data-view]")) {
    button.addEventListener("click", () => {
        notify("");
        selectView(button.dataset.view);
    });
}
const resize = new ResizeObserver(() => drawCursor());
for (const id of ["map", "speed-chart", "yaw-chart"]) resize.observe(byId(id));
updateRunOptions("sim-a", "sim-b");
mountTuning(byId("tuning"));
mountCalibration(byId("calibration"), {download});

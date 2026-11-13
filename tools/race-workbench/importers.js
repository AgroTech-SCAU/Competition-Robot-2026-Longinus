// Formats emitted by assemble_log.c and tools/capture_odom_warmup.ps1.
const VOFA_FIELDS = [
    ['ms', 'sourceTimeMs'], ['x', 'x'], ['y', 'y'], ['yaw', 'yaw'],
    ['vx_cmd', 'vxCmd'], ['vy_cmd', 'vyCmd'], ['vx_fk', 'vxFk'],
    ['vy_fk', 'vyFk'], ['wz_fk', 'wzFk'], ['gz', 'gz'],
    ['vx', 'vx'], ['vy', 'vy'], ['wz', 'wz'], ['rem', 'remaining'], ['active', 'active'],
];
const VOFA_NAME = VOFA_FIELDS.map(([name]) => name).join(String.raw`\s*,\s*`);
const VOFA_LINE = new RegExp(`(?:^|[^\\w])${VOFA_NAME}\\s*:\\s*(.*)$`, 'i');
const VOFA_HEADER = new RegExp(`^${VOFA_NAME}$`, 'i');
const ANSI = /\x1b(?:\[[0-?]*[ -/]*[@-~]|\][^\x07]*(?:\x07|\x1b\\))/g;
const DECIMAL = /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?$/i;
const UINT32_MODULUS = 2 ** 32;

const DEBUG_FIELDS = {
    x: ['s_odom.odom.x', 's_odom.odom_interface.x'],
    y: ['s_odom.odom.y', 's_odom.odom_interface.y'],
    yaw: ['s_odom.angle.z'],
    vxCmd: ['s_chassis.kine.control.vx'],
    vyCmd: ['s_chassis.kine.control.vy'],
    wzCmd: ['s_chassis.kine.control.wz'],
    vxFk: ['s_odom.wheel_corrected.x'],
    vyFk: ['s_odom.wheel_corrected.y'],
    wzFk: ['s_odom.wheel_corrected.z'],
    vxFkRaw: ['s_odom.wheel_raw.x'],
    vyFkRaw: ['s_odom.wheel_raw.y'],
    wzFkRaw: ['s_odom.wheel_raw.z'],
    gz: ['s_odom.gyro_corrected.z'],
    gzRaw: ['s_odom.gyro.z', 's_bmi088_sample.gyro.z'],
    vx: ['s_odom.velocity.x'],
    vy: ['s_odom.velocity.y'],
    wz: ['s_odom.velocity.z'],
};
const FUSED_FIELDS = ['x', 'y', 'yaw', 'vx', 'vy', 'wz'];
const WHEEL_FIELDS = ['vxFk', 'vyFk', 'wzFk', 'vxFkRaw', 'vyFkRaw', 'wzFkRaw'];

function number(value) {
    if (typeof value === 'number') return Number.isFinite(value) ? value : undefined;
    if (typeof value === 'boolean') return Number(value);
    if (typeof value !== 'string' || !DECIMAL.test(value.trim())) return undefined;
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : undefined;
}

function flag(value) {
    if (typeof value === 'string' && /^true$/i.test(value.trim())) return true;
    if (typeof value === 'string' && /^false$/i.test(value.trim())) return false;
    const parsed = number(value);
    return parsed === undefined ? undefined : parsed !== 0;
}

function uint32(value) {
    const parsed = number(value);
    return Number.isInteger(parsed) && parsed >= 0 && parsed < UINT32_MODULUS ? parsed : undefined;
}

function firstField(fields, names) {
    for (const name of names) {
        if (Object.hasOwn(fields, name)) return fields[name];
    }
    return undefined;
}

function addLeaf(fields, path, value) {
    const parsed = number(value);
    if (parsed !== undefined) fields[path] = parsed;
    else if (typeof value === 'string' && /^(?:true|false)$/i.test(value.trim())) {
        fields[path] = Number(/^true$/i.test(value.trim()));
    }
}

function flatten(value, path, fields) {
    if (value === null || typeof value !== 'object') {
        addLeaf(fields, path, value);
    } else if (value.tree && typeof value.tree === 'object') {
        flatten(value.tree, path, fields);
    } else if (Array.isArray(value.members)) {
        for (const member of value.members) {
            if (typeof member?.name === 'string') flatten(member, `${path}.${member.name}`, fields);
        }
    } else if (Array.isArray(value.elements)) {
        for (const element of value.elements) {
            if (element?.index !== undefined) flatten(element, `${path}[${element.index}]`, fields);
        }
    } else if (Object.hasOwn(value, 'valueText') || Object.hasOwn(value, 'value')) {
        const scalar = number(value.valueText) ?? number(value.value);
        addLeaf(fields, path, scalar ?? flag(value.valueText) ?? flag(value.value));
    } else if (Array.isArray(value)) {
        value.forEach((element, i) => flatten(element, `${path}[${i}]`, fields));
    } else {
        for (const [name, element] of Object.entries(value)) flatten(element, `${path}.${name}`, fields);
    }
}

function parseDebugRow(item, lineNumber) {
    if (!item.values || typeof item.values !== 'object' || Array.isArray(item.values)) return null;
    const fields = Object.create(null);
    for (const [name, value] of Object.entries(item.values)) {
        if (name.includes('::') || /^s_[a-zA-Z]/.test(name)) {
            flatten(value, name.split('::').at(-1), fields);
        }
    }
    if (!Object.keys(fields).length) return null;

    const sample = { fields, lineNumber };
    const firmware = item.firmwareConsistency;
    const firmwareStatus = firmware && typeof firmware === 'object' ? firmware.status : firmware;
    if (typeof firmwareStatus === 'string') sample.firmwareStatus = firmwareStatus;
    const mismatched = sample.firmwareStatus === 'mismatch';
    const fusionReady = flag(fields['s_odom.fusion_ready']);
    const imuReady = flag(fields['s_odom.imu_ready']);
    const wheelValid = flag(fields['s_odom.wheel_valid']);
    if (mismatched) sample.poseValid = false;
    else if (fusionReady !== undefined) sample.poseValid = fusionReady;
    for (const [key, aliases] of Object.entries(DEBUG_FIELDS)) {
        if (mismatched) break;
        if (fusionReady === false && FUSED_FIELDS.includes(key)) continue;
        if (key === 'yaw' && fusionReady === undefined && imuReady === false) continue;
        if (wheelValid === false && WHEEL_FIELDS.includes(key)) continue;
        const value = firstField(fields, aliases);
        if (value !== undefined) sample[key] = value;
    }

    const outputEnabled = flag(fields.s_output_enabled);
    if (!mismatched && outputEnabled !== undefined) sample.outputEnabled = outputEnabled;
    if (item.partial === true || (Array.isArray(item.variableErrors) && item.variableErrors.length)) {
        sample.partial = true;
    }
    const elfSha256 = item.elf?.sha256 ?? item.elfSha256;
    if (typeof elfSha256 === 'string') sample.elfSha256 = elfSha256;

    const mcuUs = uint32(firstField(fields, ['s_odom.sample_time_us', 's_bmi088_sample.gyro_timestamp_us']));
    const hostMs = number(item.timestamp);
    if (!mismatched && mcuUs !== undefined) {
        return { sample, clock: 'debug-mcu-us', tick: mcuUs, alternate: hostMs };
    }
    if (hostMs !== undefined) return { sample, clock: 'debug-host-ms', tick: hostMs };
    return { sample };
}

function parseVofaRow(payload, lineNumber) {
    const columns = payload.split(',').map(column => column.trim());
    if (columns.length !== VOFA_FIELDS.length) return null;
    const time = uint32(columns[0]);
    if (time === undefined || !/^(?:0|1|true|false)$/i.test(columns.at(-1))) return null;
    const sample = { lineNumber };
    for (const [i, column] of columns.entries()) {
        if (i === 0) continue;
        if (VOFA_FIELDS[i][1] === 'active') {
            sample.active = flag(column);
            continue;
        }
        const value = number(column);
        if (value !== undefined) sample[VOFA_FIELDS[i][1]] = value;
        else if (!/^(?:[+-]?(?:nan|inf(?:inity)?)|null|)$/i.test(column)) return null;
    }
    if (!Object.keys(sample).some(key => key !== 'lineNumber' && key !== 'active')) return null;
    return { sample, clock: 'vofa-ms', tick: time };
}

function serialEvent(line, lineNumber) {
    const match = line.match(/(?:\[(?:INFO|WARN|ERROR|DEBUG)\]\s*)?((?:ODOM ROUTE|LASER (?:POS|RELOC|CONFIG|CAN3)|Heartbeat state=|APP_RUNTIME (?:fault|safe stop)|SCHED ticks=|BOOT\s).*)/i);
    if (!match) return null;
    const label = match[1];
    let kind = 'status';
    if (/^ODOM ROUTE/i.test(label)) kind = 'route';
    else if (/^LASER (?:POS|RELOC|CONFIG|CAN3)/i.test(label)) kind = 'laser';
    else if (/^APP_RUNTIME/i.test(label)) kind = 'fault';
    else if (/^BOOT/i.test(label)) kind = 'boot';
    else if (/^SCHED/i.test(label)) kind = 'scheduler';

    const event = { t: null, kind, label, lineNumber };
    const segment = label.match(/(?:\bseg=|\bsegment=|ODOM ROUTE\s+)(\d+)/i);
    if (segment) event.segment = Number(segment[1]);
    const laser = label.match(/^LASER POS\s+seg=\d+\s+frame=(\d+)\s+F=(\d+)\s+B=(\d+)\s+L=(\d+)\s+R=(\d+)\s+mm\s+x=(\S+)\s+y=(\S+)\s+m\s+fix=0x([0-9a-f]+)/i);
    if (laser) {
        const [, frame, front, rear, left, right, x, y, fixMask] = laser;
        event.laser = {
            frame: Number(frame), frontMm: Number(front), rearMm: Number(rear),
            leftMm: Number(left), rightMm: Number(right), x: number(x), y: number(y),
            fixMask: Number.parseInt(fixMask, 16),
        };
    }
    return event;
}

function debugEvent(item, lineNumber) {
    if (!['header', 'summary', 'error', 'warning', 'done', 'end', 'start'].includes(item.type)) return null;
    const errors = Array.isArray(item.variableErrors) ? item.variableErrors.length : 0;
    return {
        t: null,
        kind: 'diagnostic',
        label: `调试记录 ${item.type}${errors ? ` · ${errors} 项读取异常` : ''}`,
        lineNumber,
    };
}

function clockStep(previous, current) {
    if (previous.clock !== current.clock || current.clock === undefined) return undefined;
    const micro = current.clock === 'debug-mcu-us';
    const divisor = micro ? 1e6 : 1e3;
    const delta = current.tick - previous.tick;
    if (delta >= 0) return delta / divisor;
    // Only accept a crossing very close to the boundary as a uint32 rollover.
    if (current.clock !== 'debug-host-ms' &&
        previous.tick > UINT32_MODULUS * 0.95 && current.tick < UINT32_MODULUS * 0.05) {
        const wrapped = (delta + UINT32_MODULUS) / divisor;
        if (wrapped <= (micro ? 120 : 10)) return wrapped;
    }
    return undefined;
}

function median(values) {
    const sorted = [...values].sort((a, b) => a - b);
    const mid = Math.floor(sorted.length / 2);
    return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
}

function addTimes(records) {
    const samples = [];
    const untimedSamples = [];
    const timeSources = Object.create(null);
    const intervals = new Map();
    const stats = { gaps: 0, wraps: 0, resets: 0, switches: 0, duplicates: 0 };
    let previous = null;
    let currentTime = 0;
    let epoch = 0;

    for (const record of records) {
        if (!record.clock) {
            untimedSamples.push(record.sample);
            if (previous) epoch++;
            previous = null;
            continue;
        }
        let gapStep;
        let breakBefore = samples.length > 0 && previous === null;
        if (previous) {
            gapStep = record.bootBoundary ? undefined : clockStep(previous, record);
            if (gapStep === undefined) {
                if (previous.clock === record.clock) stats.resets++;
                else stats.switches++;
                epoch++;
                breakBefore = true;
            } else if (gapStep === 0) {
                stats.duplicates++;
                continue;
            } else {
                if (record.tick < previous.tick) stats.wraps++;
                currentTime += gapStep;
                if (!intervals.has(record.clock)) intervals.set(record.clock, []);
                intervals.get(record.clock).push(gapStep);
            }
        }
        const timeSource = record.clock;
        const sourceTimeMs = timeSource === 'debug-mcu-us' ? record.tick / 1e3 : record.tick;
        const sample = { ...record.sample, t: currentTime, epoch, timeSource, sourceTimeMs, breakBefore };
        if (record.alternate !== undefined) sample.hostTimeMs = record.alternate;
        samples.push({ sample, gapStep });
        timeSources[timeSource] = (timeSources[timeSource] || 0) + 1;
        previous = record;
    }

    const gapLimits = new Map();
    for (const [clock, durations] of intervals) {
        // VOFA's cadence is fixed. Probe captures can choose their own interval.
        const limit = clock === 'vofa-ms' ? 0.35 :
            durations.length >= 3 ? Math.max(0.75, 3 * median(durations)) : 0.75;
        gapLimits.set(clock, limit);
    }
    for (const row of samples) {
        if (row.gapStep > gapLimits.get(row.sample.timeSource)) {
            row.sample.breakBefore = true;
            stats.gaps++;
        }
    }
    return { samples: samples.map(row => row.sample), untimedSamples, timeSources, stats };
}

function addEventContext(events, samples) {
    let i = 0;
    for (const event of events) {
        while (i < samples.length && samples[i].lineNumber < event.lineNumber) i++;
        if (i > 0) {
            event.afterSampleT = samples[i - 1].t;
            event.epoch = samples[i - 1].epoch;
        }
    }
}

function isBareVofa(line) {
    return /^\s*[+-]?\d+\s*,/.test(line) && line.split(',').length === VOFA_FIELDS.length;
}

/**
 * Read a saved ASCII VOFA serial stream or EmberProbe read.js NDJSON.
 * Time uses one source per epoch. Untimed text events retain file order only.
 */
export function parseLog(text, filename = '日志') {
    if (typeof text !== 'string' || !text.trim()) throw new Error('日志文件为空。');
    const warnings = [];
    const events = [];
    const records = [];
    let serial = false;
    let debug = false;
    let csvHeader = false;
    let bareCount = 0;
    let badVofa = 0;
    let badDebug = 0;
    let partialDebug = 0;
    let firmwareMismatches = 0;
    let serialBootPending = false;
    const lines = text.replace(/^\uFEFF/, '').split(/\r\n|\n|\r/);
    for (const [i, raw] of lines.entries()) {
        const line = raw.replace(ANSI, '').trim();
        if (!line) continue;
        const lineNumber = i + 1;

        const named = line.match(VOFA_LINE);
        const prefixed = line.match(/(?:^|[^\w])VOFA\s*:\s*(.*)$/i);
        const bare = !named && !prefixed && (csvHeader || isBareVofa(line)) && /^\s*[+-]?\d+\s*,/.test(line);
        if (VOFA_HEADER.test(line)) {
            csvHeader = true;
            serial = true;
        } else if (named || prefixed || bare) {
            const record = parseVofaRow(named?.[1] ?? prefixed?.[1] ?? line, lineNumber);
            if (record) {
                if (bare && !csvHeader) bareCount++;
                if (serialBootPending) {
                    record.bootBoundary = true;
                    serialBootPending = false;
                }
                records.push(record);
                serial = true;
            } else {
                badVofa++;
            }
        } else if (line.startsWith('{')) {
            let item;
            try {
                item = JSON.parse(line);
            } catch {
                badDebug++;
                continue;
            }
            if (item.type === 'sample') {
                const record = parseDebugRow(item, lineNumber);
                if (record) {
                    if (record.sample.partial) partialDebug++;
                    if (record.sample.firmwareStatus === 'mismatch') firmwareMismatches++;
                    records.push(record);
                    debug = true;
                } else {
                    badDebug++;
                }
            } else {
                const event = debugEvent(item, lineNumber);
                if (event) events.push(event);
            }
        } else {
            const event = serialEvent(line, lineNumber);
            if (event) {
                serial = true;
                events.push(event);
                if (event.kind === 'boot') serialBootPending = true;
            }
        }
    }

    if (!records.length && !events.some(event => event.kind !== 'diagnostic')) {
        const malformed = badVofa + badDebug;
        const detail = malformed ? `发现 ${malformed} 行损坏或截断。 ` : '';
        throw new Error(`${detail}未找到可用的 Longinus VOFA 或调试 NDJSON 数据。`);
    }
    const { samples, untimedSamples, timeSources, stats } = addTimes(records);
    addEventContext(events, samples);
    const format = serial && debug ? 'vofa+debug' : debug ? 'debug' : 'vofa';
    const availability = {};
    const allSamples = [...samples, ...untimedSamples];
    for (const key of new Set([...VOFA_FIELDS.slice(1).map(([, key]) => key), ...Object.keys(DEBUG_FIELDS), 'outputEnabled'])) {
        availability[key] = allSamples.some(sample => sample[key] !== undefined);
    }
    availability.xy = allSamples.some(sample => Number.isFinite(sample.x) && Number.isFinite(sample.y));
    availability.laserRange = events.some(event => event.laser !== undefined);
    availability.timedEvents = false;
    availability.rawFields = allSamples.some(sample => sample.fields !== undefined);

    if (bareCount) warnings.push(`${bareCount} 行无列名数据按本分支的固定 14 列 VOFA 顺序解析。`);
    if (badVofa) warnings.push(`跳过 ${badVofa} 行损坏或截断的 VOFA 帧。`);
    if (badDebug) warnings.push(`跳过 ${badDebug} 行损坏或缺少可读字段的调试记录。`);
    if (stats.duplicates) warnings.push(`跳过 ${stats.duplicates} 条同源重复时间戳，原始行仍保留。`);
    if (stats.resets) warnings.push(`检测到 ${stats.resets} 次时间回退，回放按独立时钟段显示。`);
    if (stats.wraps) warnings.push(`已展开 ${stats.wraps} 次 MCU 32 位时间戳回绕。`);
    if (stats.switches) warnings.push(`检测到 ${stats.switches} 次时钟来源切换，未对不同来源做时间对齐。`);
    if (stats.gaps) warnings.push(`标记了 ${stats.gaps} 处采样缺口，回放不跨缺口连线。`);
    if (untimedSamples.length) warnings.push(`${untimedSamples.length} 条调试记录缺少时间戳，仅保留字段和原始记录。`);
    if (partialDebug) warnings.push(`${partialDebug} 条调试快照仅部分读取。`);
    if (firmwareMismatches) warnings.push(`${firmwareMismatches} 条调试快照报告固件与 ELF 不匹配，已停用其解算曲线，仅保留原始字段。`);
    if (debug) warnings.push('调试快照的变量可能来自不同控制周期，使用采样间隔和有效标志判断可比性。');
    if (events.length) warnings.push('文本事件没有精确采样时刻，仅保留文件顺序。');
    if (!samples.length) warnings.push('此文件没有可播放的带时间戳样本。');

    return {
        format, frame: availability.xy ? 'odom' : 'unknown', samples, untimedSamples,
        events, availability, timeSources, warnings, rawText: text, filename,
    };
}

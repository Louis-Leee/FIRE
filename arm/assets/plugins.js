var __create = Object.create;
var __defProp = Object.defineProperty;
var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
var __getOwnPropNames = Object.getOwnPropertyNames;
var __getProtoOf = Object.getPrototypeOf;
var __hasOwnProp = Object.prototype.hasOwnProperty;
var __commonJS = (cb, mod) => function __require() {
  try {
    return mod || (0, cb[__getOwnPropNames(cb)[0]])((mod = { exports: {} }).exports, mod), mod.exports;
  } catch (e) {
    throw mod = 0, e;
  }
};
var __copyProps = (to, from, except, desc) => {
  if (from && typeof from === "object" || typeof from === "function") {
    for (let key of __getOwnPropNames(from))
      if (!__hasOwnProp.call(to, key) && key !== except)
        __defProp(to, key, { get: () => from[key], enumerable: !(desc = __getOwnPropDesc(from, key)) || desc.enumerable });
  }
  return to;
};
var __toESM = (mod, isNodeMode, target) => (target = mod != null ? __create(__getProtoOf(mod)) : {}, __copyProps(
  // If the importer is in node compatibility mode or this is not an ESM
  // file that has been converted to a CommonJS file using a Babel-
  // compatible transform (i.e. "__esModule" has not been set), then set
  // "default" to the CommonJS "module.exports" for node compatibility.
  isNodeMode || !mod || !mod.__esModule ? __defProp(target, "default", { value: mod, enumerable: true }) : target,
  mod
));

// src/engine/plugin-three-shim.cjs
var require_plugin_three_shim = __commonJS({
  "src/engine/plugin-three-shim.cjs"(exports, module) {
    "use strict";
    var THREE4 = globalThis.__mjswanThree;
    if (!THREE4) {
      throw new Error(
        "mjswan: the engine must be loaded before a custom-MDP plugin (three not initialized)."
      );
    }
    module.exports = THREE4;
  }
});

// src/core/observation/ObservationBase.ts
var ObservationBase = class {
  constructor(runner, config) {
    this.runner = runner;
    this.config = config;
  }
};

// ../../../../../../web/plugins/session.ts
var session = {
  controller: null
};

// ../../../../../../web/plugins/mjmath.ts
function norm3(v) {
  return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
function norm4(q) {
  return Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}
function normalize3(v, eps = 1e-9) {
  const n = Math.max(norm3(v), eps);
  return [v[0] / n, v[1] / n, v[2] / n];
}
function normalize4(q, eps = 1e-9) {
  const n = Math.max(norm4(q), eps);
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}
function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  ];
}
function quatMul(q1, q2) {
  const w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
  const w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
  return [
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
  ];
}
function quatConjugate(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}
function quatInv(q) {
  const s = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  return [q[0] / s, -q[1] / s, -q[2] / s, -q[3] / s];
}
function quatPositiveReal(q) {
  return q[0] < 0 ? [-q[0], -q[1], -q[2], -q[3]] : [q[0], q[1], q[2], q[3]];
}
function quatApply(q, v) {
  const xyz = [q[1], q[2], q[3]];
  const c = cross(xyz, v);
  const t = [2 * c[0], 2 * c[1], 2 * c[2]];
  const c2 = cross(xyz, t);
  return [
    v[0] + q[0] * t[0] + c2[0],
    v[1] + q[0] * t[1] + c2[1],
    v[2] + q[0] * t[2] + c2[2]
  ];
}
function quatApplyInverse(q, v) {
  const xyz = [q[1], q[2], q[3]];
  const c = cross(xyz, v);
  const t = [2 * c[0], 2 * c[1], 2 * c[2]];
  const c2 = cross(xyz, t);
  return [
    v[0] - q[0] * t[0] + c2[0],
    v[1] - q[0] * t[1] + c2[1],
    v[2] - q[0] * t[2] + c2[2]
  ];
}
function axisAngleFromQuat(qIn, eps = 1e-6) {
  const s = 1 - 2 * (qIn[0] < 0 ? 1 : 0);
  const q = [qIn[0] * s, qIn[1] * s, qIn[2] * s, qIn[3] * s];
  const mag = Math.sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  const half = Math.atan2(mag, q[0]);
  const angle = 2 * half;
  const ratio = Math.abs(angle) > eps ? Math.sin(half) / (angle === 0 ? 1 : angle) : 0.5 - angle * angle / 48;
  return [q[1] / ratio, q[2] / ratio, q[3] / ratio];
}
function quatFromAngleAxis(angle, axis) {
  const theta = angle / 2;
  const a = normalize3(axis);
  const st = Math.sin(theta);
  return normalize4([Math.cos(theta), a[0] * st, a[1] * st, a[2] * st]);
}
function matrixFromQuat(q) {
  const r = q[0], i = q[1], j = q[2], k = q[3];
  const twoS = 2 / (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return [
    1 - twoS * (j * j + k * k),
    twoS * (i * j - k * r),
    twoS * (i * k + j * r),
    twoS * (i * j + k * r),
    1 - twoS * (i * i + k * k),
    twoS * (j * k - i * r),
    twoS * (i * k - j * r),
    twoS * (j * k + i * r),
    1 - twoS * (i * i + j * j)
  ];
}
function quatFromMatrix(a) {
  const tr = a[0] + a[4] + a[8];
  let q;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q = [0.25 * s, (a[7] - a[5]) / s, (a[2] - a[6]) / s, (a[3] - a[1]) / s];
  } else if (a[0] > a[4] && a[0] > a[8]) {
    const s = Math.sqrt(1 + a[0] - a[4] - a[8]) * 2;
    q = [(a[7] - a[5]) / s, 0.25 * s, (a[1] + a[3]) / s, (a[2] + a[6]) / s];
  } else if (a[4] > a[8]) {
    const s = Math.sqrt(1 + a[4] - a[0] - a[8]) * 2;
    q = [(a[2] - a[6]) / s, (a[1] + a[3]) / s, 0.25 * s, (a[5] + a[7]) / s];
  } else {
    const s = Math.sqrt(1 + a[8] - a[0] - a[4]) * 2;
    q = [(a[3] - a[1]) / s, (a[2] + a[6]) / s, (a[5] + a[7]) / s, 0.25 * s];
  }
  return quatPositiveReal(normalize4(q));
}
function rot6d(q) {
  const m = matrixFromQuat(q);
  return [m[0], m[1], m[3], m[4], m[6], m[7]];
}
function rot6dToMatrix(r6) {
  const a0 = [r6[0], r6[2], r6[4]];
  const a1 = [r6[1], r6[3], r6[5]];
  const c1 = normalize3(a0, 1e-12);
  const d = c1[0] * a1[0] + c1[1] * a1[1] + c1[2] * a1[2];
  const c2 = normalize3(
    [a1[0] - d * c1[0], a1[1] - d * c1[1], a1[2] - d * c1[2]],
    1e-12
  );
  const c3 = cross(c1, c2);
  return [c1[0], c2[0], c3[0], c1[1], c2[1], c3[1], c1[2], c2[2], c3[2]];
}
function rot6dToQuat(r6) {
  return quatFromMatrix(rot6dToMatrix(r6));
}
function slerpToward(prev, next, alpha) {
  const dq = quatPositiveReal(quatMul(next, quatConjugate(prev)));
  const aa = axisAngleFromQuat(dq);
  const rotvec = [aa[0] * alpha, aa[1] * alpha, aa[2] * alpha];
  const angle = norm3(rotvec);
  const den = Math.max(angle, 1e-12);
  const axis = [rotvec[0] / den, rotvec[1] / den, rotvec[2] / den];
  const q = quatMul(quatFromAngleAxis(angle, axis), prev);
  const n = Math.max(norm4(q), 1e-8);
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}
function clipNorm(v, maxNorm) {
  const n = norm3(v);
  const k = Math.min(maxNorm / Math.max(n, 1e-8), 1);
  return [v[0] * k, v[1] * k, v[2] * k];
}
function poseInFrame(fp, fq, p, q) {
  return {
    pos: quatApplyInverse(fq, [p[0] - fp[0], p[1] - fp[1], p[2] - fp[2]]),
    quat: quatMul(quatConjugate(fq), q)
  };
}
function poseFromFrame(fp, fq, rp, rq) {
  const o = quatApply(fq, rp);
  return {
    pos: [fp[0] + o[0], fp[1] + o[1], fp[2] + o[2]],
    quat: quatMul(fq, rq)
  };
}
function quatAngle(a, b) {
  const r = quatMul(quatConjugate(a), b);
  return 2 * Math.atan2(
    Math.sqrt(r[1] * r[1] + r[2] * r[2] + r[3] * r[3]),
    Math.abs(r[0])
  );
}

// ../../../../../../web/plugins/student_io.ts
var OBS_DIM = 201;
var ACT_DIM = 29;
var FRAME_DIM = 67;
var MEAS_DIM = 38;
var NUM_FINGERS = 20;
var FRAME_BLOCKS = [
  [0, 3],
  [3, 9],
  [9, 29],
  [29, 38],
  [38, 67]
];
var EXACT = { pos: 0, rot: 0, joint: 0, lag: 0 };
var EVAL_SENSE = {
  pos: 2e-3,
  rot: 0.02,
  joint: 0.02,
  lag: 2
};
function measuredTarget(s) {
  return [...s.wristPos, ...rot6d(s.wristQuat), ...s.jointPos];
}
var StudentView = class {
  noise;
  rng;
  frames = [
    new Float64Array(FRAME_DIM),
    new Float64Array(FRAME_DIM),
    new Float64Array(FRAME_DIM)
  ];
  started = false;
  meas;
  lag = 0;
  constructor(noise, rng) {
    this.noise = noise;
    this.rng = rng;
    this.meas = Array.from(
      { length: noise.lag + 1 },
      () => new Array(MEAS_DIM).fill(0)
    );
  }
  /** The latency drawn for this episode (control steps). */
  get latency() {
    return this.lag;
  }
  reset() {
    this.started = false;
  }
  jitter(x, s) {
    if (!s) return x;
    const rng = this.rng;
    if (!rng) throw new Error("[mjweb] sensing noise needs an RNG");
    return x.map((v) => v + (rng.random() * 2 - 1) * s);
  }
  /** One 67-D frame: noisy (and possibly delayed) 38-D measurement + the exact previous target. */
  measure(s, prevTarget) {
    const nz = this.noise;
    const wp = this.jitter(s.wristPos, nz.pos);
    const op = this.jitter(s.objectPos, nz.pos);
    const q = this.jitter(s.jointPos, nz.joint);
    const objPos = quatApplyInverse(s.wristQuat, [
      op[0] - wp[0],
      op[1] - wp[1],
      op[2] - wp[2]
    ]);
    const objRot = this.jitter(
      rot6d(quatMul(quatInv(s.wristQuat), s.objectQuat)),
      nz.rot
    );
    let m = [
      ...wp,
      ...this.jitter(rot6d(s.wristQuat), nz.rot),
      ...q,
      ...objPos,
      ...objRot
    ];
    if (nz.lag > 0) {
      this.meas.unshift(this.meas.pop());
      this.meas[0] = m;
      if (!this.started) {
        for (let i = 0; i < this.meas.length; i++) this.meas[i] = m.slice();
        const rng = this.rng;
        if (!rng) throw new Error("[mjweb] sensing latency needs an RNG");
        this.lag = rng.integer(0, nz.lag);
      }
      m = this.meas[this.lag];
    }
    return [...m, ...Array.from(prevTarget)];
  }
  push(frame) {
    this.frames[0].set(this.frames[1]);
    this.frames[1].set(this.frames[2]);
    this.frames[2].set(frame);
    if (!this.started) {
      this.frames[0].set(frame);
      this.frames[1].set(frame);
    }
    this.started = true;
  }
  observation() {
    const out = new Float32Array(OBS_DIM);
    let k = 0;
    for (const [a, b] of FRAME_BLOCKS) {
      for (let f = 0; f < 3; f++) {
        const fr = this.frames[f];
        for (let i = a; i < b; i++) out[k++] = fr[i];
      }
    }
    return out;
  }
};
var TargetFilter = class {
  lower;
  upper;
  alphaWrist;
  alphaFingers;
  stepDt;
  maxFfLin;
  maxFfAng;
  pos = [0, 0, 0];
  quat = [1, 0, 0, 0];
  joints = new Array(NUM_FINGERS).fill(0);
  ffLin = [0, 0, 0];
  ffAng = [0, 0, 0];
  initialized = false;
  constructor(lower, upper, alphaWrist, alphaFingers, stepDt = 1 / 30, maxFfLin = 1, maxFfAng = 6) {
    this.lower = Float64Array.from(lower);
    this.upper = Float64Array.from(upper);
    this.alphaWrist = alphaWrist;
    this.alphaFingers = alphaFingers;
    this.stepDt = stepDt;
    this.maxFfLin = maxFfLin;
    this.maxFfAng = maxFfAng;
  }
  reset() {
    this.initialized = false;
    this.ffLin = [0, 0, 0];
    this.ffAng = [0, 0, 0];
  }
  /** Consume one 29-D student output (absolute target). */
  process(y) {
    const newP = [y[0], y[1], y[2]];
    const newQ = rot6dToQuat([y[3], y[4], y[5], y[6], y[7], y[8]]);
    const newJ = new Array(NUM_FINGERS);
    for (let i = 0; i < NUM_FINGERS; i++)
      newJ[i] = Math.min(Math.max(y[9 + i], this.lower[i]), this.upper[i]);
    if (!this.initialized) {
      this.pos = newP;
      this.quat = newQ;
      this.joints = newJ;
      this.ffLin = [0, 0, 0];
      this.ffAng = [0, 0, 0];
      this.initialized = true;
      return;
    }
    const prevP = this.pos;
    const prevQ = this.quat;
    const aw = this.alphaWrist;
    const af = this.alphaFingers;
    const pos = aw < 1 ? [0, 1, 2].map((i) => prevP[i] + aw * (newP[i] - prevP[i])) : newP;
    const quat = aw < 1 ? slerpToward(prevQ, newQ, aw) : newQ;
    this.joints = af < 1 ? this.joints.map((j, i) => j + af * (newJ[i] - j)) : newJ;
    this.ffLin = clipNorm(
      [0, 1, 2].map((i) => (pos[i] - prevP[i]) / this.stepDt),
      this.maxFfLin
    );
    const dq = quatPositiveReal(quatMul(quat, quatConjugate(prevQ)));
    const aa = axisAngleFromQuat(dq);
    this.ffAng = clipNorm(
      [aa[0] / this.stepDt, aa[1] / this.stepDt, aa[2] / this.stepDt],
      this.maxFfAng
    );
    this.pos = pos;
    this.quat = quat;
  }
  /** `single_teacher.student.applied_target`: [pos, rot6d(quat), joints] (29). */
  applied() {
    return [...this.pos, ...rot6d(this.quat), ...this.joints];
  }
};

// ../../../../../../web/plugins/StudentObs.ts
var StudentObs = class extends ObservationBase {
  zeros = new Float32Array(OBS_DIM);
  constructor(runner, config) {
    super(runner, config);
  }
  get size() {
    return OBS_DIM;
  }
  compute(_state) {
    return session.controller?.observe() ?? this.zeros;
  }
};

// ../../../../../../web/plugins/drive.ts
var DenseSolver = class {
  m;
  rhs;
  constructor(maxN) {
    this.m = new Float64Array(maxN * maxN);
    this.rhs = new Float64Array(maxN);
  }
  /**
   * Solve A[idx, idx] x = rhs for the `count` indices in `idx` (A is n x n row-major); writes
   * x into `out[0..count)`. Returns false for a numerically singular system.
   */
  solve(a, n, idx, count, rhs, out) {
    const m = this.m;
    const r = this.rhs;
    for (let i = 0; i < count; i++) {
      const row = idx[i] * n;
      for (let j = 0; j < count; j++) m[i * count + j] = a[row + idx[j]];
      r[i] = rhs[i];
    }
    for (let k = 0; k < count; k++) {
      let p = k;
      let best = Math.abs(m[k * count + k]);
      for (let i = k + 1; i < count; i++) {
        const v = Math.abs(m[i * count + k]);
        if (v > best) {
          best = v;
          p = i;
        }
      }
      if (best === 0) return false;
      if (p !== k) {
        for (let j = 0; j < count; j++) {
          const t2 = m[k * count + j];
          m[k * count + j] = m[p * count + j];
          m[p * count + j] = t2;
        }
        const t = r[k];
        r[k] = r[p];
        r[p] = t;
      }
      const piv = m[k * count + k];
      for (let i = k + 1; i < count; i++) {
        const f = m[i * count + k] / piv;
        if (f === 0) continue;
        m[i * count + k] = f;
        for (let j = k + 1; j < count; j++)
          m[i * count + j] -= f * m[k * count + j];
        r[i] -= f * r[k];
      }
    }
    for (let i = count - 1; i >= 0; i--) {
      let s = r[i];
      for (let j = i + 1; j < count; j++) s -= m[i * count + j] * out[j];
      out[i] = s / m[i * count + i];
    }
    return true;
  }
};
var FingerDrive = class {
  n;
  solver;
  a;
  b;
  c;
  x;
  cand;
  d;
  g;
  fixed;
  freeIdx;
  subRhs;
  subX;
  /** Iterations of the last solve (diagnostics). */
  lastIters = 0;
  constructor(n = 20) {
    this.n = n;
    this.solver = new DenseSolver(n);
    this.a = new Float64Array(n * n);
    this.b = new Float64Array(n);
    this.c = new Float64Array(n);
    this.x = new Float64Array(n);
    this.cand = new Float64Array(n);
    this.d = new Float64Array(n);
    this.g = new Float64Array(n);
    this.fixed = new Uint8Array(n);
    this.freeIdx = new Int32Array(n);
    this.subRhs = new Float64Array(n);
    this.subX = new Float64Array(n);
  }
  /**
   * `coupled_implicit_pd`: `sol` holds the 21 rows of mj_solveM (row 0 = M^-1 f_other, row 1+j =
   * M^-1 e_{finger j}), each `nv` long. Writes the drive torques into `tau` and returns them.
   */
  coupled(sol, nv, vFree, fingerDof, e, kp, kd, h, effort, warm, tau) {
    const n = this.n;
    const a = this.a;
    const c = this.c;
    for (let i = 0; i < n; i++) c[i] = h * kp[i] + kd[i];
    for (let i = 0; i < n; i++) {
      const di = fingerDof[i];
      for (let j = 0; j < n; j++) {
        a[i * n + j] = (i === j ? 1 / c[i] : 0) + h * sol[(1 + j) * nv + di];
      }
    }
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        const s = 0.5 * (a[i * n + j] + a[j * n + i]);
        a[i * n + j] = s;
        a[j * n + i] = s;
      }
      a[i * n + i] = 0.5 * (a[i * n + i] + a[i * n + i]);
    }
    const b = this.b;
    for (let i = 0; i < n; i++)
      b[i] = kp[i] * e[i] / c[i] - vFree[fingerDof[i]];
    return this.boxQp(effort, warm, tau);
  }
  /** `box_qp` on the assembled (a, b): min 0.5 x^T A x - b^T x, |x_i| <= bound. */
  boxQp(bound, warm, out, iters = 100) {
    const n = this.n;
    const a = this.a;
    const b = this.b;
    const x = this.x;
    const fixed = this.fixed;
    let useWarm = false;
    if (warm) {
      for (let i = 0; i < n; i++) if (warm[i]) useWarm = true;
    }
    if (useWarm && warm) {
      for (let i = 0; i < n; i++) {
        fixed[i] = warm[i] ? 1 : 0;
        x[i] = 0;
      }
      for (let i = 0; i < n; i++) if (fixed[i]) x[i] = Math.sign(b[i]) * bound;
      this.solveFree(x, x);
      let maxFree = 0;
      for (let i = 0; i < n; i++)
        if (!fixed[i]) maxFree = Math.max(maxFree, Math.abs(x[i]));
      if (maxFree > bound) useWarm = false;
    }
    if (!useWarm) {
      for (let i = 0; i < n; i++) fixed[i] = 0;
      this.solveFree(x, x);
      for (let i = 0; i < n; i++) {
        x[i] = Math.min(Math.max(x[i], -bound), bound);
        fixed[i] = Math.abs(x[i]) >= bound ? 1 : 0;
      }
    }
    const cand = this.cand;
    const d = this.d;
    const g = this.g;
    let it = 0;
    for (; it < iters; it++) {
      cand.set(x);
      this.solveFree(x, cand);
      let anyViol = false;
      for (let i = 0; i < n; i++)
        if (!fixed[i] && Math.abs(cand[i]) > bound) anyViol = true;
      if (anyViol) {
        let best2 = Infinity;
        let bi = 0;
        for (let i = 0; i < n; i++) {
          d[i] = cand[i] - x[i];
          const viol = !fixed[i] && Math.abs(cand[i]) > bound;
          const t = viol ? (Math.sign(d[i]) * bound - x[i]) / d[i] : Infinity;
          if (t < best2) {
            best2 = t;
            bi = i;
          }
        }
        const step = Math.min(Math.max(best2, 0), 1);
        for (let i = 0; i < n; i++)
          x[i] = Math.min(Math.max(x[i] + step * d[i], -bound), bound);
        x[bi] = Math.sign(d[bi]) * bound;
        fixed[bi] = 1;
        continue;
      }
      x.set(cand);
      for (let i = 0; i < n; i++) {
        let s = 0;
        for (let j = 0; j < n; j++) s += a[i * n + j] * x[j];
        g[i] = s - b[i];
      }
      let best = Infinity;
      let bj = 0;
      for (let i = 0; i < n; i++) {
        const mult = fixed[i] ? x[i] > 0 ? -g[i] : g[i] : Infinity;
        if (mult < best) {
          best = mult;
          bj = i;
        }
      }
      if (best >= -1e-10) break;
      fixed[bj] = 0;
    }
    this.lastIters = it;
    out.set(x);
    return out;
  }
  /**
   * x_free = A[free, free]^-1 (b[free] - A[free, fixed] x[fixed]), written into `dst` (the fixed
   * entries of `dst` are left alone). `src` supplies the fixed values.
   */
  solveFree(src, dst) {
    const n = this.n;
    const a = this.a;
    const fixed = this.fixed;
    const idx = this.freeIdx;
    let count = 0;
    for (let i = 0; i < n; i++) if (!fixed[i]) idx[count++] = i;
    if (count === 0) return;
    const rhs = this.subRhs;
    for (let k = 0; k < count; k++) {
      const i = idx[k];
      let s = 0;
      for (let j = 0; j < n; j++) if (fixed[j]) s += a[i * n + j] * src[j];
      rhs[k] = this.b[i] - s;
    }
    if (!this.solver.solve(a, n, idx, count, rhs, this.subX))
      throw new Error("[mjweb] singular finger-drive system");
    for (let k = 0; k < count; k++) dst[idx[k]] = this.subX[k];
  }
};

// ../../../../../../web/plugins/hand_env.ts
var ISAAC_DT = 1 / 120;
var CONTROL_DT = 1 / 30;
var WRIST_BODY = "r_wrist";
var WRIST_JOINT = "r_wrist_free";
var OBJECT_BODY = "object";
var OBJECT_JOINT = "object_free";
var TIP_LINKS = [
  "r_thumb_tip_link",
  "r_index_finger_tip_link",
  "r_middle_finger_tip_link",
  "r_ring_finger_tip_link",
  "r_pinky_tip_link"
];
var BADQACC = 5;
function id(mujoco, model, type, name) {
  const t = type === "body" ? mujoco.mjtObj.mjOBJ_BODY : type === "joint" ? mujoco.mjtObj.mjOBJ_JOINT : mujoco.mjtObj.mjOBJ_ACTUATOR;
  const i = mujoco.mj_name2id(model, t.value, name);
  if (i < 0) throw new Error(`[mjweb] ${type} "${name}" not in the model`);
  return i;
}
function layoutOf(mujoco, model, jointOrder) {
  const wj = id(mujoco, model, "joint", WRIST_JOINT);
  const oj = id(mujoco, model, "joint", OBJECT_JOINT);
  if (wj !== 0) throw new Error("[mjweb] the wrist free joint must be joint 0");
  if (jointOrder.length !== NUM_FINGERS)
    throw new Error("[mjweb] expected 20 finger joints");
  const qadr = model.jnt_qposadr;
  const vadr = model.jnt_dofadr;
  const range = model.jnt_range;
  const jbody = model.jnt_bodyid;
  const finger = jointOrder.map((n) => id(mujoco, model, "joint", n));
  jointOrder.forEach((n, i) => {
    const act = id(mujoco, model, "actuator", `act_${n}`);
    if (act !== i)
      throw new Error(`[mjweb] actuator act_${n} has id ${act}, expected ${i}`);
  });
  const fingerRange = new Float64Array(2 * NUM_FINGERS);
  finger.forEach((j, i) => {
    fingerRange[2 * i] = range[2 * j];
    fingerRange[2 * i + 1] = range[2 * j + 1];
  });
  const wristBody = id(mujoco, model, "body", WRIST_BODY);
  const tips = TIP_LINKS.map((n) => id(mujoco, model, "body", n));
  return {
    wristBody,
    objectBody: id(mujoco, model, "body", OBJECT_BODY),
    wristQpos: qadr[wj],
    wristQvel: vadr[wj],
    objectQpos: qadr[oj],
    objectQvel: vadr[oj],
    fingerQpos: Int32Array.from(finger.map((j) => qadr[j])),
    fingerQvel: Int32Array.from(finger.map((j) => vadr[j])),
    fingerRange,
    handPointBodies: Int32Array.from([
      wristBody,
      ...finger.map((j) => jbody[j]),
      ...tips
    ])
  };
}
function badqaccCount(warnings) {
  const stat = warnings.get(BADQACC);
  if (!stat) return 0;
  const n = stat.number;
  stat.delete();
  return n;
}
function luFactor(a, n, piv) {
  for (let k = 0; k < n; k++) {
    let p = k;
    let best = Math.abs(a[k * n + k]);
    for (let i = k + 1; i < n; i++) {
      const v = Math.abs(a[i * n + k]);
      if (v > best) {
        best = v;
        p = i;
      }
    }
    piv[k] = p;
    if (p !== k) {
      for (let j = 0; j < n; j++) {
        const t = a[k * n + j];
        a[k * n + j] = a[p * n + j];
        a[p * n + j] = t;
      }
    }
    const d = a[k * n + k];
    for (let i = k + 1; i < n; i++) {
      const f = a[i * n + k] / d;
      a[i * n + k] = f;
      if (f === 0) continue;
      for (let j = k + 1; j < n; j++) a[i * n + j] -= f * a[k * n + j];
    }
  }
}
function luSolve(lu, n, piv, b) {
  for (let k = 0; k < n; k++) {
    const p = piv[k];
    if (p !== k) {
      const t = b[k];
      b[k] = b[p];
      b[p] = t;
    }
  }
  for (let i = 1; i < n; i++) {
    let s = b[i];
    for (let j = 0; j < i; j++) s -= lu[i * n + j] * b[j];
    b[i] = s;
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = b[i];
    for (let j = i + 1; j < n; j++) s -= lu[i * n + j] * b[j];
    b[i] = s / lu[i * n + i];
  }
}
var HandEnv = class {
  hand = "beta1";
  mujoco;
  model;
  data;
  L;
  filter;
  wristMode;
  substeps;
  decimation;
  h;
  nv;
  kp;
  kd;
  vmax;
  effort;
  gains;
  drive = new FingerDrive(NUM_FINGERS);
  sat = new Uint8Array(NUM_FINGERS);
  tau = new Float64Array(NUM_FINGERS);
  e = new Float64Array(NUM_FINGERS);
  vFree;
  // explicit modes: [f_other; S^T] (21 rows) and its solve
  rhs;
  sol;
  solBuf;
  velBuf;
  // implicit mode: [f_other; S^T; J] (27 rows), J, Woodbury factor, coupling rows
  rhs27;
  sol27;
  sol27Buf;
  jacpBuf;
  jacrBuf;
  jac;
  kw = new Float64Array(6);
  dw = new Float64Array(6);
  cw = new Float64Array(6);
  kmat = new Float64Array(36);
  kpiv = new Int32Array(6);
  /** M~^-1 S in the row layout `FingerDrive.coupled` reads: row 1+j = column j (nv each). */
  btS;
  bR;
  vNew;
  col6 = new Float64Array(6);
  eW = new Float64Array(6);
  ffW = new Float64Array(6);
  wExpl = new Float64Array(6);
  // wrist pose (and, explicit, COM velocity), snapshotted after the mj_step the next wrench reads
  wPos = new Float64Array(3);
  wQuat = new Float64Array(4);
  wVel = new Float64Array(3);
  wOmega = new Float64Array(3);
  wrench = new Float64Array(6);
  /** What the wrist's `xfrc_applied` row held before this control step (e.g. a mouse pull). */
  base = new Float64Array(6);
  wrenchOn = false;
  warnings;
  badqacc0 = 0;
  stepQpos;
  stepQvel;
  /** MuJoCo blew up (mjWARN_BADQACC) during the last control step (`MjHandEnv.unstable`). */
  unstable = false;
  /** The measured state and hand points the harness's `state()` / `hand_points()` return. */
  state;
  handPoints = new Float64Array(26 * 3);
  /** Box-QP iterations summed over the last control step (diagnostics). */
  qpIters = 0;
  constructor(mujoco, model, data, layout, ctl, alphaWrist, alphaFingers, wristMode = "isaac") {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    this.L = layout;
    this.wristMode = wristMode;
    this.nv = model.nv;
    this.h = model.opt.timestep;
    this.substeps = Math.round(ISAAC_DT / this.h);
    if (Math.abs(this.substeps * this.h - ISAAC_DT) > 1e-6)
      throw new Error(`[mjweb] timestep ${this.h} does not divide 1/120 s`);
    this.decimation = 4 * this.substeps;
    if (layout.wristQvel !== 0)
      throw new Error("[mjweb] expected the wrist DOFs first");
    const lo = new Float64Array(NUM_FINGERS);
    const hi = new Float64Array(NUM_FINGERS);
    for (let i = 0; i < NUM_FINGERS; i++) {
      lo[i] = layout.fingerRange[2 * i];
      hi[i] = layout.fingerRange[2 * i + 1];
    }
    this.filter = new TargetFilter(
      lo,
      hi,
      alphaWrist,
      alphaFingers,
      ctl.control_dt,
      ctl.max_ff_lin,
      ctl.max_ff_ang
    );
    if (ctl.finger_kp === void 0 || ctl.finger_kd === void 0 || ctl.finger_effort === void 0) {
      throw new Error(
        "[mjweb] the beta1 drive needs finger_kp / finger_kd / finger_effort in the web data"
      );
    }
    this.kp = new Float64Array(NUM_FINGERS).fill(ctl.finger_kp);
    this.kd = new Float64Array(NUM_FINGERS).fill(ctl.finger_kd);
    this.vmax = Float64Array.from(ctl.finger_vel_limits);
    this.effort = ctl.finger_effort;
    const g = ctl.wrist_gains;
    this.gains = [g[0], g[1], g[2], g[3]];
    for (let i = 0; i < 3; i++) {
      this.kw[i] = g[0];
      this.kw[3 + i] = g[2];
      this.dw[i] = g[1];
      this.dw[3 + i] = g[3];
    }
    for (let i = 0; i < 6; i++) this.cw[i] = this.h * this.kw[i] + this.dw[i];
    const nv = this.nv;
    this.rhs = new Float64Array(21 * nv);
    this.rhs27 = new Float64Array(27 * nv);
    for (let j = 0; j < NUM_FINGERS; j++) {
      this.rhs[(1 + j) * nv + layout.fingerQvel[j]] = 1;
      this.rhs27[(1 + j) * nv + layout.fingerQvel[j]] = 1;
    }
    this.sol = new Float64Array(21 * nv);
    this.sol27 = new Float64Array(27 * nv);
    this.btS = new Float64Array(21 * nv);
    this.jac = new Float64Array(6 * nv);
    this.vFree = new Float64Array(nv);
    this.bR = new Float64Array(nv);
    this.vNew = new Float64Array(nv);
    this.stepQpos = new Float64Array(model.nq);
    this.stepQvel = new Float64Array(nv);
    const M = mujoco;
    this.solBuf = new M.DoubleBuffer(21 * nv);
    this.velBuf = new M.DoubleBuffer(6);
    this.sol27Buf = new M.DoubleBuffer(27 * nv);
    this.jacpBuf = new M.DoubleBuffer(3 * nv);
    this.jacrBuf = new M.DoubleBuffer(3 * nv);
    this.warnings = data.warning;
    this.state = this.readState();
  }
  dispose() {
    for (const b of [
      this.solBuf,
      this.velBuf,
      this.sol27Buf,
      this.jacpBuf,
      this.jacrBuf
    ])
      b.delete();
  }
  /** mjWARN_BADQACC count (MuJoCo's automatic resets after a blow-up). */
  badqacc() {
    return badqaccCount(this.warnings);
  }
  // ------------------------------------------------------------------ reset
  /** `MjHandEnv.reset` from the exact MuJoCo state (`webdata.json` starts are env.reset's qpos/qvel). */
  reset(qpos, qvel) {
    const { mujoco, model, data } = this;
    if (qpos.length !== model.nq || qvel.length !== model.nv)
      throw new Error("[mjweb] start state has the wrong size");
    mujoco.mj_resetData(model, data);
    data.qpos.set(qpos);
    data.qvel.set(qvel);
    mujoco.mj_forward(model, data);
    this.filter.reset();
    this.sat.fill(0);
    this.base.fill(0);
    this.wrenchOn = false;
    this.unstable = false;
    this.badqacc0 = this.badqacc();
    this.snapshot(-1);
    this.state = this.readState();
    this.readHandPoints();
    return this.state;
  }
  // ------------------------------------------------------------------ one control step
  beginStep(y) {
    this.filter.process(y);
    this.qpIters = 0;
    this.stepQpos.set(this.data.qpos);
    this.stepQvel.set(this.data.qvel);
  }
  /** Before `mj_step` of substep `s` (0 .. decimation-1): drives -> ctrl and the wrist wrench. */
  preSubstep(s) {
    const data = this.data;
    const xfrc = data.xfrc_applied;
    const row = 6 * this.L.wristBody;
    if (s === 0) for (let i = 0; i < 6; i++) this.base[i] = xfrc[row + i];
    let tau;
    if (this.wristMode === "implicit") {
      tau = this.implicitDrives();
      for (let i = 0; i < 6; i++) xfrc[row + i] = this.base[i] + this.wrench[i];
      this.wrenchOn = true;
    } else {
      if (this.wristMode === "substep" || s % this.substeps === 0) {
        this.explicitWrench();
        for (let i = 0; i < 6; i++)
          xfrc[row + i] = this.base[i] + this.wrench[i];
        this.wrenchOn = true;
      }
      tau = this.fingerTorque();
    }
    const ctrl = data.ctrl;
    for (let i = 0; i < NUM_FINGERS; i++) ctrl[i] = tau[i];
  }
  /** After `mj_step` of substep `s`: speed clamp and the snapshot the next substep reads. */
  postSubstep(s) {
    this.clampFingerSpeed();
    this.snapshot(s);
    if (s === this.decimation - 1) {
      this.removeWrench();
      if (this.badqacc() > this.badqacc0) {
        this.unstable = true;
        this.data.qpos.set(this.stepQpos);
        this.data.qvel.set(this.stepQvel);
      }
      this.state = this.readState();
      this.readHandPoints();
    }
  }
  /** The whole control step with MuJoCo's own `mj_step` (Node / tests; mjswan steps in the browser). */
  step(y) {
    this.beginStep(y);
    for (let s = 0; s < this.decimation; s++) {
      this.preSubstep(s);
      this.mujoco.mj_step(this.model, this.data);
      this.postSubstep(s);
    }
    return this.state;
  }
  /** Take the wrench back off the wrist row (leaving what another writer put there). */
  removeWrench() {
    if (!this.wrenchOn) return;
    const xfrc = this.data.xfrc_applied;
    const row = 6 * this.L.wristBody;
    for (let i = 0; i < 6; i++) xfrc[row + i] = this.base[i];
    this.wrenchOn = false;
  }
  // ------------------------------------------------------------------ snapshots (after mj_step / reset)
  /** Everything the next substep reads from MuJoCo's (one-substep-stale) derived quantities. */
  snapshot(s) {
    const data = this.data;
    const nv = this.nv;
    if (this.wristMode === "implicit") {
      this.snapshotImplicit();
      this.snapshotWristPose();
      return;
    }
    const smooth = data.qfrc_smooth;
    const act = data.qfrc_actuator;
    const con = data.qfrc_constraint;
    for (let k = 0; k < nv; k++) this.rhs[k] = smooth[k] - act[k] + con[k];
    this.mujoco.mj_solveM(
      this.model,
      data,
      this.solBuf,
      this.rhs
    );
    this.sol.set(this.solBuf.GetView());
    if (s < 0 || this.wristMode === "substep" || (s + 1) % this.substeps === 0) {
      this.snapshotWristPose();
      this.mujoco.mj_objectVelocity(
        this.model,
        data,
        this.mujoco.mjtObj.mjOBJ_BODY.value,
        this.L.wristBody,
        this.velBuf,
        0
      );
      const res = this.velBuf.GetView();
      for (let i = 0; i < 3; i++) {
        this.wOmega[i] = res[i];
        this.wVel[i] = res[3 + i];
      }
    }
  }
  snapshotWristPose() {
    const b = this.L.wristBody;
    const xpos = this.data.xpos;
    const xquat = this.data.xquat;
    for (let i = 0; i < 3; i++) this.wPos[i] = xpos[3 * b + i];
    for (let i = 0; i < 4; i++) this.wQuat[i] = xquat[4 * b + i];
  }
  /**
   * The stale part of `implicit_drives`: f_other, J, the 27 `mj_solveM` rows, the Woodbury factor
   * kmat = diag(1 / (h c_w)) + J M^-1 J^T and M~^-1 S = M^-1 S - M^-1 J^T kmat^-1 J M^-1 S.
   */
  snapshotImplicit() {
    const { mujoco, model, data } = this;
    const nv = this.nv;
    const rhs = this.rhs27;
    const passive = data.qfrc_passive;
    const bias = data.qfrc_bias;
    const applied = data.qfrc_applied;
    const con = data.qfrc_constraint;
    for (let k = 0; k < nv; k++)
      rhs[k] = passive[k] - bias[k] + applied[k] + con[k];
    mujoco.mj_jacBodyCom(
      model,
      data,
      this.jacpBuf,
      this.jacrBuf,
      this.L.wristBody
    );
    const jp = this.jacpBuf.GetView();
    const jr = this.jacrBuf.GetView();
    const jac = this.jac;
    jac.set(jp.subarray(0, 3 * nv), 0);
    jac.set(jr.subarray(0, 3 * nv), 3 * nv);
    rhs.set(jac, 21 * nv);
    mujoco.mj_solveM(model, data, this.sol27Buf, rhs);
    const sol = this.sol27;
    sol.set(this.sol27Buf.GetView());
    const km = this.kmat;
    for (let i = 0; i < 6; i++) {
      for (let k = 0; k < 6; k++) {
        let acc = 0;
        const jrow = i * nv;
        const brow = (21 + k) * nv;
        for (let r = 0; r < nv; r++) acc += jac[jrow + r] * sol[brow + r];
        km[i * 6 + k] = (i === k ? 1 / (this.h * this.cw[i]) : 0) + acc;
      }
    }
    luFactor(km, 6, this.kpiv);
    const y = this.col6;
    const bt = this.btS;
    for (let j = 0; j < NUM_FINGERS; j++) {
      const srow = (1 + j) * nv;
      for (let i = 0; i < 6; i++) {
        let acc = 0;
        for (let r = 0; r < nv; r++) acc += jac[i * nv + r] * sol[srow + r];
        y[i] = acc;
      }
      luSolve(km, 6, this.kpiv, y);
      for (let r = 0; r < nv; r++) {
        let acc = 0;
        for (let k = 0; k < 6; k++) acc += sol[(21 + k) * nv + r] * y[k];
        bt[srow + r] = sol[srow + r] - acc;
      }
    }
  }
  // ------------------------------------------------------------------ drives
  /** `MjHandEnv.implicit_drives`: finger torques (returned) and the wrist wrench (this.wrench). */
  implicitDrives() {
    const data = this.data;
    const L = this.L;
    const nv = this.nv;
    const h = this.h;
    const f = this.filter;
    const qvel = data.qvel;
    const qpos = data.qpos;
    const jac = this.jac;
    const sol = this.sol27;
    const eW = this.eW;
    const ff = this.ffW;
    for (let i = 0; i < 3; i++) eW[i] = f.pos[i] - this.wPos[i];
    const aa = axisAngleFromQuat(
      quatPositiveReal(quatMul(f.quat, quatConjugate(this.wQuat)))
    );
    for (let i = 0; i < 3; i++) {
      eW[3 + i] = aa[i];
      ff[i] = f.ffLin[i];
      ff[3 + i] = f.ffAng[i];
    }
    const w = this.wExpl;
    for (let i = 0; i < 6; i++) {
      let jv = 0;
      for (let r = 0; r < nv; r++) jv += jac[i * nv + r] * qvel[r];
      w[i] = this.kw[i] * eW[i] + this.dw[i] * ff[i] - this.cw[i] * jv;
    }
    const bR = this.bR;
    for (let r = 0; r < nv; r++) {
      let acc = 0;
      for (let k = 0; k < 6; k++) acc += sol[(21 + k) * nv + r] * w[k];
      bR[r] = sol[r] + acc;
    }
    const z = this.col6;
    for (let i = 0; i < 6; i++) {
      let acc = 0;
      for (let r = 0; r < nv; r++) acc += jac[i * nv + r] * bR[r];
      z[i] = acc;
    }
    luSolve(this.kmat, 6, this.kpiv, z);
    for (let r = 0; r < nv; r++) {
      let acc = 0;
      for (let k = 0; k < 6; k++) acc += sol[(21 + k) * nv + r] * z[k];
      this.vFree[r] = qvel[r] + h * (bR[r] - acc);
    }
    const joints = f.joints;
    for (let i = 0; i < NUM_FINGERS; i++)
      this.e[i] = joints[i] - qpos[L.fingerQpos[i]];
    const tau = this.drive.coupled(
      this.btS,
      nv,
      this.vFree,
      L.fingerQvel,
      this.e,
      this.kp,
      this.kd,
      h,
      this.effort,
      this.sat,
      this.tau
    );
    this.qpIters += this.drive.lastIters;
    const lim = this.effort - 1e-12;
    for (let i = 0; i < NUM_FINGERS; i++)
      this.sat[i] = Math.abs(tau[i]) >= lim ? 1 : 0;
    const vn = this.vNew;
    const bt = this.btS;
    for (let r = 0; r < nv; r++) {
      let acc = 0;
      for (let j = 0; j < NUM_FINGERS; j++)
        acc += bt[(1 + j) * nv + r] * tau[j];
      vn[r] = this.vFree[r] + h * acc;
    }
    for (let i = 0; i < 6; i++) {
      let jv = 0;
      for (let r = 0; r < nv; r++) jv += jac[i * nv + r] * vn[r];
      this.wrench[i] = this.kw[i] * eW[i] + this.dw[i] * ff[i] - this.cw[i] * jv;
    }
    return tau;
  }
  /** `_wrist_wrench`: the explicit PD wrench from the snapshotted wrist state. */
  explicitWrench() {
    const f = this.filter;
    const [kpP, kdP, kpR, kdR] = this.gains;
    const w = this.wrench;
    for (let i = 0; i < 3; i++)
      w[i] = kpP * (f.pos[i] - this.wPos[i]) + kdP * (f.ffLin[i] - this.wVel[i]);
    const qErr = quatPositiveReal(quatMul(f.quat, quatConjugate(this.wQuat)));
    const aa = axisAngleFromQuat(qErr);
    for (let i = 0; i < 3; i++)
      w[3 + i] = kpR * aa[i] + kdR * (f.ffAng[i] - this.wOmega[i]);
  }
  /** `finger_torque` (explicit wrist modes): coupled implicit finger drive alone. */
  fingerTorque() {
    const data = this.data;
    const L = this.L;
    const qpos = data.qpos;
    const qvel = data.qvel;
    const nv = this.nv;
    const h = this.h;
    const joints = this.filter.joints;
    for (let i = 0; i < NUM_FINGERS; i++)
      this.e[i] = joints[i] - qpos[L.fingerQpos[i]];
    for (let k = 0; k < nv; k++) this.vFree[k] = qvel[k] + h * this.sol[k];
    this.drive.coupled(
      this.sol,
      nv,
      this.vFree,
      L.fingerQvel,
      this.e,
      this.kp,
      this.kd,
      h,
      this.effort,
      this.sat,
      this.tau
    );
    this.qpIters += this.drive.lastIters;
    const lim = this.effort - 1e-12;
    for (let i = 0; i < NUM_FINGERS; i++)
      this.sat[i] = Math.abs(this.tau[i]) >= lim ? 1 : 0;
    return this.tau;
  }
  clampFingerSpeed() {
    const qvel = this.data.qvel;
    const dof = this.L.fingerQvel;
    for (let i = 0; i < NUM_FINGERS; i++) {
      const v = qvel[dof[i]];
      const m = this.vmax[i];
      qvel[dof[i]] = v < -m ? -m : v > m ? m : v;
    }
  }
  readState() {
    const d = this.data;
    const L = this.L;
    const xpos = d.xpos;
    const xquat = d.xquat;
    const qpos = d.qpos;
    const w = L.wristBody;
    const o = L.objectBody;
    return {
      wristPos: [xpos[3 * w], xpos[3 * w + 1], xpos[3 * w + 2]],
      wristQuat: [
        xquat[4 * w],
        xquat[4 * w + 1],
        xquat[4 * w + 2],
        xquat[4 * w + 3]
      ],
      jointPos: Array.from(L.fingerQpos, (a) => qpos[a]),
      objectPos: [xpos[3 * o], xpos[3 * o + 1], xpos[3 * o + 2]],
      objectQuat: [
        xquat[4 * o],
        xquat[4 * o + 1],
        xquat[4 * o + 2],
        xquat[4 * o + 3]
      ]
    };
  }
  readHandPoints() {
    const xpos = this.data.xpos;
    const b = this.L.handPointBodies;
    for (let i = 0; i < b.length; i++) {
      for (let k = 0; k < 3; k++)
        this.handPoints[3 * i + k] = xpos[3 * b[i] + k];
    }
  }
};

// ../../../../../../web/plugins/hand_env21.ts
function handBodyIds(model, wristBody) {
  const parent = model.body_parentid;
  const out = [wristBody];
  for (let b = wristBody + 1; b < model.nbody; b++) {
    let p = b;
    while (p > 0 && p !== wristBody) p = parent[p];
    if (p === wristBody) out.push(b);
  }
  return Int32Array.from(out);
}
function pairwiseSum(a) {
  const n = a.length;
  if (n < 8) {
    let s = 0;
    for (let i2 = 0; i2 < n; i2++) s += a[i2];
    return s;
  }
  if (n > 128)
    throw new Error("[mjweb] pairwiseSum: only the unblocked case is ported");
  const r = [a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]];
  let i = 8;
  for (; i < n - n % 8; i += 8) {
    for (let j = 0; j < 8; j++) r[j] += a[i + j];
  }
  let res = r[0] + r[1] + (r[2] + r[3]) + (r[4] + r[5] + (r[6] + r[7]));
  for (; i < n; i++) res += a[i];
  return res;
}
function solve3(aIn, bIn) {
  const a = Array.from(aIn);
  const b = Array.from(bIn);
  const at = (i, j) => a[3 * i + j];
  for (let k = 0; k < 2; k++) {
    let p = k;
    for (let i = k + 1; i < 3; i++)
      if (Math.abs(at(i, k)) > Math.abs(at(p, k))) p = i;
    if (at(p, k) === 0) return [NaN, NaN, NaN];
    if (p !== k) {
      for (let j = 0; j < 3; j++) {
        const t2 = a[3 * k + j];
        a[3 * k + j] = a[3 * p + j];
        a[3 * p + j] = t2;
      }
      const t = b[k];
      b[k] = b[p];
      b[p] = t;
    }
    const inv = 1 / at(k, k);
    for (let i = k + 1; i < 3; i++) a[3 * i + k] *= inv;
    for (let i = k + 1; i < 3; i++) {
      const l = a[3 * i + k];
      for (let j = k + 1; j < 3; j++) a[3 * i + j] -= l * a[3 * k + j];
    }
  }
  if (at(2, 2) === 0) return [NaN, NaN, NaN];
  for (let j = 0; j < 3; j++)
    for (let i = j + 1; i < 3; i++) b[i] -= b[j] * a[3 * i + j];
  for (let k = 2; k >= 0; k--) {
    b[k] /= at(k, k);
    for (let i = 0; i < k; i++) b[i] -= b[k] * a[3 * i + k];
  }
  return b;
}
var HandEnv21 = class {
  hand = "hand21";
  mujoco;
  model;
  data;
  L;
  filter;
  /** The official wrench is computed once per 1/120 s physics step and held (the harness's rate "isaac"). */
  wristMode = "isaac";
  substeps;
  decimation;
  h;
  dt;
  vmax;
  gains;
  gravity;
  /** r_wrist and every body below it (the articulation's links). */
  bodies;
  masses;
  velBuf;
  warnings;
  badqacc0 = 0;
  stepQpos;
  stepQvel;
  wrench = new Float64Array(6);
  base = new Float64Array(6);
  wrenchOn = false;
  /** The inputs of the last wrench (diagnostics, Node checks). */
  lastInputs = null;
  unstable = false;
  state;
  handPoints = new Float64Array(26 * 3);
  /** No QP in this drive (kept for the shared stats). */
  qpIters = 0;
  constructor(mujoco, model, data, layout, ctl, alphaWrist, alphaFingers) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    this.L = layout;
    this.h = model.opt.timestep;
    this.dt = ctl.isaac_dt ?? ISAAC_DT;
    this.substeps = Math.round(this.dt / this.h);
    if (Math.abs(this.substeps * this.h - this.dt) > 1e-6)
      throw new Error(`[mjweb] timestep ${this.h} does not divide 1/120 s`);
    this.decimation = 4 * this.substeps;
    if (model.nu < NUM_FINGERS)
      throw new Error("[mjweb] Hand 2.1 needs the 20 MJCF position actuators");
    const lo = new Float64Array(NUM_FINGERS);
    const hi = new Float64Array(NUM_FINGERS);
    for (let i = 0; i < NUM_FINGERS; i++) {
      lo[i] = layout.fingerRange[2 * i];
      hi[i] = layout.fingerRange[2 * i + 1];
    }
    this.filter = new TargetFilter(
      lo,
      hi,
      alphaWrist,
      alphaFingers,
      ctl.control_dt,
      ctl.max_ff_lin,
      ctl.max_ff_ang
    );
    if (ctl.finger_vel_limits.length !== NUM_FINGERS)
      throw new Error("[mjweb] expected 20 finger velocity limits");
    this.vmax = Float64Array.from(ctl.finger_vel_limits);
    const g = ctl.wrist_gains;
    this.gains = [g[0], g[1], g[2], g[3]];
    const grav = model.opt.gravity;
    this.gravity = ctl.gravity ? [...ctl.gravity] : [grav[0], grav[1], grav[2]];
    this.bodies = handBodyIds(model, layout.wristBody);
    if (this.bodies.includes(layout.objectBody))
      throw new Error("[mjweb] the object must not hang below r_wrist");
    const mass = model.body_mass;
    this.masses = Float64Array.from(this.bodies, (b) => mass[b]);
    this.stepQpos = new Float64Array(model.nq);
    this.stepQvel = new Float64Array(model.nv);
    const M = mujoco;
    this.velBuf = new M.DoubleBuffer(6);
    this.warnings = data.warning;
    this.state = this.readState();
  }
  dispose() {
    this.velBuf.delete();
  }
  badqacc() {
    return badqaccCount(this.warnings);
  }
  // ------------------------------------------------------------------ reset
  /** `MjHandEnv.reset` from the exact MuJoCo state (`webdata.json` starts are env.reset's qpos / qvel). */
  reset(qpos, qvel) {
    const { mujoco, model, data } = this;
    if (qpos.length !== model.nq || qvel.length !== model.nv)
      throw new Error("[mjweb] start state has the wrong size");
    mujoco.mj_resetData(model, data);
    data.qpos.set(qpos);
    data.qvel.set(qvel);
    const ctrl = data.ctrl;
    const q = data.qpos;
    for (let i = 0; i < NUM_FINGERS; i++) ctrl[i] = q[this.L.fingerQpos[i]];
    mujoco.mj_forward(model, data);
    this.filter.reset();
    this.base.fill(0);
    this.wrench.fill(0);
    this.wrenchOn = false;
    this.unstable = false;
    this.lastInputs = null;
    this.badqacc0 = this.badqacc();
    this.state = this.readState();
    this.readHandPoints();
    return this.state;
  }
  // ------------------------------------------------------------------ one control step
  beginStep(y) {
    this.filter.process(y);
    this.stepQpos.set(this.data.qpos);
    this.stepQvel.set(this.data.qvel);
  }
  /** Before `mj_step` of substep `s`: the finger targets, and at a physics step's start the wrist wrench. */
  preSubstep(s) {
    const data = this.data;
    const ctrl = data.ctrl;
    const joints = this.filter.joints;
    for (let i = 0; i < NUM_FINGERS; i++) ctrl[i] = joints[i];
    const xfrc = data.xfrc_applied;
    const row = 6 * this.L.wristBody;
    if (s === 0) for (let i = 0; i < 6; i++) this.base[i] = xfrc[row + i];
    if (s % this.substeps === 0) {
      this.computeWrench();
      for (let i = 0; i < 6; i++) xfrc[row + i] = this.base[i] + this.wrench[i];
      this.wrenchOn = true;
    }
  }
  /** After `mj_step` of substep `s`: the speed clamp; after the last one the fresh state. */
  postSubstep(s) {
    this.clampFingerSpeed();
    if (s === this.decimation - 1) {
      this.removeWrench();
      if (this.badqacc() > this.badqacc0) {
        this.unstable = true;
        this.data.qpos.set(this.stepQpos);
        this.data.qvel.set(this.stepQvel);
      }
      this.mujoco.mj_kinematics(this.model, this.data);
      this.state = this.readState();
      this.readHandPoints();
    }
  }
  /** The whole control step with MuJoCo's own `mj_step` (Node / tests; mjswan steps in the browser). */
  step(y) {
    this.beginStep(y);
    for (let s = 0; s < this.decimation; s++) {
      this.preSubstep(s);
      this.mujoco.mj_step(this.model, this.data);
      this.postSubstep(s);
    }
    return this.state;
  }
  removeWrench() {
    if (!this.wrenchOn) return;
    const xfrc = this.data.xfrc_applied;
    const row = 6 * this.L.wristBody;
    for (let i = 0; i < 6; i++) xfrc[row + i] = this.base[i];
    this.wrenchOn = false;
  }
  // ------------------------------------------------------------------ the official wrist law
  /** `wrench_inputs` of the current state (refreshed kinematics, COM velocity, composite body). */
  wrenchInputs() {
    const { mujoco, model, data } = this;
    mujoco.mj_kinematics(model, data);
    mujoco.mj_comPos(model, data);
    mujoco.mj_comVel(model, data);
    const w = this.L.wristBody;
    mujoco.mj_objectVelocity(
      model,
      data,
      mujoco.mjtObj.mjOBJ_BODY.value,
      w,
      this.velBuf,
      0
    );
    const res = this.velBuf.GetView();
    const omega = [res[0], res[1], res[2]];
    const vel = [res[3], res[4], res[5]];
    const xipos = data.xipos;
    const ximat = data.ximat;
    const inertiaDiag = model.body_inertia;
    const bodies = this.bodies;
    const m = this.masses;
    const mass = pairwiseSum(m);
    const sum = [0, 0, 0];
    for (let k = 0; k < bodies.length; k++) {
      const b = bodies[k];
      for (let i = 0; i < 3; i++)
        sum[i] = k === 0 ? m[k] * xipos[3 * b + i] : sum[i] + m[k] * xipos[3 * b + i];
    }
    const c = [sum[0] / mass, sum[1] / mass, sum[2] / mass];
    const inertia = new Array(9).fill(0);
    for (let k = 0; k < bodies.length; k++) {
      const b = bodies[k];
      const r = ximat.subarray(9 * b, 9 * b + 9);
      const di = inertiaDiag.subarray(3 * b, 3 * b + 3);
      const d = [
        xipos[3 * b] - c[0],
        xipos[3 * b + 1] - c[1],
        xipos[3 * b + 2] - c[2]
      ];
      const dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          let rot = 0;
          for (let a = 0; a < 3; a++)
            rot += r[3 * i + a] * di[a] * r[3 * j + a];
          const par = (i === j ? dd : 0) - d[i] * d[j];
          const v = rot + m[k] * par;
          inertia[3 * i + j] = k === 0 ? v : inertia[3 * i + j] + v;
        }
      }
    }
    const xpos = data.xpos;
    const xquat = data.xquat;
    return {
      pos: [xpos[3 * w], xpos[3 * w + 1], xpos[3 * w + 2]],
      quat: [
        xquat[4 * w],
        xquat[4 * w + 1],
        xquat[4 * w + 2],
        xquat[4 * w + 3]
      ],
      vel,
      omega,
      mass,
      com: c,
      inertia,
      ffPoint: [xipos[3 * w], xipos[3 * w + 1], xipos[3 * w + 2]]
    };
  }
  /** The wrench the current state and filter give (`MjHand21Env.wrench`), without applying it (Node checks). */
  evalWrench() {
    this.computeWrench();
    return Float64Array.from(this.wrench);
  }
  /** `wrench_actions_py` + the weight feed-forward; the result is applied at the r_wrist COM. */
  computeWrench() {
    const x = this.wrenchInputs();
    this.lastInputs = x;
    const f = this.filter;
    const [kpP, kdP, kpR, kdR] = this.gains;
    const dt = this.dt;
    const qErr = quatPositiveReal(quatMul(f.quat, quatConjugate(x.quat)));
    const k = kdP * x.mass / (x.mass + kdP * dt);
    const force = [0, 1, 2].map(
      (i) => kpP * (f.pos[i] - x.pos[i]) + k * (f.ffLin[i] - x.vel[i])
    );
    const I = x.inertia;
    const a = I.map((v, i) => i % 4 === 0 ? v + kdR * dt : v);
    const y = solve3(a, [
      f.ffAng[0] - x.omega[0],
      f.ffAng[1] - x.omega[1],
      f.ffAng[2] - x.omega[2]
    ]);
    const aa = axisAngleFromQuat(qErr);
    const iy = [0, 1, 2].map(
      (i) => I[3 * i] * y[0] + I[3 * i + 1] * y[1] + I[3 * i + 2] * y[2]
    );
    const torque = [0, 1, 2].map((i) => kpR * aa[i] + kdR * iy[i]);
    const fg = [
      -x.mass * this.gravity[0],
      -x.mass * this.gravity[1],
      -x.mass * this.gravity[2]
    ];
    const arm = [
      x.com[0] - x.ffPoint[0],
      x.com[1] - x.ffPoint[1],
      x.com[2] - x.ffPoint[2]
    ];
    const tg = cross(arm, fg);
    for (let i = 0; i < 3; i++) {
      this.wrench[i] = force[i] + fg[i];
      this.wrench[3 + i] = torque[i] + tg[i];
    }
  }
  clampFingerSpeed() {
    const qvel = this.data.qvel;
    const dof = this.L.fingerQvel;
    for (let i = 0; i < NUM_FINGERS; i++) {
      const v = qvel[dof[i]];
      const m = this.vmax[i];
      qvel[dof[i]] = v < -m ? -m : v > m ? m : v;
    }
  }
  readState() {
    const d = this.data;
    const L = this.L;
    const xpos = d.xpos;
    const xquat = d.xquat;
    const qpos = d.qpos;
    const w = L.wristBody;
    const o = L.objectBody;
    return {
      wristPos: [xpos[3 * w], xpos[3 * w + 1], xpos[3 * w + 2]],
      wristQuat: [
        xquat[4 * w],
        xquat[4 * w + 1],
        xquat[4 * w + 2],
        xquat[4 * w + 3]
      ],
      jointPos: Array.from(L.fingerQpos, (a) => qpos[a]),
      objectPos: [xpos[3 * o], xpos[3 * o + 1], xpos[3 * o + 2]],
      objectQuat: [
        xquat[4 * o],
        xquat[4 * o + 1],
        xquat[4 * o + 2],
        xquat[4 * o + 3]
      ]
    };
  }
  readHandPoints() {
    const xpos = this.data.xpos;
    const b = this.L.handPointBodies;
    for (let i = 0; i < b.length; i++) {
      for (let k = 0; k < 3; k++)
        this.handPoints[3 * i + k] = xpos[3 * b[i] + k];
    }
  }
};

// ../../../../../../web/plugins/success.ts
var LOST_WRIST_OBJECT_M = 0.25;
var LOST_REL_KP_M = 0.15;
var GoalMetrics = class {
  kp;
  goal;
  crit;
  tier;
  surface;
  axis;
  kpW;
  handO;
  constructor(keypoints, goal, criterion, fg) {
    if (!goal.fg || !fg?.surface)
      throw new Error(
        "[mjweb] the web data has no FG-v1 goal geometry: re-export it (build_web.py --rebuild-webdata)"
      );
    const tier = criterion.fg_tiers?.fg_v1;
    if (!tier || tier.length !== 4)
      throw new Error(
        "[mjweb] criterion.fg_tiers.fg_v1 missing from the web data"
      );
    this.kp = keypoints;
    this.goal = goal;
    this.crit = criterion;
    this.tier = tier;
    this.surface = Float64Array.from(fg.surface.flat());
    this.axis = fg.axial ? fg.axis : null;
    this.kpW = new Float64Array(3 * keypoints.length);
    this.handO = new Float64Array(26 * 3);
  }
  /** One post-control sample (`sample_metrics`); `ref` = the reference row at the clock. */
  sample(s, hand26World, ref) {
    const g = this.goal;
    const K = this.kp.length;
    let sum = 0;
    for (let k = 0; k < K; k++) {
      const pw = quatApply(s.objectQuat, this.kp[k]);
      const world = [
        pw[0] + s.objectPos[0],
        pw[1] + s.objectPos[1],
        pw[2] + s.objectPos[2]
      ];
      const kw = quatApplyInverse(s.wristQuat, [
        world[0] - s.wristPos[0],
        world[1] - s.wristPos[1],
        world[2] - s.wristPos[2]
      ]);
      this.kpW[3 * k] = kw[0];
      this.kpW[3 * k + 1] = kw[1];
      this.kpW[3 * k + 2] = kw[2];
      const t = g.object_keypoints_in_wrist[k];
      const dx = kw[0] - t[0], dy = kw[1] - t[1], dz = kw[2] - t[2];
      sum += dx * dx + dy * dy + dz * dz;
    }
    const rms = Math.sqrt(sum / K);
    let handMax = 0;
    for (let j = 0; j < 26; j++) {
      const ho = quatApplyInverse(s.objectQuat, [
        hand26World[3 * j] - s.objectPos[0],
        hand26World[3 * j + 1] - s.objectPos[1],
        hand26World[3 * j + 2] - s.objectPos[2]
      ]);
      this.handO[3 * j] = ho[0];
      this.handO[3 * j + 1] = ho[1];
      this.handO[3 * j + 2] = ho[2];
      const t = g.hand26_in_goal_object[j];
      handMax = Math.max(
        handMax,
        Math.sqrt(
          (ho[0] - t[0]) ** 2 + (ho[1] - t[1]) ** 2 + (ho[2] - t[2]) ** 2
        )
      );
    }
    const near = g.fg.near;
    let nearSum = 0;
    for (let i = 0; i < near.length; i++) {
      const j = Math.floor(near[i] / K);
      const k = near[i] - j * K;
      const p = this.kp[k];
      const dx = this.handO[3 * j] - p[0], dy = this.handO[3 * j + 1] - p[1], dz = this.handO[3 * j + 2] - p[2];
      nearSum += Math.abs(
        Math.sqrt(dx * dx + dy * dy + dz * dz) - g.fg.near_d[i]
      );
    }
    const fgNear = nearSum / Math.max(near.length, 1);
    const qow = quatMul(quatInv(s.wristQuat), s.objectQuat);
    const qg = g.object_quat_in_wrist;
    const rotErr = quatAngle(qow, qg);
    let axialErr = NaN;
    let fgRot;
    if (this.axis) {
      const a = quatApply(qow, this.axis);
      const b = quatApply(qg, this.axis);
      axialErr = Math.acos(
        Math.min(1, Math.max(-1, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))
      );
      fgRot = axialErr;
    } else {
      fgRot = 2 * Math.acos(
        Math.min(
          Math.abs(
            qow[0] * qg[0] + qow[1] * qg[1] + qow[2] * qg[2] + qow[3] * qg[3]
          ),
          1
        )
      );
    }
    const rel = quatApplyInverse(s.wristQuat, [
      s.objectPos[0] - s.wristPos[0],
      s.objectPos[1] - s.wristPos[1],
      s.objectPos[2] - s.wristPos[2]
    ]);
    const gp = g.object_pos_in_wrist;
    const posErr = Math.sqrt(
      (rel[0] - gp[0]) ** 2 + (rel[1] - gp[1]) ** 2 + (rel[2] - gp[2]) ** 2
    );
    let fgContact = 0;
    const surf = this.surface;
    for (let i = 0; i < 5; i++) {
      if (!g.fg.contact[i]) continue;
      const j = 21 + i;
      const x = this.handO[3 * j], y = this.handO[3 * j + 1], z = this.handO[3 * j + 2];
      let best = Infinity;
      for (let q = 0; q < surf.length; q += 3) {
        const dx = x - surf[q], dy = y - surf[q + 1], dz = z - surf[q + 2];
        const d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < best) best = d2;
      }
      fgContact = Math.max(fgContact, Math.sqrt(best));
    }
    let objectRelKpErr = NaN;
    if (ref) {
      let acc = 0;
      for (let k = 0; k < K; k++) {
        const pw = quatApply(ref.objectQuat, this.kp[k]);
        const kr = quatApplyInverse(ref.wristQuat, [
          pw[0] + ref.objectPos[0] - ref.wristPos[0],
          pw[1] + ref.objectPos[1] - ref.wristPos[1],
          pw[2] + ref.objectPos[2] - ref.wristPos[2]
        ]);
        const dx = this.kpW[3 * k] - kr[0], dy = this.kpW[3 * k + 1] - kr[1], dz = this.kpW[3 * k + 2] - kr[2];
        acc += Math.sqrt(dx * dx + dy * dy + dz * dz);
      }
      objectRelKpErr = acc / K;
    }
    const dw = [
      s.objectPos[0] - s.wristPos[0],
      s.objectPos[1] - s.wristPos[1],
      s.objectPos[2] - s.wristPos[2]
    ];
    const wristObjectDist = Math.sqrt(
      dw[0] * dw[0] + dw[1] * dw[1] + dw[2] * dw[2]
    );
    const passed = rms < this.crit.object_rms_m && handMax <= this.crit.hand_max_m;
    const [tNear, tRot, tPos, tContact] = this.tier;
    const fgFails = {
      near: !(fgNear <= tNear),
      rot: !(fgRot <= tRot),
      pos: !(posErr <= tPos),
      contact: !(fgContact <= tContact)
    };
    const fgPassed = !fgFails.near && !fgFails.rot && !fgFails.pos && !fgFails.contact;
    return {
      rms,
      handMax,
      posErr,
      rotErr,
      axialErr,
      passed,
      fgNear,
      fgRot,
      fgPos: posErr,
      fgContact,
      fgPassed,
      fgFails,
      objectRelKpErr,
      wristObjectDist
    };
  }
};
var HoldScore = class {
  counts = 0;
  fgCounts = 0;
  samples = 0;
  anySuccess = false;
  firstLost = -1;
  firstNonfinite = -1;
  hold;
  constructor(holdSamples) {
    this.hold = holdSamples;
  }
  update(m, nonfinite = false) {
    this.counts = m.passed ? this.counts + 1 : 0;
    this.fgCounts = m.fgPassed ? this.fgCounts + 1 : 0;
    this.anySuccess = this.anySuccess || this.counts >= this.hold;
    if (nonfinite && this.firstNonfinite < 0)
      this.firstNonfinite = this.samples;
    if (this.firstLost < 0 && (m.wristObjectDist > LOST_WRIST_OBJECT_M || m.objectRelKpErr > LOST_REL_KP_M || this.firstNonfinite >= 0)) {
      this.firstLost = this.samples;
    }
    this.samples += 1;
  }
  /** main `terminal_hold_success` once the final sample is in. */
  get success() {
    return this.counts >= this.hold;
  }
  /** `fg_v1_terminal` once the final sample is in. */
  get fgSuccess() {
    return this.fgCounts >= this.hold;
  }
};
function mainOutcome(score, last, unstableSample, crit) {
  const lost = score.firstLost;
  if (unstableSample < 0 && score.success) return "success";
  if (unstableSample >= 0 && !(lost >= 0 && lost < unstableSample))
    return "unstable";
  if (lost >= 0 || !last) return "lost";
  const badRms = !(last.rms < crit.object_rms_m);
  const badHand = !(last.handMax <= crit.hand_max_m);
  if (badRms && badHand) return "held_off_both";
  if (badRms) return "held_off_rms";
  if (badHand) return "held_off_hand";
  return "short_hold";
}
function fgOutcome(score, last, unstableSample) {
  const lost = score.firstLost;
  if (unstableSample < 0 && score.fgSuccess) return "success";
  if (unstableSample >= 0 && !(lost >= 0 && lost < unstableSample))
    return "unstable";
  if (lost >= 0 || !last) return "lost";
  return last.fgPassed ? "short_hold" : "off";
}

// ../../../../../../web/plugins/perturb.ts
var PSTART_RANGES = {
  pos: 0.02,
  rot: 0.15,
  finger: 0.05
};
function samplePerturbation(rng, r = PSTART_RANGES) {
  const dp = [rng.symmetric(r.pos), rng.symmetric(r.pos), rng.symmetric(r.pos)];
  const axis = normalize3([rng.normal(), rng.normal(), rng.normal()]);
  const angle = rng.symmetric(r.rot);
  const dFinger = Array.from({ length: 20 }, () => rng.symmetric(r.finger));
  return { dp, axis, angle, dFinger };
}
function applyPerturbation(qpos, qvel, L, p) {
  const q = Array.from(qpos);
  const v = Array.from(qvel);
  const dq = quatFromAngleAxis(p.angle, p.axis);
  const w = L.wristQpos;
  const o = L.objectQpos;
  const wp = [q[w], q[w + 1], q[w + 2]];
  const op = [q[o], q[o + 1], q[o + 2]];
  const rel = quatApply(dq, [op[0] - wp[0], op[1] - wp[1], op[2] - wp[2]]);
  for (let i = 0; i < 3; i++) {
    q[o + i] = wp[i] + p.dp[i] + rel[i];
    q[w + i] = wp[i] + p.dp[i];
  }
  const wq = quatMul(dq, [q[w + 3], q[w + 4], q[w + 5], q[w + 6]]);
  const oq = quatMul(dq, [q[o + 3], q[o + 4], q[o + 5], q[o + 6]]);
  for (let i = 0; i < 4; i++) {
    q[w + 3 + i] = wq[i];
    q[o + 3 + i] = oq[i];
  }
  for (const base of [L.wristQvel, L.objectQvel]) {
    const lin = quatApply(dq, [v[base], v[base + 1], v[base + 2]]);
    for (let i = 0; i < 3; i++) v[base + i] = lin[i];
  }
  for (let i = 0; i < 20; i++) {
    const a = L.fingerQpos[i];
    q[a] = Math.min(
      Math.max(q[a] + p.dFinger[i], L.fingerRange[2 * i]),
      L.fingerRange[2 * i + 1]
    );
  }
  return { qpos: q, qvel: v };
}

// ../../../../../../web/plugins/rng.ts
function splitmix32(state) {
  state.s = state.s + 2654435769 | 0;
  let z = state.s;
  z = Math.imul(z ^ z >>> 16, 2246822507);
  z = Math.imul(z ^ z >>> 13, 3266489909);
  return (z ^ z >>> 16) >>> 0;
}
var Rng = class {
  a;
  b;
  c;
  d;
  spare = null;
  constructor(seed, stream = 0) {
    const st = {
      s: Math.trunc(seed) ^ Math.imul(stream + 1, 1663821227) | 0
    };
    this.a = splitmix32(st);
    this.b = splitmix32(st);
    this.c = splitmix32(st);
    this.d = splitmix32(st);
    if ((this.a | this.b | this.c | this.d) === 0) this.a = 1;
  }
  /** Next 32-bit unsigned integer (xoshiro128**). */
  nextU32() {
    const result = Math.imul(rotl(Math.imul(this.b, 5), 7), 9) >>> 0;
    const t = this.b << 9;
    this.c ^= this.a;
    this.d ^= this.b;
    this.b ^= this.c;
    this.a ^= this.d;
    this.c ^= t;
    this.d = rotl(this.d, 11);
    return result;
  }
  /** Uniform double in [0, 1) with 53 random bits. */
  random() {
    const hi = this.nextU32() >>> 5;
    const lo = this.nextU32() >>> 6;
    return (hi * 67108864 + lo) / 9007199254740992;
  }
  /** Uniform in [-amp, amp). */
  symmetric(amp) {
    return (this.random() * 2 - 1) * amp;
  }
  /** Integer in [lo, hi] (inclusive). */
  integer(lo, hi) {
    return lo + Math.min(hi - lo, Math.floor(this.random() * (hi - lo + 1)));
  }
  /** Standard normal (Box-Muller, the second value is kept for the next call). */
  normal() {
    if (this.spare !== null) {
      const v2 = this.spare;
      this.spare = null;
      return v2;
    }
    let u = 0;
    while (u <= 0) u = this.random();
    const v = this.random();
    const r = Math.sqrt(-2 * Math.log(u));
    this.spare = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v);
  }
};
function rotl(x, k) {
  return x << k | x >>> 32 - k;
}

// ../../../../../../web/plugins/episode.ts
function makeEnv(mujoco, model, data, wd, L, alphaWrist, alphaFingers, wristMode) {
  if ((wd.hand ?? "beta1") === "hand21") {
    if (wristMode !== "isaac")
      throw new Error(
        `[mjweb] Hand 2.1 holds the official wrench per physics step (wrist_mode "isaac"), not ${wristMode}`
      );
    return new HandEnv21(
      mujoco,
      model,
      data,
      L,
      wd.controller,
      alphaWrist,
      alphaFingers
    );
  }
  return new HandEnv(
    mujoco,
    model,
    data,
    L,
    wd.controller,
    alphaWrist,
    alphaFingers,
    wristMode
  );
}
var Episode = class {
  wd;
  /** null: no policy (the scene, its starts and ghosts only). */
  policy;
  env;
  L;
  /** The layout of the web data's start states (= `L`, except under another base). */
  startLayout;
  settings;
  startIndex = 0;
  start;
  ref;
  view;
  metrics;
  score;
  last = null;
  /** The relational metrics of the start state itself (shown without a policy; not a sample). */
  startMetrics = null;
  perturbation = null;
  /** Clock row of the next step (start_t + samples). */
  t = 0;
  /** motion_steps + hold rows: the clock runs start_t .. length - 1. */
  length = 0;
  done = false;
  /** MuJoCo blew up (mjWARN_BADQACC) in this episode: it ended there, as a failure. */
  unstable = false;
  /** Samples taken before the blow-up (the harness's `unstable_sample`), else -1. */
  unstableSample = -1;
  /** ONNX inference failed (mjswan then holds the last action): the episode ended there, as a failure. */
  inferenceError = null;
  epoch = 0;
  obsEpoch = -1;
  stepEpoch = -1;
  stepValid = false;
  lastObs = null;
  lastAction = null;
  stepNonfinite = false;
  recording = false;
  record = [];
  /** Wall-clock ms of the last `endStep` work (metrics + history), for the stats. */
  endStepMs = 0;
  constructor(mujoco, model, data, wd, policyId, settings, wristMode = "isaac", base = null) {
    this.wd = wd;
    let policy = null;
    if (policyId !== null) {
      policy = wd.policies.find((p) => p.id === policyId) ?? null;
      if (!policy)
        throw new Error(
          `[mjweb] policy ${policyId} not in webdata of ${wd.object_id}`
        );
    }
    this.policy = policy;
    this.L = base ? base.layout : layoutOf(mujoco, model, wd.controller.joint_order);
    this.startLayout = base ? base.startLayout : this.L;
    const aw = policy?.target_ema.wrist ?? 0.3;
    const af = policy?.target_ema.fingers ?? 0.3;
    this.env = base ? base.makeEnv(this.L, aw, af) : makeEnv(mujoco, model, data, wd, this.L, aw, af, wristMode);
    this.settings = { ...settings };
    this.start = wd.starts[0];
    this.ref = wd.references[this.start.motion];
    this.view = new StudentView(EXACT, null);
    this.metrics = this.goalFor(this.ref);
    this.score = new HoldScore(wd.criterion.hold_samples);
  }
  dispose() {
    this.env.dispose();
  }
  goalFor(ref) {
    return new GoalMetrics(
      this.wd.keypoints,
      ref.goal,
      this.wd.criterion,
      this.wd.fg
    );
  }
  /** The reference row at the clock (rows past the motion hold the final row), for the lost rule. */
  refPose(t) {
    const r = Math.max(0, Math.min(t, this.length - 1));
    const ref = this.ref;
    return {
      wristPos: ref.wrist_pos[r],
      wristQuat: ref.wrist_quat[r],
      objectPos: ref.object_pos[r],
      objectQuat: ref.object_quat[r]
    };
  }
  /** Number of control steps of the current episode. */
  get total() {
    return this.length - this.start.start_t;
  }
  get samples() {
    return this.score.samples;
  }
  /** main (FullHand v1) `terminal_hold_success` of the finished episode. */
  get success() {
    return this.done && !this.unstable && this.inferenceError === null && this.score.success;
  }
  /** FG-v1 `fg_v1_terminal` of the finished episode. */
  get fgSuccess() {
    return this.done && !this.unstable && this.inferenceError === null && this.score.fgSuccess;
  }
  /** Why an unstable episode ended, when the base names it (the arm's guards); null: MuJoCo's blow-up. */
  get unstableReason() {
    return null;
  }
  /** How the finished episode ended for main (null while running or after an inference failure). */
  get mainOutcome() {
    if (!this.done || this.inferenceError !== null) return null;
    return mainOutcome(
      this.score,
      this.last,
      this.unstableSample,
      this.wd.criterion
    );
  }
  /** How the finished episode ended for FG-v1 (null while running or after an inference failure). */
  get fgOutcome() {
    if (!this.done || this.inferenceError !== null) return null;
    return fgOutcome(this.score, this.last, this.unstableSample);
  }
  /** The Isaac outcome of the selected start for this policy (null without a policy or a known outcome). */
  isaacOutcome() {
    return this.policy ? this.start.isaac[this.policy.id] ?? null : null;
  }
  /** Hand 2.1 (official physics) or the old beta1 physics. */
  get hand() {
    return this.env.hand;
  }
  selectStart(index) {
    if (!(index >= 0 && index < this.wd.starts.length))
      throw new Error(`[mjweb] no start #${index}`);
    this.startIndex = index;
  }
  /** Reset into the selected start (perturbed when enabled); returns the measured state. */
  reset() {
    this.epoch += 1;
    this.start = this.wd.starts[this.startIndex];
    this.ref = this.wd.references[this.start.motion];
    this.metrics = this.goalFor(this.ref);
    this.score = new HoldScore(this.wd.criterion.hold_samples);
    this.last = null;
    this.done = false;
    this.unstable = false;
    this.unstableSample = -1;
    this.inferenceError = null;
    this.length = this.ref.motion_steps + this.wd.controller.hold_rows;
    if (this.ref.rows !== this.length)
      throw new Error(
        `[mjweb] reference ${this.ref.case}: ${this.ref.rows} rows != ${this.length}`
      );
    this.t = this.start.start_t;
    let qpos = this.start.qpos;
    let qvel = this.start.qvel;
    this.perturbation = null;
    if (this.settings.perturb) {
      this.perturbation = samplePerturbation(new Rng(this.settings.seed, 2));
      ({ qpos, qvel } = applyPerturbation(
        qpos,
        qvel,
        this.startLayout,
        this.perturbation
      ));
    }
    const noise = this.settings.sense ? this.wd.sense ?? EVAL_SENSE : EXACT;
    this.view = new StudentView(
      noise,
      this.settings.sense ? new Rng(this.settings.seed, 1) : null
    );
    const s = this.resetEnv(qpos, qvel);
    this.startMetrics = this.metrics.sample(
      s,
      this.env.handPoints,
      this.refPose(this.t)
    );
    this.view.push(this.view.measure(s, measuredTarget(s)));
    this.record = [];
    this.lastObs = null;
    this.lastAction = null;
    return s;
  }
  /** The env's reset into the (perturbed) start state; another base overrides it (the arm's phases). */
  resetEnv(qpos, qvel) {
    return this.env.reset(qpos, qvel);
  }
  /** The observation for the next control step (stamps it with the current epoch). */
  observe() {
    this.obsEpoch = this.epoch;
    const obs = this.view.observation();
    this.lastObs = obs;
    return obs;
  }
  /** End the episode as a failure because the policy did not run (its action would be stale). */
  failInference(message) {
    if (this.done) return;
    this.inferenceError = message;
    this.done = true;
  }
  /** Substep 0 of a control step, with the policy's raw 29-D output. */
  beginStep(action) {
    this.stepEpoch = this.epoch;
    this.stepValid = this.policy !== null && this.obsEpoch === this.epoch && !this.done && action !== null && action.length >= ACT_DIM;
    if (!this.stepValid || !action) return;
    const y = new Float64Array(ACT_DIM);
    let finite = true;
    for (let i = 0; i < ACT_DIM; i++) {
      y[i] = action[i];
      if (!Number.isFinite(y[i])) finite = false;
    }
    this.stepNonfinite = !finite;
    if (!finite) {
      const r = Math.min(this.t, this.length - 1);
      const ref = this.ref;
      y.set([
        ...ref.wrist_pos[r],
        ...rot6d(ref.wrist_quat[r]),
        ...ref.joints[r]
      ]);
    }
    this.lastAction = Float32Array.from(action);
    this.env.beginStep(y);
  }
  preSubstep(s) {
    if (this.stepValid && this.stepEpoch === this.epoch) this.env.preSubstep(s);
  }
  postSubstep(s) {
    if (this.stepValid && this.stepEpoch === this.epoch)
      this.env.postSubstep(s);
  }
  /** After the control step: the sample, the hold counter and the next history frame. */
  endStep() {
    if (this.stepEpoch !== this.epoch) return "ignored";
    if (!this.stepValid) {
      this.stepValid = false;
      if (this.done) return "ignored";
      return "stale";
    }
    this.stepValid = false;
    const t0 = typeof performance !== "undefined" ? performance.now() : 0;
    const env = this.env;
    if (env.unstable) {
      this.unstable = true;
      this.unstableSample = this.score.samples;
      this.done = true;
      return "unstable";
    }
    const s = env.state;
    const m = this.metrics.sample(s, env.handPoints, this.refPose(this.t));
    this.last = m;
    this.score.update(m, this.stepNonfinite);
    const applied = env.filter.applied();
    if (this.recording) {
      this.record.push({
        t: this.t,
        wristPos: s.wristPos,
        wristQuat: s.wristQuat,
        joints: s.jointPos,
        objectPos: s.objectPos,
        objectQuat: s.objectQuat,
        target: applied,
        obs: this.lastObs ? Array.from(this.lastObs) : [],
        action: this.lastAction ? Array.from(this.lastAction) : [],
        rms: m.rms,
        handMax: m.handMax,
        passed: m.passed,
        hand26: Array.from(env.handPoints),
        fgNear: m.fgNear,
        fgRot: m.fgRot,
        fgPos: m.fgPos,
        fgContact: m.fgContact,
        fgPassed: m.fgPassed,
        objectRelKpErr: m.objectRelKpErr,
        wristObjectDist: m.wristObjectDist,
        nonfinite: this.stepNonfinite
      });
    }
    if (this.t >= this.length - 1) {
      this.done = true;
      this.endStepMs = (typeof performance !== "undefined" ? performance.now() : 0) - t0;
      return "done";
    }
    this.view.push(this.view.measure(s, applied));
    this.t += 1;
    this.endStepMs = (typeof performance !== "undefined" ? performance.now() : 0) - t0;
    return "step";
  }
  /** Whether the last step's raw output was non-finite (replaced by the reference row). */
  get lastNonfinite() {
    return this.stepNonfinite;
  }
};

// ../../../../../../web/plugins/ghosts.ts
var THREE = __toESM(require_plugin_three_shim());
var GHOST_COLORS = {
  reference: 4892927
};
function setPose(obj, p, q) {
  obj.position.set(p[0], p[2], -p[1]);
  obj.quaternion.set(-q[1], -q[3], q[2], -q[0]);
}
function ghostMaterial(color, opacity) {
  return new THREE.MeshStandardMaterial({
    color,
    transparent: true,
    opacity,
    depthWrite: false,
    roughness: 0.6,
    metalness: 0
  });
}
function cloneBody(source, material, name) {
  const clone = source.clone(true);
  clone.name = name;
  clone.userData = { ignoreDragForce: true, mjwebGhost: true };
  clone.traverse((node) => {
    const mesh = node;
    if (mesh.isMesh) {
      mesh.material = material;
      mesh.castShadow = false;
      mesh.receiveShadow = false;
      mesh.renderOrder = 10;
      mesh.bodyID = -1;
      mesh.userData = { ...mesh.userData, ignoreDragForce: true };
    }
  });
  return clone;
}
function applyBackdrop(scene) {
  const previous = scene.background;
  if (typeof document === "undefined") return previous;
  const canvas = document.createElement("canvas");
  canvas.width = 4;
  canvas.height = 256;
  const ctx = canvas.getContext("2d");
  if (!ctx) return previous;
  const g = ctx.createLinearGradient(0, 0, 0, 256);
  g.addColorStop(0, "#f7f8fa");
  g.addColorStop(0.6, "#e9ecf0");
  g.addColorStop(1, "#d6dbe2");
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 4, 256);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  scene.background = tex;
  return previous;
}
var CHECKER_TILE_M = 0.2;
function applyCheckerFloor(floor) {
  if (typeof document === "undefined") return () => {
  };
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = 256;
  const ctx = canvas.getContext("2d");
  if (!ctx) return () => {
  };
  const a = "rgb(51,77,102)";
  const b = "rgb(26,51,77)";
  for (let i = 0; i < 2; i++)
    for (let j = 0; j < 2; j++) {
      ctx.fillStyle = (i + j) % 2 === 0 ? a : b;
      ctx.fillRect(i * 128, j * 128, 128, 128);
    }
  ctx.strokeStyle = "rgba(204,204,204,0.6)";
  ctx.lineWidth = 2;
  ctx.strokeRect(1, 1, 254, 254);
  const restore = [];
  floor.traverse((node) => {
    const mesh = node;
    if (!mesh.isMesh) return;
    mesh.geometry.computeBoundingBox();
    const box = mesh.geometry.boundingBox;
    const size = box ? box.getSize(new THREE.Vector3()) : new THREE.Vector3(5, 5, 5);
    const extent = [size.x, size.y, size.z].sort((u, v) => v - u);
    const tex = new THREE.CanvasTexture(canvas);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(extent[0] / (2 * CHECKER_TILE_M), extent[1] / (2 * CHECKER_TILE_M));
    tex.anisotropy = 8;
    const old = mesh.material;
    mesh.material = new THREE.MeshStandardMaterial({ map: tex, roughness: 0.85, metalness: 0 });
    restore.push(() => {
      mesh.material.dispose();
      tex.dispose();
      mesh.material = old;
    });
  });
  return () => restore.forEach((f) => f());
}
var DEFAULT_VISIBILITY = {
  reference: false,
  inset: true
};
var Ghosts = class {
  mujoco;
  model;
  fk;
  L;
  root;
  handBodies;
  handClones;
  refObject;
  materials;
  visibility = { ...DEFAULT_VISIBILITY };
  constructor(mujoco, model, bodies, parent, L) {
    this.mujoco = mujoco;
    this.model = model;
    this.L = L;
    this.fk = new mujoco.MjData(model);
    this.root = new THREE.Group();
    this.root.name = "mjweb ghosts";
    this.root.userData = { ignoreDragForce: true };
    const refMat = ghostMaterial(GHOST_COLORS.reference, 0.22);
    this.materials = [refMat];
    const parentId = model.body_parentid;
    const inHand = (b) => {
      for (let p = b; p > 0; p = parentId[p])
        if (p === L.wristBody) return true;
      return false;
    };
    this.handBodies = [];
    for (let b = 1; b < model.nbody; b++)
      if (inHand(b) && bodies[b]) this.handBodies.push(b);
    this.handClones = this.handBodies.map(
      (b) => cloneBody(bodies[b], refMat, `ghost ${bodies[b].name}`)
    );
    const src = bodies[L.objectBody];
    this.refObject = src ? cloneBody(src, refMat, "ghost object (reference)") : null;
    for (const c of [...this.handClones, this.refObject])
      if (c) this.root.add(c);
    parent.add(this.root);
    this.applyVisibility();
  }
  setVisibility(v) {
    this.visibility = { ...v };
    this.applyVisibility();
  }
  applyVisibility() {
    for (const c of this.handClones) c.visible = this.visibility.reference;
    if (this.refObject) this.refObject.visible = this.visibility.reference;
  }
  /** Test hook: the overlays drawn now ("reference hand" / "reference object"; the arm's approach object ghost is the
   *  latter), and how many objects our overlay group holds (the reference clones only, since 10-20). */
  drawn() {
    const visible = [];
    if (this.handClones.some((c) => c.visible)) visible.push("reference hand");
    if (this.refObject?.visible) visible.push("reference object");
    return { visible, objects: this.root.children.length };
  }
  /** Forward kinematics of hand + object on the ghost `MjData`. */
  kinematics(wristPos, wristQuat, joints, objectPos, objectQuat) {
    const L = this.L;
    const qpos = this.fk.qpos;
    qpos.fill(0);
    const floating = L.wristQpos >= 0;
    if (floating) {
      qpos.set(wristPos, L.wristQpos);
      qpos.set(wristQuat, L.wristQpos + 3);
    }
    for (let i = 0; i < 20; i++) qpos[L.fingerQpos[i]] = joints[i];
    qpos.set(objectPos, L.objectQpos);
    qpos.set(objectQuat, L.objectQpos + 3);
    this.mujoco.mj_kinematics(this.model, this.fk);
    if (!floating) this.moveHand(wristPos, wristQuat);
  }
  /**
   * Without a free wrist (the arm scene) the hand is posed rigidly: every hand body of the FK data is carried by
   * T_wrist(wanted) inv(T_wrist(fk)), so the hand's internal pose is the joints' and its r_wrist is the given one.
   */
  moveHand(wristPos, wristQuat) {
    const xpos = this.fk.xpos;
    const xquat = this.fk.xquat;
    const w = this.L.wristBody;
    const fp = [xpos[3 * w], xpos[3 * w + 1], xpos[3 * w + 2]];
    const fq = [
      xquat[4 * w],
      xquat[4 * w + 1],
      xquat[4 * w + 2],
      xquat[4 * w + 3]
    ];
    for (const b of this.handBodies) {
      const rel = poseInFrame(
        fp,
        fq,
        xpos.subarray(3 * b, 3 * b + 3),
        xquat.subarray(4 * b, 4 * b + 4)
      );
      const out = poseFromFrame(wristPos, wristQuat, rel.pos, rel.quat);
      xpos.set(out.pos, 3 * b);
      xquat.set(out.quat, 4 * b);
    }
  }
  /** Reference ghost at row `row` (clamped to the last row) by forward kinematics. */
  setReferenceRow(ref, row) {
    const r = Math.max(0, Math.min(row, ref.rows - 1));
    this.kinematics(
      ref.wrist_pos[r],
      ref.wrist_quat[r],
      ref.joints[r],
      ref.object_pos[r],
      ref.object_quat[r]
    );
    const xpos = this.fk.xpos;
    const xquat = this.fk.xquat;
    this.handBodies.forEach(
      (b, i) => setPose(
        this.handClones[i],
        xpos.subarray(3 * b, 3 * b + 3),
        xquat.subarray(4 * b, 4 * b + 4)
      )
    );
    const o = this.L.objectBody;
    if (this.refObject)
      setPose(
        this.refObject,
        xpos.subarray(3 * o, 3 * o + 3),
        xquat.subarray(4 * o, 4 * o + 4)
      );
  }
  /** The reference-colour object ghost at a world pose (the arm's phases 0-1: the start's object). */
  /** Test hook: whether the reference object ghost is drawn (phases 0-1 of the arm page show it only on request). */
  get objectGhostVisible() {
    return !!this.refObject?.visible;
  }
  setObjectGhost(objectPos, objectQuat) {
    if (this.refObject) {
      setPose(this.refObject, objectPos, objectQuat);
      this.refObject.visible = true;
    }
  }
  dispose() {
    this.root.removeFromParent();
    for (const m of this.materials) m.dispose();
    this.fk.delete();
  }
};

// ../../../../../../web/plugins/panel.ts
var PANEL_ID = "mjweb-panel";
var CSS = `
#${PANEL_ID} { position: fixed; left: 12px; top: 12px; z-index: 1000; width: 412px; max-height: calc(100vh - 24px);
  overflow-y: auto; padding: 10px 12px; border-radius: 10px; background: rgba(16, 19, 26, 0.86); color: #eef1f6;
  font: 12.5px/1.45 -apple-system, "PingFang SC", "Hiragino Sans GB", "Noto Sans SC", "Microsoft YaHei", sans-serif;
  box-shadow: 0 4px 18px rgba(0,0,0,0.35); user-select: none; }
#${PANEL_ID} h1 { font-size: 14px; margin: 0 0 2px; font-weight: 650; }
#${PANEL_ID} .note { color: #ffd479; font-size: 12px; margin: 2px 0 8px; }
#${PANEL_ID} .row { display: flex; align-items: center; gap: 6px; margin: 3px 0; flex-wrap: wrap; }
#${PANEL_ID} .col { display: flex; flex-direction: column; gap: 2px; flex: 1; min-width: 0; }
#${PANEL_ID} .lab { width: 44px; flex: none; color: #9aa6b8; align-self: flex-start; }
#${PANEL_ID} button { background: #2a3140; color: #eef1f6; border: 1px solid #3b4558; border-radius: 6px;
  padding: 2px 7px; font: inherit; cursor: pointer; }
#${PANEL_ID} button:hover { background: #343d50; }
#${PANEL_ID} button.on { background: #2f6fde; border-color: #4a86f0; }
#${PANEL_ID} button:disabled { opacity: 0.45; cursor: default; }
#${PANEL_ID} select { background: #222937; color: #eef1f6; border: 1px solid #3b4558; border-radius: 6px;
  padding: 2px 4px; font: inherit; max-width: 350px; min-width: 0; flex: 1; }
#${PANEL_ID} input[type=number] { width: 64px; background: #222937; color: #eef1f6; border: 1px solid #3b4558;
  border-radius: 6px; padding: 1px 4px; font: inherit; }
#${PANEL_ID} label { display: inline-flex; align-items: center; gap: 4px; cursor: pointer; }
#${PANEL_ID} .swatch { display: inline-block; width: 10px; height: 10px; border-radius: 2px; flex: none; opacity: 0.85; }
#${PANEL_ID} .sub { color: #9aa6b8; font-size: 11.5px; }
#${PANEL_ID} .live { margin-top: 8px; padding-top: 6px; border-top: 1px solid #333c4d; font-variant-numeric: tabular-nums; }
#${PANEL_ID} .live div { margin: 1px 0; }
#${PANEL_ID} .ok { color: #46d483; } #${PANEL_ID} .bad { color: #ff6b6b; } #${PANEL_ID} .dim { color: #9aa6b8; }
#${PANEL_ID} .result { font-size: 13px; font-weight: 600; margin-top: 4px; }
#${PANEL_ID} .result div { margin: 1px 0; }
#${PANEL_ID} .result .dim { font-weight: 400; font-size: 12px; }
#${PANEL_ID} .banner { margin: 4px 0 8px; padding: 6px 8px; border-radius: 6px; background: #5c1f24; color: #ffd9d9;
  border: 1px solid #a8434b; font-weight: 600; }
#${PANEL_ID} .nopolicy { margin: 4px 0; padding: 6px 8px; border-radius: 6px; background: #22324a; color: #dbe8ff;
  border: 1px solid #36507a; }
#${PANEL_ID} .dry { margin: 4px 0; padding: 4px 8px; border-radius: 6px; background: #4a3a12; color: #ffe7a8;
  border: 1px solid #8a6a1c; font-size: 12px; }
#${PANEL_ID} .prow { display: flex; align-items: center; gap: 5px; flex-wrap: wrap; }
#${PANEL_ID} .prow .rl { color: #c7d0dd; min-width: 54px; }
`;
var TIP = {
  ghost: "reference \u4ECE\u8D77\u59CB\u884C\u5F00\u59CB\u56DE\u653E\u7684\u624B\u548C\u7269\u4F53\uFF08\u84DD\uFF09\u3002",
  inset: "\u53F3\u4E0B\u89D2\u7684\u9759\u6001\u5C0F\u56FE\uFF1A\u793A\u8303\u6700\u540E\u4E00\u5E27\u7684\u624B\u548C\u7269\u4F53\uFF08functional grasp\uFF0C\u4E5F\u5C31\u662F\u8981\u8FBE\u5230\u7684\u624B\u2013\u7269\u4F53\u76F8\u5BF9\u4F4D\u7F6E\uFF09\uFF0C\u548C\u62A5\u544A\u89C6\u9891\u91CC\u7684\u5C0F\u56FE\u76F8\u540C\u3002",
  rel: "\u7269\u4F53\u5728\u5F53\u524D\u624B\u8155\u5750\u6807\u7CFB\u91CC\u7684\u4F4D\u7F6E\u548C\u59FF\u6001\uFF0C\u5BF9\u6BD4\u793A\u8303\u6700\u540E\u4E00\u5E27\u91CC\u7269\u4F53\u5728\u624B\u8155\u5750\u6807\u7CFB\u91CC\u7684\u4F4D\u7F6E\u548C\u59FF\u6001\uFF08\u4E0E\u4E16\u754C\u5750\u6807\u65E0\u5173\uFF09\u3002",
  main: "FullHand v1 main\uFF1A\u2460 \u7269\u4F53 100 \u4E2A\u5173\u952E\u70B9\u5728\u5F53\u524D\u624B\u8155\u7CFB\u91CC\uFF0C\u5BF9\u793A\u8303\u6700\u540E\u4E00\u5E27\u7684 RMS < 12 mm\uFF1B\u2461 \u624B\u90E8 26 \u70B9\uFF08\u624B\u8155\u300120 \u4E2A\u5173\u8282\u5B50 link\u30015 \u4E2A\u6307\u5C16\uFF09\u5728\u5F53\u524D\u7269\u4F53\u7CFB\u91CC\uFF0C\u5BF9\u793A\u8303\u6700\u540E\u4E00\u5E27\u7684\u6700\u5927\u504F\u5DEE \u2264 13 mm\u3002\u672B\u5C3E\u8FDE\u7EED 15 \u4E2A\u6837\u672C\u90FD\u6EE1\u8DB3\u624D\u7B97\u6210\u529F\u3002",
  fg: "FG-v1\uFF08docs/dexfun/CRITERION.md\uFF09\uFF1A\u2460 \u793A\u8303\u6700\u540E\u4E00\u5E27\u91CC\u76F8\u8DDD < 30 mm \u7684\u201C\u624B\u90E8\u70B9\u2013\u7269\u4F53\u5173\u952E\u70B9\u201D\u5BF9\uFF0C\u8DDD\u79BB\u53D8\u5316\u7684\u5E73\u5747 \u2264 10 mm\uFF1B\u2461 \u7269\u4F53\u5728\u624B\u8155\u7CFB\u91CC\u7684\u65CB\u8F6C \u2264 15\xB0\uFF08\u9A6C\u514B\u7B14\u53EA\u770B\u957F\u8F74\u504F\u6446\uFF09\uFF1B\u2462 \u7269\u4F53\u5728\u624B\u8155\u7CFB\u91CC\u7684\u4F4D\u7F6E \u2264 25 mm\uFF1B\u2463 \u793A\u8303\u91CC\u63A5\u89E6\u7269\u4F53\uFF08< 10 mm\uFF09\u7684\u6307\u5C16\u79BB\u7269\u4F53\u8868\u9762 \u2264 20 mm\u3002\u672B\u5C3E\u8FDE\u7EED 15 \u4E2A\u6837\u672C\u90FD\u6EE1\u8DB3\u624D\u7B97\u6210\u529F\u3002"
};
function el(tag, attrs = {}, text) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (text !== void 0) e.textContent = text;
  return e;
}
function fmt(v, digits) {
  return Number.isFinite(v) ? v.toFixed(digits) : "\u2014";
}
var mm = (v) => fmt(v * 1e3, 1);
var deg = (v) => fmt(v * 180 / Math.PI, 1);
function isaacLabel(o) {
  if (!o) return "Isaac \u2014";
  if (typeof o.main === "number") {
    const f = o;
    return `Isaac main ${f.main}/${f.n} \xB7 FG-v1 ${f.fg_v1}/${f.n}`;
  }
  const m = o;
  return `Isaac main ${m.main ? "\u2713" : "\u2717"} \xB7 FG-v1 ${m.fg_v1 ? "\u2713" : "\u2717"}`;
}
function startLabel(s, policyId) {
  if (policyId === null) {
    if (s.kind === "frame0") return "frame 0";
    return `\u4E2D\u6BB5 seed ${s.seed} \xB7 #${s.index} \xB7 \u7B2C ${s.start_t} \u884C`;
  }
  const o = s.isaac[policyId] ?? null;
  if (s.kind === "frame0") return `frame 0 \xB7 ${isaacLabel(o)}`;
  return `\u4E2D\u6BB5 seed ${s.seed} \xB7 #${s.index} \xB7 \u7B2C ${s.start_t} \u884C \xB7 ${isaacLabel(o)}`;
}
var FAMILY_ORDER = ["16", "R", "T", "C", "X", "SM", "MO", "U"];
function rowLabel(catalog, row) {
  if (row === null || row === void 0) return "\u6846\u67B6 \u2014";
  const what = catalog.rows?.[String(row)];
  return what ? `\u6846\u67B6 ${row}\uFF08${what}\uFF09` : `\u6846\u67B6 ${row}`;
}
var pct = (v) => v === null || v === void 0 ? "\u2014" : `${v}%`;
var NO_CHECKPOINT = "\u8FD8\u6CA1\u6709 Hand 2.1 checkpoint\uFF1A\u8FD9\u91CC\u53EA\u663E\u793A Hand 2.1 \u573A\u666F\u3001\u6BCF\u6761 reference \u7684\u8D77\u59CB\u6293\u53D6\u548C\u53F3\u4E0B\u89D2\u7684 functional grasp \u5C0F\u56FE\uFF08\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF09\uFF0C\u4E0D\u8FD0\u884C\u7B56\u7565\uFF08\u5B98\u65B9\u7269\u7406\u4E0A\u91CD\u8BAD\u7684\u5B66\u751F\u51FA\u6765\u540E\u81EA\u52A8\u52A0\u5165\uFF09\u3002";
var LEGACY_BANNER = "\u65E7\u7269\u7406\uFF08\u4F5C\u5E9F\uFF09\uFF1AWUJI2 beta1 + Isaac \u8BAD\u7EC3\u7528\u7684\u9A71\u52A8\u53C2\u6570\uFF081 kg \u6307\u5C16\u30012864.8 N\xB7m/rad\uFF09\uFF0C10-19 \u8D77\u5168\u90E8\u4F5C\u5E9F\uFF0C\u53EA\u4F5C\u5BF9\u7167\uFF1B\u9ED8\u8BA4\u9875\u9762\u662F Hand 2.1 \u5B98\u65B9\u7269\u7406\u3002";
var HELD_OFF = "\u6293\u4F4F\u4E86\uFF0C\u4F46\u624B\u2013\u7269\u4F53\u76F8\u5BF9\u4F4D\u7F6E\u6CA1\u8FBE\u5230 functional grasp";
var LOST = "\u7269\u4F53\u6389\u4E86\uFF08\u79BB\u624B\u8155 > 25 cm\uFF0C\u6216\u504F\u79BB reference > 15 cm\uFF09";
var UNSTABLE = "\u6570\u503C\u53D1\u6563\uFF08MuJoCo mjWARN_BADQACC\uFF09";
function mainReason(o, m, counts, hold) {
  const rms = m ? `\u7269\u4F53\u5173\u952E\u70B9\u5728\u624B\u8155\u7CFB RMS ${mm(m.rms)} mm \u2265 12 mm` : "";
  const hand = m ? `\u624B\u90E8\u70B9\u5728\u7269\u4F53\u5750\u6807\u7CFB\u6700\u5927 ${mm(m.handMax)} mm > 13 mm` : "";
  switch (o) {
    case "success":
      return "\u672B\u5C3E\u8FDE\u7EED 15 \u4E2A\u6837\u672C\u90FD\u6EE1\u8DB3";
    case "unstable":
      return UNSTABLE;
    case "lost":
      return LOST;
    case "held_off_rms":
      return `${HELD_OFF}\uFF08main\uFF1A${rms}\uFF09`;
    case "held_off_hand":
      return `${HELD_OFF}\uFF08main\uFF1A${hand}\uFF09`;
    case "held_off_both":
      return `${HELD_OFF}\uFF08main\uFF1A${rms}\uFF1B${hand}\uFF09`;
    case "short_hold":
      return `\u6700\u540E\u6EE1\u8DB3\u4E86 main\uFF0C\u4F46\u53EA\u8FDE\u7EED ${counts} / ${hold} \u4E2A\u6837\u672C`;
  }
}
function fgReason(o, m, counts, hold, axial) {
  switch (o) {
    case "success":
      return "\u672B\u5C3E\u8FDE\u7EED 15 \u4E2A\u6837\u672C\u90FD\u6EE1\u8DB3";
    case "unstable":
      return UNSTABLE;
    case "lost":
      return LOST;
    case "short_hold":
      return `\u6700\u540E\u6EE1\u8DB3\u4E86 FG-v1\uFF0C\u4F46\u53EA\u8FDE\u7EED ${counts} / ${hold} \u4E2A\u6837\u672C`;
    case "off": {
      const f = m?.fgFails;
      const parts = [];
      if (m && f?.near) parts.push(`\u8FD1\u90BB\u5BF9\u8BEF\u5DEE ${mm(m.fgNear)} mm > 10 mm`);
      if (m && f?.rot)
        parts.push(
          `${axial ? "\u957F\u8F74\u504F\u6446" : "\u7269\u4F53\u5728\u624B\u8155\u7CFB\u7684\u65CB\u8F6C"} ${deg(m.fgRot)}\xB0 > 15\xB0`
        );
      if (m && f?.pos)
        parts.push(`\u7269\u4F53\u5728\u624B\u8155\u7CFB\u7684\u4F4D\u7F6E ${mm(m.fgPos)} mm > 25 mm`);
      if (m && f?.contact)
        parts.push(`\u63A5\u89E6\u6307\u5C16\u79BB\u8868\u9762 ${mm(m.fgContact)} mm > 20 mm`);
      return `\u6293\u4F4F\u4E86\uFF0C\u4F46\u624B\u2013\u7269\u4F53\u76F8\u5BF9\u5173\u7CFB\u6CA1\u8FBE\u5230 FG-v1\uFF08${parts.join("\uFF1B")}\uFF09`;
    }
  }
}
var MjwebPanel = class {
  root;
  style;
  cb;
  catalog;
  ep;
  state;
  refSelect;
  startSelect;
  playBtn;
  mouseBtns = /* @__PURE__ */ new Map();
  live = {};
  lastPaint = 0;
  constructor(catalog, ep, state, cb, mouseModes) {
    this.catalog = catalog;
    this.ep = ep;
    this.state = { ...state };
    this.cb = cb;
    document.getElementById(PANEL_ID)?.remove();
    document.getElementById(`${PANEL_ID}-style`)?.remove();
    this.style = el("style", { id: `${PANEL_ID}-style` });
    this.style.textContent = CSS;
    document.head.appendChild(this.style);
    const root = el("div", { id: PANEL_ID, "data-testid": PANEL_ID });
    this.root = root;
    const scene = catalog.scenes[state.sceneId];
    const h21 = ep.hand === "hand21";
    if (h21) {
      root.append(
        el("h1", {}, "Wuji Hand 2.1 \u7075\u5DE7\u624B \xB7 \u5B66\u751F\u7B56\u7565 \xB7 MuJoCo \u5B9E\u65F6\u4EFF\u771F")
      );
      root.append(
        el(
          "div",
          { class: "note", "data-testid": "mjweb-note" },
          "\u8FD9\u662F MuJoCo \u4E2D\u7684\u4EFF\u771F\uFF1A\u5B98\u65B9 Wuji Hand 2.1 \u7269\u7406\uFF1B\u7B56\u7565\u5728 Isaac PhysX \u7684\u540C\u4E00\u5B98\u65B9\u7269\u7406\u91CC\u8BAD\u7EC3\uFF08sim2sim\uFF09"
        )
      );
      root.append(
        el(
          "div",
          { class: "sub", "data-testid": "mjweb-physics" },
          `MuJoCo \u7269\u7406 = Hand 2.1 \u5B98\u65B9\uFF08${ep.wd.physics.tag}\uFF09\uFF1A1/1200 s\uFF0CMJCF \u81EA\u5E26\u7684\u4F4D\u7F6E\u6267\u884C\u5668\uFF0C\u624B\u8155\u5B98\u65B9\u5B9A\u5F8B\uFF08\u590D\u5408\u521A\u4F53\u9690\u5F0F\u963B\u5C3C + \u91CD\u91CF\u524D\u9988\uFF0C\u6BCF 1/120 s\uFF09\uFF0C\u624B\u6307\u9650\u901F`
        )
      );
    } else {
      root.append(
        el("h1", {}, "WUJI2 \u7075\u5DE7\u624B \xB7 \u5B66\u751F\u7B56\u7565 \xB7 MuJoCo \u5B9E\u65F6\u4EFF\u771F\uFF08\u65E7\u7269\u7406\uFF09")
      );
      root.append(
        el(
          "div",
          { class: "banner", "data-testid": "mjweb-legacy" },
          LEGACY_BANNER
        )
      );
      root.append(
        el(
          "div",
          { class: "note", "data-testid": "mjweb-note" },
          "\u8FD9\u662F MuJoCo \u4E2D\u7684\u4EFF\u771F\uFF1B\u7B56\u7565\u5728 Isaac PhysX \u4E2D\u8BAD\u7EC3\uFF08sim2sim\uFF09"
        )
      );
      const wristLabel = {
        implicit: "\u624B\u8155\u4E0E\u624B\u6307\u9690\u5F0F\u8054\u5408\u79EF\u5206",
        isaac: "\u624B\u8155 PD \u663E\u5F0F\uFF08\u6BCF 1/120 s\uFF09",
        substep: "\u624B\u8155 PD \u663E\u5F0F\uFF08\u6BCF\u5B50\u6B65\uFF09"
      };
      root.append(
        el(
          "div",
          { class: "sub", "data-testid": "mjweb-physics" },
          `MuJoCo \u7269\u7406 = \u65E7 CHOSEN\uFF08${ep.wd.physics.tag}\uFF09\uFF1A1/1200 s\uFF0C${wristLabel[ep.env.wristMode] ?? ep.env.wristMode}`
        )
      );
    }
    if (catalog.dry) {
      root.append(
        el(
          "div",
          { class: "dry", "data-testid": "mjweb-dry" },
          "\u5E72\u8DD1\uFF08DRY\uFF09\uFF1A\u8FD9\u91CC\u7684\u7B56\u7565\u90FD\u662F\u65E7\u7269\u7406\u8BAD\u7EC3\u7684\u66FF\u8EAB\uFF0C\u53EA\u7528\u6765\u68C0\u67E5\u7F51\u9875\uFF0C\u4E0D\u662F Hand 2.1 \u7684\u7ED3\u679C"
        )
      );
    }
    const objRow = this.row("\u7269\u4F53");
    for (const o of catalog.objects) {
      const on = o.variants.some((v) => v.scene_id === state.sceneId);
      const b = this.button(
        o.label_zh,
        on,
        () => this.cb.navigate(
          o.variants[0].scene_id,
          this.defaultKey(o.variants[0].scene_id)
        )
      );
      b.setAttribute("data-object", o.key);
      objRow.append(b);
    }
    const obj = catalog.objects.find(
      (o) => o.variants.some((v) => v.scene_id === state.sceneId)
    );
    if (obj && obj.variants.length > 1) {
      const vRow = this.row("\u6570\u636E");
      for (const v of obj.variants) {
        const b = this.button(
          v.label,
          v.scene_id === state.sceneId,
          () => this.cb.navigate(
            v.scene_id,
            this.keyFor(v.scene_id, this.state.policyKey)
          )
        );
        b.setAttribute("data-variant", v.variant);
        vRow.append(b);
      }
    }
    const pRow = this.row("\u7B56\u7565");
    const pols = scene?.policies ?? [];
    if (state.viewer || pols.length === 0) {
      pRow.append(
        el(
          "div",
          { class: "nopolicy", "data-testid": "mjweb-nopolicy" },
          NO_CHECKPOINT
        )
      );
    } else if (h21) {
      const col = el("div", { class: "col" });
      pRow.append(col);
      const byRow = /* @__PURE__ */ new Map();
      for (const p of pols) {
        const k = p.row !== null && p.row !== void 0 ? String(p.row) : p.family ? `f:${p.family}` : "\u2014";
        if (!byRow.has(k)) byRow.set(k, []);
        byRow.get(k).push(p);
      }
      const rank = (k) => k === "\u2014" ? 1e6 : k.startsWith("f:") ? 1e5 + Math.max(0, FAMILY_ORDER.indexOf(k.slice(2))) : 0;
      const groups = [...byRow.entries()].sort((a, b) => rank(a[0]) - rank(b[0]));
      for (const [row, ps] of groups) {
        const fam = row.startsWith("f:") ? row.slice(2) : null;
        const line = el("div", { class: "prow", "data-row": row });
        line.append(
          el(
            "span",
            {
              class: "rl",
              title: fam ? catalog.families?.[fam] ?? fam : rowLabel(catalog, ps[0].row)
            },
            fam ? fam === "16" ? "#16" : fam : row === "\u2014" ? "\u5176\u4ED6" : `\u6846\u67B6 ${row}`
          )
        );
        const many = new Set(ps.map((p) => p.seed)).size !== ps.length;
        for (const p of ps) {
          const base = fam && p.fair4_label ? p.fair4_label : `seed ${p.seed}${many ? ` \xB7 it ${p.iteration}` : ""}`;
          const text = `${base}${p.default ? "\uFF08\u9ED8\u8BA4\uFF09" : ""}`;
          const b = this.button(
            text,
            p.key === state.policyKey,
            () => this.cb.navigate(state.sceneId, p.key)
          );
          b.setAttribute("data-seed", String(p.seed));
          b.setAttribute("data-policy", p.key);
          b.setAttribute("data-row", row);
          b.title = `${p.checkpoint_name ?? p.policy_id}`;
          line.append(b);
        }
        col.append(line);
      }
    } else {
      for (const p of pols) {
        const b = this.button(
          `seed ${p.seed}${p.default ? "\uFF08\u9ED8\u8BA4\uFF09" : ""}`,
          p.key === state.policyKey,
          () => this.cb.navigate(state.sceneId, p.key)
        );
        b.setAttribute("data-seed", String(p.seed));
        b.setAttribute("data-policy", p.key);
        pRow.append(b);
      }
    }
    const pol = state.viewer ? void 0 : pols.find((p) => p.key === state.policyKey);
    if (pol && h21) {
      root.append(
        el(
          "div",
          {
            class: "sub",
            "data-testid": "mjweb-policy",
            title: "Isaac \u7684\u6570\u5B57\uFF1A\u540C\u4E00 checkpoint \u5728 Isaac \u5B98\u65B9\u7269\u7406\u91CC\u3001eval seed 1\u20133 \u7684\u7ED3\u679C\uFF08main / FG-v1\uFF09"
          },
          `${rowLabel(catalog, pol.row)} \xB7 ${pol.checkpoint_name ?? pol.policy_id} \xB7 Isaac main\uFF1Aframe\xA00 ${pct(pol.isaac_frame0_pct)} \xB7 \u4E2D\u6BB5 ${pct(pol.isaac_mid_pct)}\uFF1BFG-v1\uFF1A${pct(pol.isaac_frame0_fg_pct)} \xB7 ${pct(pol.isaac_mid_fg_pct)}`
        )
      );
    } else if (pol) {
      root.append(
        el(
          "div",
          {
            class: "sub",
            "data-testid": "mjweb-policy",
            title: "\u6846\u67B6 8 = x3000 student-of-students\uFF1BIsaac \u6210\u529F\u7387\u6309 main \u5224\u636E\uFF08\u65E7\u7269\u7406\uFF09"
          },
          `\u6846\u67B6 8 \xB7 it ${pol.iteration} \xB7 Isaac main\uFF08\u65E7\u7269\u7406\uFF09\uFF1Aframe\xA00 ${pct(pol.isaac_frame0_pct)} \xB7 \u4E2D\u6BB5 ${pct(pol.isaac_mid_pct)}`
        )
      );
    }
    const rRow = this.row("\u53C2\u8003");
    this.refSelect = el("select", { "data-testid": "mjweb-ref" });
    ep.wd.references.forEach((r, m) => {
      this.refSelect.append(
        el("option", { value: String(m) }, `${r.case}\uFF08${r.motion_steps} \u6B65\uFF09`)
      );
    });
    this.refSelect.addEventListener("change", () => {
      const m = Number(this.refSelect.value);
      const first = ep.wd.starts.findIndex(
        (s) => s.motion === m && s.kind === "frame0"
      );
      this.fillStarts(m);
      if (first >= 0) this.cb.selectStart(first);
    });
    rRow.append(this.refSelect);
    const sRow = this.row("\u8D77\u59CB");
    this.startSelect = el("select", { "data-testid": "mjweb-start" });
    this.startSelect.addEventListener(
      "change",
      () => this.cb.selectStart(Number(this.startSelect.value))
    );
    sRow.append(this.startSelect);
    const tRow = this.row("");
    const pert = this.checkbox(
      "\u6270\u52A8",
      state.perturb,
      (on) => {
        this.state.perturb = on;
        this.cb.setPerturb(on, this.state.seed);
      },
      "mjweb-perturb"
    );
    const seedIn = el("input", {
      type: "number",
      value: String(state.seed),
      step: "1",
      "data-testid": "mjweb-seed",
      title: "\u968F\u673A\u79CD\u5B50\uFF08\u6270\u52A8\u4E0E\u4F20\u611F\u566A\u58F0\uFF09"
    });
    seedIn.addEventListener("change", () => {
      const v = Math.trunc(Number(seedIn.value)) || 0;
      seedIn.value = String(v);
      this.state.seed = v;
      this.cb.setPerturb(this.state.perturb, v);
    });
    tRow.append(pert, el("span", { class: "sub" }, "\u79CD\u5B50"), seedIn);
    const sense = this.checkbox(
      "\u4F20\u611F\u566A\u58F0\uFF08SENSE\uFF09",
      state.sense,
      (on) => {
        this.state.sense = on;
        this.cb.setSense(on);
      },
      "mjweb-sense"
    );
    tRow.append(sense);
    pert.title = "\u6270\u52A8\uFF1A\u624B+\u7269\u4F53\u7ED5\u624B\u8155\u521A\u4F53\u53D8\u6362 \xB12 cm / \xB10.15 rad\uFF0C\u624B\u6307 \xB10.05 rad\uFF08PSTART \u5F0F\uFF0C\u5E26\u79CD\u5B50\uFF09";
    sense.title = "SENSE = Isaac \u8BC4\u6D4B\u534F\u8BAE\u7684\u4F20\u611F\u566A\u58F0\uFF1A\xB12 mm / rot6d \xB10.02 / \u5173\u8282 \xB10.02 rad\uFF0C\u5EF6\u8FDF 0\u20132 \u6B65";
    const gRow = this.row("\u663E\u793A");
    const gCol = el("div", { class: "col" });
    gRow.append(gCol);
    const vis = () => this.cb.setVisibility({
      reference: this.state.reference,
      inset: this.state.inset
    });
    const insetRow = ep.wd.references.some((r) => !!r.fg_inset) ? [
      [
        "inset",
        "functional grasp \u5C0F\u56FE\uFF1A\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF08\u53F3\u4E0B\u89D2\uFF09",
        "mjweb-fginset",
        5921370,
        TIP.inset
      ]
    ] : [];
    const overlays = [
      ...insetRow,
      [
        "reference",
        "\u53C2\u8003\u6B8B\u5F71\uFF1Areference \u56DE\u653E",
        "mjweb-ghost",
        GHOST_COLORS.reference,
        TIP.ghost
      ]
    ];
    for (const [key, text, testId, color, tip] of overlays) {
      const c = this.checkbox(
        text,
        state[key],
        (on) => {
          this.state[key] = on;
          vis();
        },
        testId,
        color
      );
      c.title = tip;
      gCol.append(c);
    }
    const cRow = this.row("\u63A7\u5236");
    this.playBtn = this.button("\u23F8 \u6682\u505C", false, () => this.cb.playPause());
    this.playBtn.setAttribute("data-testid", "mjweb-play");
    if (state.viewer) {
      this.playBtn.disabled = true;
      this.playBtn.title = "\u6CA1\u6709\u7B56\u7565\u53EF\u8FD0\u884C";
    }
    const resetBtn = this.button("\u21BA \u91CD\u7F6E", false, () => this.cb.reset());
    resetBtn.setAttribute("data-testid", "mjweb-reset");
    cRow.append(this.playBtn, resetBtn);
    const mRow = this.row("\u9F20\u6807");
    mRow.title = "\u62D6\u52A8\u89C6\u89D2\uFF1A\u5DE6\u952E\u65CB\u8F6C / \u53F3\u952E\u5E73\u79FB / \u6EDA\u8F6E\u7F29\u653E\uFF1B\u201C\u62C9/\u63A8\u201D\u6A21\u5F0F\u4E0B\u5DE6\u952E\u4F5C\u7528\u4E8E\u624B\u6216\u7269\u4F53\u3002\u6309 C \u663E\u793A mjswan \u539F\u751F\u9762\u677F\u3002";
    const labels = {
      view: "\u89C6\u89D2",
      pull: "\u62C9\uFF08Pull\uFF09",
      push: "\u63A8\uFF08Push\uFF09"
    };
    for (const m of mouseModes) {
      if (!(m.id in labels)) continue;
      const b = this.button(labels[m.id], m.id === state.mouseMode, () => {
        this.state.mouseMode = m.id;
        for (const [id2, btn] of this.mouseBtns)
          btn.classList.toggle("on", id2 === m.id);
        this.cb.setMouseMode(m.id);
      });
      b.disabled = !m.available;
      b.setAttribute("data-mouse", m.id);
      this.mouseBtns.set(m.id, b);
      mRow.append(b);
    }
    mRow.append(el("span", { class: "sub" }, "\u6309 C\uFF1Amjswan \u9762\u677F"));
    const live = el("div", { class: "live", "data-testid": "mjweb-live" });
    live.append(
      el(
        "div",
        { class: "sub", "data-testid": "mjweb-criterion" },
        "\u5224\u636E\u53EA\u770B\u624B\u548C\u7269\u4F53\u7684\u76F8\u5BF9\u5173\u7CFB\uFF08functional grasp\uFF09\uFF1A\u5BF9\u6BD4\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF0C\u672B\u5C3E\u8FDE\u7EED 15 \u4E2A\u6837\u672C\u90FD\u6EE1\u8DB3\u624D\u7B97\u6210\u529F\uFF1B\u4E0D\u8981\u6C42\u7269\u4F53\u6216\u624B\u5728\u4E16\u754C\u91CC\u7684\u4F4D\u7F6E\u3002"
      )
    );
    const rows = [
      ["progress", {}],
      ["rate", {}],
      ["rel", { "data-testid": "mjweb-rel", title: TIP.rel }],
      ["main", { "data-testid": "mjweb-main", title: TIP.main }],
      ["fg", { "data-testid": "mjweb-fg", title: TIP.fg }],
      ["lost", { "data-testid": "mjweb-lost", class: "bad" }],
      ["result", { class: "result", "data-testid": "mjweb-result" }]
    ];
    for (const [key, attrs] of rows) {
      this.live[key] = el("div", attrs);
      live.append(this.live[key]);
    }
    root.append(live);
    document.body.appendChild(root);
    this.refresh();
  }
  defaultKey(sceneId) {
    const s = this.catalog.scenes[sceneId];
    return s?.policies.find((p) => p.default)?.key ?? s?.policies[0]?.key ?? s?.viewer_key ?? "none";
  }
  /** `key` if the scene has that policy, else the scene's default. */
  keyFor(sceneId, key) {
    return this.catalog.scenes[sceneId]?.policies.some((p) => p.key === key) ? key : this.defaultKey(sceneId);
  }
  row(label) {
    const r = el("div", { class: "row" });
    r.append(el("span", { class: "lab" }, label));
    this.root.append(r);
    return r;
  }
  button(text, on, onClick) {
    const b = el("button", { type: "button" }, text);
    if (on) b.classList.add("on");
    b.addEventListener("click", (e) => {
      e.stopPropagation();
      onClick();
    });
    return b;
  }
  checkbox(text, checked, onChange, testId, color) {
    const l = el("label");
    const c = el("input", { type: "checkbox", "data-testid": testId });
    c.checked = checked;
    c.addEventListener("change", () => onChange(c.checked));
    l.append(c);
    if (color !== void 0) {
      const sw = el("span", { class: "swatch" });
      sw.style.background = `#${color.toString(16).padStart(6, "0")}`;
      l.append(sw);
    }
    l.append(document.createTextNode(text));
    return l;
  }
  fillStarts(motion) {
    const ep = this.ep;
    this.startSelect.replaceChildren();
    const idx = ep.wd.starts.map((s, i) => ({ s, i })).filter(({ s }) => s.motion === motion);
    idx.sort(
      (a, b) => a.s.kind === b.s.kind ? (a.s.seed ?? 0) - (b.s.seed ?? 0) || a.s.index - b.s.index : a.s.kind === "frame0" ? -1 : 1
    );
    for (const { s, i } of idx)
      this.startSelect.append(
        el(
          "option",
          { value: String(i) },
          startLabel(s, ep.policy?.id ?? null)
        )
      );
  }
  /** Redraw the selectors from the episode (after a start change or reset). */
  refresh() {
    const ep = this.ep;
    const m = ep.start.motion;
    if (this.refSelect.value !== String(m)) this.refSelect.value = String(m);
    this.fillStarts(m);
    this.startSelect.value = String(ep.startIndex);
  }
  setPaused(paused) {
    this.playBtn.textContent = this.state.viewer ? "\u25B6 \u64AD\u653E\uFF08\u65E0\u7B56\u7565\uFF09" : paused ? "\u25B6 \u64AD\u653E" : "\u23F8 \u6682\u505C";
  }
  /** Live numbers; repaints at most 10 times a second unless forced. */
  update(stats, force = false) {
    const now = performance.now();
    if (!force && now - this.lastPaint < 100) return;
    this.lastPaint = now;
    this.setPaused(stats.paused);
    const ep = this.ep;
    const m = this.state.viewer ? null : ep.last;
    const crit = ep.wd.criterion;
    const hold = crit.hold_samples;
    const axial = ep.wd.fg.axial;
    const simT = ep.samples / 30;
    this.live.rate.style.display = this.state.viewer ? "none" : "";
    if (this.state.viewer) {
      this.live.progress.textContent = `${ep.ref.case}\uFF1A${ep.ref.motion_steps} \u6B65\uFF08${(ep.ref.motion_steps / 30).toFixed(1)} s\uFF09\xB7 \u8D77\u59CB\u7B2C ${ep.start.start_t} \u884C`;
    } else
      this.live.progress.textContent = `\u8FDB\u5EA6 ${ep.samples} / ${ep.total} \u63A7\u5236\u6B65 \xB7 \u4EFF\u771F ${simT.toFixed(2)} s` + (ep.perturbation ? " \xB7 \u5DF2\u6270\u52A8" : "") + (ep.view.noise.lag > 0 ? ` \xB7 SENSE \u5EF6\u8FDF ${ep.view.latency} \u6B65` : "");
    this.live.rate.textContent = `\u6BCF\u79D2\u63A7\u5236\u6B65 ${fmt(stats.controlRate, 1)}\uFF08\u5B9E\u65F6 ${fmt(stats.realtime, 2)}x${stats.unpaced ? "\uFF0C\u4E0D\u9650\u901F" : ""}\uFF09 \xB7 \u7269\u7406 ${fmt(stats.physicsMs, 1)} ms/\u6B65 \xB7 \u63A8\u7406 ${fmt(stats.inferMs, 1)} ms`;
    const span = (text, ok) => el("span", { class: ok ? "ok" : "bad" }, text);
    const dim = (text) => el("span", { class: "dim" }, text);
    const fgTier = crit.fg_tiers.fg_v1;
    if (m) {
      const axialText = axial ? ` \xB7 \u957F\u8F74\u504F\u6446 ${deg(m.axialErr)}\xB0` : "";
      this.live.rel.textContent = `\u7269\u4F53\u5728\u624B\u8155\u7CFB\u91CC\uFF08\u5BF9\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF09\uFF1A\u4F4D\u7F6E ${mm(m.posErr)} mm \xB7 \u65CB\u8F6C ${deg(m.rotErr)}\xB0${axialText}`;
      const rmsOk = m.rms < crit.object_rms_m;
      const handOk = m.handMax <= crit.hand_max_m;
      this.live.main.replaceChildren(
        dim("main\uFF1A"),
        span(`\u7269\u4F53\u5173\u952E\u70B9\u5728\u624B\u8155\u7CFB\u91CC RMS ${mm(m.rms)} mm`, rmsOk),
        dim(`\uFF08\u8981 < ${crit.object_rms_m * 1e3}\uFF09\xB7 `),
        span(`\u624B\u90E8 26 \u70B9\u5728\u7269\u4F53\u7CFB\u91CC\u6700\u5927 ${mm(m.handMax)} mm`, handOk),
        dim(`\uFF08main \u8981 \u2264 ${crit.hand_max_m * 1e3} mm\uFF09\u2192 `),
        span(m.passed ? "\u672C\u6B65 \u2713" : "\u672C\u6B65 \u2717", m.passed),
        dim(` \xB7 \u8FDE\u7EED ${Math.min(ep.score.counts, hold)}/${hold}`)
      );
      const f = m.fgFails;
      const contact = ep.ref.goal.fg.contact.some(Boolean) ? [
        span(`\u63A5\u89E6\u6307\u5C16\u79BB\u8868\u9762 ${mm(m.fgContact)} mm`, !f.contact),
        dim(`\uFF08\u2264 ${fgTier[3] * 1e3}\uFF09\u2192 `)
      ] : [dim("\u63A5\u89E6\u6307\u5C16\uFF1A\u793A\u8303\u6700\u540E\u4E00\u5E27\u6CA1\u6709\u8D34\u7740\u7269\u4F53\u7684\u6307\u5C16\uFF08\u4E0D\u68C0\u67E5\uFF09\u2192 ")];
      this.live.fg.replaceChildren(
        dim("FG-v1\uFF1A"),
        span(`\u8FD1\u90BB\u5BF9\u8BEF\u5DEE ${mm(m.fgNear)} mm`, !f.near),
        dim(`\uFF08\u2264 ${fgTier[0] * 1e3}\uFF09\xB7 `),
        span(`${axial ? "\u957F\u8F74\u504F\u6446" : "\u65CB\u8F6C"} ${deg(m.fgRot)}\xB0`, !f.rot),
        dim(`\uFF08\u2264 ${Math.round(fgTier[1] * 180 / Math.PI)}\uFF09\xB7 `),
        span(`\u4F4D\u7F6E ${mm(m.fgPos)} mm`, !f.pos),
        dim(`\uFF08\u2264 ${fgTier[2] * 1e3}\uFF09\xB7 `),
        ...contact,
        span(m.fgPassed ? "\u672C\u6B65 \u2713" : "\u672C\u6B65 \u2717", m.fgPassed),
        dim(` \xB7 \u8FDE\u7EED ${Math.min(ep.score.fgCounts, hold)}/${hold}`)
      );
    } else {
      const s0 = this.state.viewer ? ep.startMetrics : null;
      this.live.rel.textContent = s0 ? `\u8D77\u59CB\u65F6\u7269\u4F53\u5728\u624B\u8155\u7CFB\u91CC\uFF08\u5BF9\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF09\uFF1A\u4F4D\u7F6E ${mm(s0.posErr)} mm \xB7 \u65CB\u8F6C ${deg(s0.rotErr)}\xB0${axial ? ` \xB7 \u957F\u8F74\u504F\u6446 ${deg(s0.axialErr)}\xB0` : ""}` : "\u7269\u4F53\u5728\u624B\u8155\u7CFB\u91CC\uFF08\u5BF9\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF09\uFF1A\u2014";
      this.live.main.textContent = `main\uFF1A\u7269\u4F53\u5173\u952E\u70B9\u5728\u624B\u8155\u7CFB\u91CC RMS < ${crit.object_rms_m * 1e3} mm\uFF0C\u4E14\u624B\u90E8 26 \u70B9\u5728\u7269\u4F53\u7CFB\u91CC\u6700\u5927 \u2264 ${crit.hand_max_m * 1e3} mm`;
      this.live.fg.textContent = `FG-v1\uFF1A\u8FD1\u90BB\u5BF9\u8BEF\u5DEE \u2264 ${fgTier[0] * 1e3} mm \xB7 \u65CB\u8F6C \u2264 ${Math.round(fgTier[1] * 180 / Math.PI)}\xB0 \xB7 \u4F4D\u7F6E \u2264 ${fgTier[2] * 1e3} mm \xB7 \u63A5\u89E6\u6307\u5C16\u79BB\u8868\u9762 \u2264 ${fgTier[3] * 1e3} mm`;
    }
    const lost = ep.score.firstLost;
    this.live.lost.textContent = lost >= 0 ? `\u7269\u4F53\u6389\u4E86\uFF08\u4ECE\u7B2C ${lost + 1} \u4E2A\u6837\u672C\u8D77\uFF1A\u79BB\u624B\u8155 > 25 cm\uFF0C\u6216\u504F\u79BB reference > 15 cm\uFF09` : "";
    this.live.lost.style.display = lost >= 0 ? "" : "none";
    const isaac = isaacLabel(ep.isaacOutcome());
    this.live.result.replaceChildren();
    const line = (...parts) => {
      const d = el("div");
      d.append(...parts);
      this.live.result.append(d);
    };
    const mo = ep.mainOutcome;
    const fo = ep.fgOutcome;
    if (this.state.viewer) {
      line(
        dim(
          "MuJoCo\uFF1A\u6CA1\u6709\u7B56\u7565\uFF08\u8FD8\u6CA1\u6709 Hand 2.1 checkpoint\uFF09\uFF0C\u4E0D\u8FD0\u884C\uFF1B\u53F3\u4E0B\u89D2\u7684\u5C0F\u56FE\u662F\u8FD9\u6761 reference \u793A\u8303\u6700\u540E\u4E00\u5E27\u7684 functional grasp"
        )
      );
      return;
    }
    if (ep.done && ep.inferenceError) {
      line(span(`MuJoCo\uFF1A\u7B56\u7565\u63A8\u7406\u5931\u8D25\uFF08ONNX\uFF09\uFF1A${ep.inferenceError}`, false));
    } else if (ep.done && mo === "unstable") {
      line(
        span(
          `MuJoCo\uFF1A${ep.unstableReason ?? UNSTABLE}\uFF0C\u7B2C ${ep.unstableSample} \u4E2A\u6837\u672C\u540E episode \u7ED3\u675F\uFF08main\u3001FG-v1 \u90FD\u7B97\u5931\u8D25\uFF09`,
          false
        )
      );
    } else if (ep.done && mo && fo) {
      line(
        span(`MuJoCo main\uFF1A${ep.success ? "\u6210\u529F \u2713" : "\u5931\u8D25 \u2717"}`, ep.success),
        dim(` \u2014 ${mainReason(mo, m, ep.score.counts, hold)}`)
      );
      line(
        span(
          `MuJoCo FG-v1\uFF1A${ep.fgSuccess ? "\u6210\u529F \u2713" : "\u5931\u8D25 \u2717"}`,
          ep.fgSuccess
        ),
        dim(` \u2014 ${fgReason(fo, m, ep.score.fgCounts, hold, axial)}`)
      );
      if (ep.unstable)
        line(
          dim(
            `\uFF08\u7269\u4F53\u6389\u4E86\u4EE5\u540E\uFF0C\u7B2C ${ep.unstableSample} \u4E2A\u6837\u672C\u540E\u53C8${ep.unstableReason ?? "\u6570\u503C\u53D1\u6563"}\uFF09`
          )
        );
    } else {
      line(dim("MuJoCo\uFF1A\u8FDB\u884C\u4E2D"));
    }
    line(dim(`\u540C\u4E00\u8D77\u59CB\uFF1A${isaac}`));
  }
  dispose() {
    this.root.remove();
    this.style.remove();
  }
};

// ../../../../../../web/plugins/arm_ik.ts
function solveN(aIn, bIn, n) {
  const a = Array.from(aIn);
  const b = Array.from(bIn);
  const piv = new Array(n);
  for (let k = 0; k < n; k++) {
    let p = k;
    for (let i = k + 1; i < n; i++)
      if (Math.abs(a[n * i + k]) > Math.abs(a[n * p + k])) p = i;
    piv[k] = p;
    if (p !== k)
      for (let j = 0; j < n; j++) {
        const t = a[n * k + j];
        a[n * k + j] = a[n * p + j];
        a[n * p + j] = t;
      }
    const d = a[n * k + k];
    if (d === 0) return new Array(n).fill(NaN);
    const inv = 1 / d;
    for (let i = k + 1; i < n; i++) a[n * i + k] *= inv;
    for (let i = k + 1; i < n; i++) {
      const l = a[n * i + k];
      for (let j = k + 1; j < n; j++) a[n * i + j] -= l * a[n * k + j];
    }
  }
  for (let k = 0; k < n; k++) {
    const p = piv[k];
    if (p !== k) {
      const t = b[k];
      b[k] = b[p];
      b[p] = t;
    }
  }
  for (let j = 0; j < n; j++)
    for (let i = j + 1; i < n; i++) b[i] -= b[j] * a[n * i + j];
  for (let k = n - 1; k >= 0; k--) {
    b[k] /= a[n * k + k];
    for (let i = 0; i < k; i++) b[i] -= b[k] * a[n * i + k];
  }
  return b;
}
function accelBox(h, c, lo, hi, vPrev, amax, dt, braking, u) {
  const n = lo.length;
  const alo = Array.from(vPrev, (v, i) => (v - amax[i] * dt) * dt);
  const ahi = Array.from(vPrev, (v, i) => (v + amax[i] * dt) * dt);
  const braked = new Array(n).fill(false);
  if (braking) {
    const g = u ? Array.from(u) : new Array(n).fill(0);
    const r = solveN(
      h,
      Array.from(c, (v) => -v),
      n
    ).map((x, i) => x - g[i] * dt);
    for (let i = 0; i < n; i++) {
      const half = amax[i] * dt / 2;
      const vcap = -half + Math.sqrt(half * half + 2 * amax[i] * Math.abs(r[i]));
      const blo = r[i] < 0 ? (g[i] - vcap) * dt : -Infinity;
      const bhi = r[i] > 0 ? (g[i] + vcap) * dt : Infinity;
      const l2 = Math.max(alo[i], blo);
      const h2 = Math.min(ahi[i], bhi);
      if (l2 <= h2) {
        braked[i] = l2 > alo[i] || h2 < ahi[i];
        alo[i] = l2;
        ahi[i] = h2;
      }
    }
  }
  const outLo = new Array(n);
  const outHi = new Array(n);
  const relaxed = new Array(n).fill(false);
  for (let i = 0; i < n; i++) {
    const l3 = Math.max(lo[i], alo[i]);
    const h3 = Math.min(hi[i], ahi[i]);
    if (l3 > h3) {
      relaxed[i] = true;
      outLo[i] = outHi[i] = ahi[i] < lo[i] ? lo[i] : hi[i];
    } else {
      outLo[i] = l3;
      outHi[i] = h3;
    }
  }
  return { lo: outLo, hi: outHi, relaxed, braked };
}
function boxQp(h, c, lo, hi, iters = 50) {
  const n = c.length;
  const clip = (v, i) => v < lo[i] ? lo[i] : v > hi[i] ? hi[i] : v;
  let x = solveN(
    h,
    Array.from(c, (v) => -v),
    n
  ).map(clip);
  const atLo = x.map((v, i) => v <= lo[i]);
  const atHi = x.map((v, i) => v >= hi[i]);
  for (let it = 0; it < iters; it++) {
    const fixed = atLo.map((v, i) => v || atHi[i]);
    const free = [];
    const fix = [];
    for (let i = 0; i < n; i++) (fixed[i] ? fix : free).push(i);
    const cand = x.slice();
    if (free.length) {
      const m = free.length;
      const a = new Array(m * m);
      const rhs = new Array(m);
      for (let r = 0; r < m; r++) {
        let s = -c[free[r]];
        for (const j2 of fix) s -= h[n * free[r] + j2] * x[j2];
        rhs[r] = s;
        for (let k = 0; k < m; k++) a[m * r + k] = h[n * free[r] + free[k]];
      }
      const sol = solveN(a, rhs, m);
      free.forEach((i, r) => cand[i] = sol[r]);
    }
    const viol = cand.map((v, i) => !fixed[i] && (v < lo[i] || v > hi[i]));
    if (viol.some((v) => v)) {
      const d = cand.map((v, i) => v - x[i]);
      let best = -1;
      let tBest = Infinity;
      for (let i = 0; i < n; i++) {
        if (!viol[i]) continue;
        const t2 = d[i] < 0 ? (lo[i] - x[i]) / d[i] : (hi[i] - x[i]) / d[i];
        if (t2 < tBest) {
          tBest = t2;
          best = i;
        }
      }
      const t = Math.min(Math.max(tBest, 0), 1);
      x = x.map((v, i) => clip(v + t * d[i], i));
      if (d[best] < 0) {
        x[best] = lo[best];
        atLo[best] = true;
      } else {
        x[best] = hi[best];
        atHi[best] = true;
      }
      continue;
    }
    x = cand;
    let j = -1;
    let mult = Infinity;
    for (let i = 0; i < n; i++) {
      let g = c[i];
      for (let k = 0; k < n; k++) g += h[n * i + k] * x[k];
      const mu = atLo[i] ? g : atHi[i] ? -g : Infinity;
      if (mu < mult) {
        mult = mu;
        j = i;
      }
    }
    if (mult >= -1e-12) return x;
    atLo[j] = false;
    atHi[j] = false;
  }
  return x;
}
function mat33ToQuat(r) {
  const tr = r[0] + r[4] + r[8];
  let q;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q = [0.25 * s, (r[7] - r[5]) / s, (r[2] - r[6]) / s, (r[3] - r[1]) / s];
  } else if (r[0] > r[4] && r[0] > r[8]) {
    const s = Math.sqrt(1 + r[0] - r[4] - r[8]) * 2;
    q = [(r[7] - r[5]) / s, 0.25 * s, (r[1] + r[3]) / s, (r[2] + r[6]) / s];
  } else if (r[4] > r[8]) {
    const s = Math.sqrt(1 + r[4] - r[0] - r[8]) * 2;
    q = [(r[2] - r[6]) / s, (r[1] + r[3]) / s, 0.25 * s, (r[5] + r[7]) / s];
  } else {
    const s = Math.sqrt(1 + r[8] - r[0] - r[4]) * 2;
    q = [(r[3] - r[1]) / s, (r[2] + r[6]) / s, (r[5] + r[7]) / s, 0.25 * s];
  }
  const n = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return q.map((v) => v / n);
}
function qmul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
  ];
}
function qrot(v, q) {
  const [w, x, y, z] = [q[0], q[1], q[2], q[3]];
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx)
  ];
}
function skew(v) {
  return [0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0];
}
function mm3(a, b) {
  const o = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      o[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
  return o;
}
function tr3(a) {
  return [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]];
}
function so3Log(qIn) {
  let q = [qIn[0], qIn[1], qIn[2], qIn[3]];
  if (q[0] < 0) q = q.map((v) => -v);
  const n = Math.sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n === 0) return [0, 0, 0];
  const angle = 2 * Math.atan2(n, q[0]);
  return [angle * q[1] / n, angle * q[2] / n, angle * q[3] / n];
}
function so3Ljacinv(w) {
  const t2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
  let beta;
  if (t2 < 0.01)
    beta = 1 / 12 * (1 + t2 / 60 * (1 + t2 / 42 * (1 + t2 / 40)));
  else {
    const half = 0.5 * Math.sqrt(t2);
    beta = (1 - half / Math.tan(half)) / t2;
  }
  const out = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      out[3 * i + j] = beta * (w[i] * w[j] - (i === j ? t2 : 0));
  const a = [-0.5 * w[0], -0.5 * w[1], -0.5 * w[2]];
  out[1] += -a[2];
  out[2] += a[1];
  out[3] += a[2];
  out[5] += -a[0];
  out[6] += -a[1];
  out[7] += a[0];
  out[0] += 1;
  out[4] += 1;
  out[8] += 1;
  return out;
}
function getQ(c) {
  const v = [c[0], c[1], c[2]];
  const w = [c[3], c[4], c[5]];
  const t2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
  const theta = Math.sqrt(t2);
  let B, C, D;
  if (t2 < 0.01) {
    B = 1 / 6 * (1 - t2 / 20 * (1 - t2 / 42 * (1 - t2 / 72)));
    C = -(1 / 24) * (1 - t2 / 30 * (1 - t2 / 56 * (1 - t2 / 90)));
    D = 1 / 120 * (1 - t2 / 21 * (1 - t2 / 48 * (1 - 2 * t2 / 165)));
  } else {
    const t4 = t2 * t2;
    const st = Math.sin(theta);
    const ct = Math.cos(theta);
    B = (theta - st) / (t2 * theta);
    C = (1 - 0.5 * t2 - ct) / t4;
    D = (2 * theta - 3 * st + theta * ct) / (2 * t4 * theta);
  }
  const V = skew(v);
  const W = skew(w);
  const VW = mm3(V, W);
  const WV = tr3(VW);
  const WVW = mm3(WV, W);
  const VWW = mm3(VW, W);
  const VWWT = tr3(VWW);
  const WVWW = mm3(WVW, W);
  const WWVW = mm3(W, WVW);
  const Q = new Array(9);
  for (let i = 0; i < 9; i++)
    Q[i] = 0.5 * V[i] + B * (WV[i] + VW[i] + WVW[i]) - C * (VWW[i] - VWWT[i] - 3 * WVW[i]) + D * (WVWW[i] + WWVW[i]);
  return Q;
}
function frameTaskTerms(target, frame) {
  const invQ = [target[0], -target[1], -target[2], -target[3]];
  const invT = qrot([-target[4], -target[5], -target[6]], invQ);
  const eq = qmul(invQ, [frame[0], frame[1], frame[2], frame[3]]);
  const et = qrot([frame[4], frame[5], frame[6]], invQ).map(
    (v, i) => v + invT[i]
  );
  const w = so3Log(eq);
  const vinv = so3Ljacinv(w);
  const tau = [
    vinv[0] * et[0] + vinv[1] * et[1] + vinv[2] * et[2],
    vinv[3] * et[0] + vinv[4] * et[1] + vinv[5] * et[2],
    vinv[6] * et[0] + vinv[7] * et[1] + vinv[8] * et[2],
    w[0],
    w[1],
    w[2]
  ];
  const neg = tau.map((v) => -v);
  const lji = so3Ljacinv([neg[3], neg[4], neg[5]]);
  const QJ = mm3(getQ(neg), lji);
  const JQJ = mm3(lji, QJ);
  const jlog = new Array(36).fill(0);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      jlog[6 * i + j] = lji[3 * i + j];
      jlog[6 * (i + 3) + j + 3] = lji[3 * i + j];
      jlog[6 * i + j + 3] = -JQJ[3 * i + j];
    }
  const [qw, qx, qy, qz] = [frame[0], frame[1], frame[2], frame[3]];
  const R = [
    1 - 2 * (qy * qy + qz * qz),
    2 * (qx * qy - qz * qw),
    2 * (qx * qz + qy * qw),
    2 * (qx * qy + qz * qw),
    1 - 2 * (qx * qx + qz * qz),
    2 * (qy * qz - qx * qw),
    2 * (qx * qz - qy * qw),
    2 * (qy * qz + qx * qw),
    1 - 2 * (qx * qx + qy * qy)
  ];
  const M = new Array(36);
  for (let i = 0; i < 6; i++)
    for (let b = 0; b < 6; b += 3)
      for (let j = 0; j < 3; j++) {
        const row = 6 * i + b;
        M[6 * i + b + j] = -(jlog[row] * R[3 * j] + jlog[row + 1] * R[3 * j + 1] + jlog[row + 2] * R[3 * j + 2]);
      }
  return { err: neg, M };
}
var ArmIk = class {
  cfg;
  meta;
  mujoco;
  model;
  data;
  vmax;
  lower;
  upper;
  q = [0, 0, 0, 0, 0, 0];
  qPosture = [0, 0, 0, 0, 0, 0];
  /** The previous step's velocity: the acceleration box's centre (`ArmIK.v_prev`). */
  vPrev = [0, 0, 0, 0, 0, 0];
  /** Steps that were not finite (the commanded configuration stays): a guard, the episode ends. */
  fallbacks = 0;
  /** Joint-steps whose acceleration box gave way to the position / velocity box (`ArmIK.accel_relaxed`). */
  accelRelaxed = 0;
  /** The last step was at the acceleration bound / had a speed capped by braking. */
  accelBound = false;
  braked = false;
  amax;
  jacp;
  jacr;
  constructor(mujoco, model, meta) {
    this.mujoco = mujoco;
    this.model = model;
    this.meta = meta;
    this.cfg = meta.ik;
    this.data = new mujoco.MjData(model);
    this.vmax = meta.vel_limits.map((v) => this.cfg.speed_fraction * v);
    this.amax = this.cfg.accel_limits ? this.cfg.accel_limits.slice() : null;
    const range = model.jnt_range;
    const jid = (n) => mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, n);
    this.lower = meta.joints.map(
      (n) => range[2 * jid(n)] + this.cfg.min_distance_from_limits
    );
    this.upper = meta.joints.map(
      (n) => range[2 * jid(n) + 1] - this.cfg.min_distance_from_limits
    );
    const M = mujoco;
    this.jacp = new M.DoubleBuffer(3 * model.nv);
    this.jacr = new M.DoubleBuffer(3 * model.nv);
  }
  dispose() {
    this.jacp.delete();
    this.jacr.delete();
    this.data.delete();
  }
  /** After the panel edits the model (base pose, mount yaw): the IK data follows the model's numbers. */
  refreshModel() {
    this.update(this.q);
  }
  /** `ArmIK.reset`: the commanded configuration, the posture target and the arm's current joint speeds (v0). */
  reset(q, qPosture, v0) {
    this.q = Array.from(q);
    this.qPosture = qPosture ? Array.from(qPosture) : this.q.slice();
    this.vPrev = v0 ? Array.from(v0) : [0, 0, 0, 0, 0, 0];
  }
  update(q) {
    const qpos = this.data.qpos;
    this.meta.arm_qpos.forEach((a, i) => qpos[a] = q[i]);
    this.mujoco.mj_kinematics(this.model, this.data);
    this.mujoco.mj_comPos(this.model, this.data);
  }
  /** (pos, wxyz) of r_wrist at q (default: the commanded configuration), as mink reads its frame. */
  wristPose(q) {
    this.update(q ?? this.q);
    const b = this.meta.wrist_body;
    const xpos = this.data.xpos;
    const xmat = this.data.xmat;
    return {
      pos: [xpos[3 * b], xpos[3 * b + 1], xpos[3 * b + 2]],
      quat: mat33ToQuat(xmat.subarray(9 * b, 9 * b + 9))
    };
  }
  /** `build_qp`: (H, c, lo, hi) at the commanded configuration (already updated). */
  buildQp(target, twist) {
    const { cfg, meta, model } = this;
    const b = meta.wrist_body;
    const xpos = this.data.xpos;
    const xmat = this.data.xmat;
    const frame = [
      ...mat33ToQuat(xmat.subarray(9 * b, 9 * b + 9)),
      xpos[3 * b],
      xpos[3 * b + 1],
      xpos[3 * b + 2]
    ];
    const { err, M } = frameTaskTerms(target, frame);
    this.mujoco.mj_jacBody(model, this.data, this.jacp, this.jacr, b);
    const jp = this.jacp.GetView();
    const jr = this.jacr.GetView();
    const nv = model.nv;
    const dof = meta.arm_qvel;
    const jac = new Array(36);
    for (let r = 0; r < 3; r++)
      for (let k = 0; k < 6; k++) {
        jac[6 * r + k] = jp[nv * r + dof[k]];
        jac[6 * (r + 3) + k] = jr[nv * r + dof[k]];
      }
    const cost = [
      cfg.position_cost,
      cfg.position_cost,
      cfg.position_cost,
      cfg.orientation_cost,
      cfg.orientation_cost,
      cfg.orientation_cost
    ];
    const W = new Array(72).fill(0);
    const e = new Array(12).fill(0);
    for (let i = 0; i < 6; i++) {
      for (let k = 0; k < 6; k++) {
        let s = 0;
        for (let j = 0; j < 6; j++) s += M[6 * i + j] * jac[6 * j + k];
        W[6 * i + k] = cost[i] * s;
      }
      e[i] = cost[i] * -err[i];
      W[6 * (6 + i) + i] = cfg.posture_cost;
      e[6 + i] = cfg.posture_cost * -(this.q[i] - this.qPosture[i]);
    }
    const h = new Array(36).fill(0);
    const c = new Array(6).fill(0);
    for (let j = 0; j < 6; j++) {
      for (let k = 0; k < 6; k++) {
        let s2 = 0;
        for (let i = 0; i < 12; i++) s2 += W[6 * i + j] * W[6 * i + k];
        h[6 * j + k] = s2;
      }
      let s = 0;
      for (let i = 0; i < 12; i++) s += e[i] * W[6 * i + j];
      c[j] = -s;
      h[6 * j + j] += cfg.damping;
    }
    const lo = this.q.map(
      (q, i) => Math.max(-cfg.limit_gain * (q - this.lower[i]), -this.vmax[i] * cfg.dt)
    );
    const hi = this.q.map(
      (q, i) => Math.min(cfg.limit_gain * (this.upper[i] - q), this.vmax[i] * cfg.dt)
    );
    let cTwist = null;
    if (twist) {
      const ex = new Array(6).fill(0);
      for (let i = 0; i < 6; i++) {
        let s = 0;
        for (let k = 0; k < 6; k++) s += M[6 * i + k] * (twist[k] * cfg.dt);
        ex[i] = cost[i] * s;
      }
      cTwist = new Array(6).fill(0);
      for (let j = 0; j < 6; j++) {
        let s = 0;
        for (let i = 0; i < 6; i++) s += ex[i] * W[6 * i + j];
        cTwist[j] = -s;
      }
    }
    return { h, c, lo, hi, cTwist };
  }
  /** One step of length dt toward the goal: (q at the step's start, joint velocity); q advances to q + v dt.
   *  `twist` (world, linear then angular): the goal's own velocity, the braking cap's reference (`ArmIK.step`). */
  step(goalPos, goalQuat, twist) {
    const n = Math.hypot(goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]);
    const target = [
      goalQuat[0] / n,
      goalQuat[1] / n,
      goalQuat[2] / n,
      goalQuat[3] / n,
      goalPos[0],
      goalPos[1],
      goalPos[2]
    ];
    this.update(this.q);
    const tw = twist && Array.from(twist).every(Number.isFinite) ? twist : void 0;
    const { h, c, lo, hi, cTwist } = this.buildQp(target, tw);
    for (let i = 0; i < 6; i++) {
      if (lo[i] > 0) lo[i] = 0;
      if (hi[i] < 0) hi[i] = 0;
    }
    const dt = this.cfg.dt;
    let box = { lo, hi };
    this.braked = false;
    if (this.amax) {
      const u = cTwist ? solveN(
        h,
        cTwist.map((x) => -x),
        6
      ).map((x) => x / dt) : void 0;
      const b = accelBox(
        h,
        c,
        lo,
        hi,
        this.vPrev,
        this.amax,
        dt,
        this.cfg.accel_braking ?? true,
        u
      );
      this.accelRelaxed += b.relaxed.filter(Boolean).length;
      this.braked = b.braked.some(Boolean);
      box = b;
    }
    const dq = boxQp(h, c, box.lo, box.hi);
    const q0 = this.q.slice();
    const v = dq.map((x) => x / dt);
    if (v.every(Number.isFinite)) this.q = q0.map((q, i) => q + v[i] * dt);
    else {
      this.fallbacks += 1;
      this.vPrev = [0, 0, 0, 0, 0, 0];
      return { q0, v: [0, 0, 0, 0, 0, 0] };
    }
    const amax = this.amax;
    this.accelBound = !!amax && v.some(
      (x, i) => Math.abs(x - this.vPrev[i]) >= (1 - 1e-6) * amax[i] * dt
    );
    this.vPrev = v.slice();
    return { q0, v };
  }
  /** `ArmIK.weighted_jacobian`: J_W (6 x 6, row-major) of r_wrist at q, the rotation rows times the cost ratio. */
  weightedJacobian(q) {
    this.update(q);
    this.mujoco.mj_jacBody(
      this.model,
      this.data,
      this.jacp,
      this.jacr,
      this.meta.wrist_body
    );
    const jp = this.jacp.GetView();
    const jr = this.jacr.GetView();
    const nv = this.model.nv;
    const w = this.cfg.orientation_cost / this.cfg.position_cost;
    const dof = this.meta.arm_qvel;
    const j = new Array(36);
    for (let r = 0; r < 3; r++)
      for (let k = 0; k < 6; k++) {
        j[6 * r + k] = jp[nv * r + dof[k]];
        j[6 * (r + 3) + k] = w * jr[nv * r + dof[k]];
      }
    return j;
  }
  /** `ArmIK.amplification` (G-arm6): rho = ||dq|| / ||dx||_W of the commanded move q0 -> q1, sigma_min(J_W) at q0. */
  amplification(q0, q1) {
    const sigma = sigmaMin6(this.weightedJacobian(q0));
    let dq = 0;
    for (let i = 0; i < 6; i++) dq += (q1[i] - q0[i]) ** 2;
    dq = Math.sqrt(dq);
    if (dq < 1e-9) return { rho: Number.NaN, sigma };
    const a = this.wristPose(q0);
    const b = this.wristPose(q1);
    const rel = qmul(b.quat, [a.quat[0], -a.quat[1], -a.quat[2], -a.quat[3]]);
    const rot = so3Log(rel);
    const w = this.cfg.orientation_cost / this.cfg.position_cost;
    const dx = Math.hypot(
      b.pos[0] - a.pos[0],
      b.pos[1] - a.pos[1],
      b.pos[2] - a.pos[2],
      w * rot[0],
      w * rot[1],
      w * rot[2]
    );
    return { rho: dq / Math.max(dx, 1e-15), sigma };
  }
  /** (mm, deg) between the commanded configuration's wrist and the goal. */
  residual(goalPos, goalQuat) {
    const { pos, quat } = this.wristPose();
    const n = Math.hypot(goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]);
    const g = [
      goalQuat[0] / n,
      -goalQuat[1] / n,
      -goalQuat[2] / n,
      -goalQuat[3] / n
    ];
    const d = so3Log(qmul(g, quat));
    return [
      1e3 * Math.hypot(
        pos[0] - goalPos[0],
        pos[1] - goalPos[1],
        pos[2] - goalPos[2]
      ),
      Math.hypot(d[0], d[1], d[2]) * 180 / Math.PI
    ];
  }
};
function mul4(a, b) {
  const o = new Array(16);
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++)
      o[4 * i + j] = a[4 * i] * b[j] + a[4 * i + 1] * b[4 + j] + a[4 * i + 2] * b[8 + j] + a[4 * i + 3] * b[12 + j];
  return o;
}
function inv4(t) {
  const r = [t[0], t[4], t[8], t[1], t[5], t[9], t[2], t[6], t[10]];
  const p = [t[3], t[7], t[11]];
  const ip = [0, 1, 2].map(
    (i) => -(r[3 * i] * p[0] + r[3 * i + 1] * p[1] + r[3 * i + 2] * p[2])
  );
  return [
    r[0],
    r[1],
    r[2],
    ip[0],
    r[3],
    r[4],
    r[5],
    ip[1],
    r[6],
    r[7],
    r[8],
    ip[2],
    0,
    0,
    0,
    1
  ];
}
function dhLink(theta, i, dh) {
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(dh.alpha[i]);
  const sa = Math.sin(dh.alpha[i]);
  return [
    ct,
    -st * ca,
    st * sa,
    dh.a[i] * ct,
    st,
    ct * ca,
    -ct * sa,
    dh.a[i] * st,
    0,
    sa,
    ca,
    dh.d[i],
    0,
    0,
    0,
    1
  ];
}
var wrap = (x) => {
  const y = (x + Math.PI) % (2 * Math.PI);
  return (y < 0 ? y + 2 * Math.PI : y) - Math.PI;
};
function ikAnalytic(t06, dh, q6Hint = 0) {
  const [d4, d6] = [dh.d[3], dh.d[5]];
  const [a2, a3] = [dh.a[1], dh.a[2]];
  const out = new Array(8).fill(null);
  const p05 = [
    t06[3] - d6 * t06[2],
    t06[7] - d6 * t06[6],
    t06[11] - d6 * t06[10]
  ];
  const r = Math.hypot(p05[0], p05[1]);
  if (r < Math.abs(d4)) return out;
  const psi = Math.atan2(p05[1], p05[0]);
  const phi = Math.acos(Math.min(1, Math.max(-1, d4 / r)));
  const t60 = inv4(t06);
  [psi + phi + Math.PI / 2, psi - phi + Math.PI / 2].forEach((th1, a) => {
    const s1 = Math.sin(th1);
    const c1 = Math.cos(th1);
    const arg5 = (t06[3] * s1 - t06[7] * c1 - d4) / d6;
    if (Math.abs(arg5) > 1 + 1e-12) return;
    [1, -1].forEach((sign5, b) => {
      const th5 = sign5 * Math.acos(Math.min(1, Math.max(-1, arg5)));
      const s5 = Math.sin(th5);
      const th6 = Math.abs(s5) < 1e-10 ? q6Hint : Math.atan2(
        (-t60[4] * s1 + t60[5] * c1) / s5,
        (t60[0] * s1 - t60[1] * c1) / s5
      );
      const t14 = mul4(
        mul4(inv4(dhLink(th1, 0, dh)), t06),
        inv4(mul4(dhLink(th5, 4, dh), dhLink(th6, 5, dh)))
      );
      const px = t14[3];
      const py = t14[7];
      const arg3 = (px * px + py * py - a2 * a2 - a3 * a3) / (2 * a2 * a3);
      if (Math.abs(arg3) > 1 + 1e-12) return;
      [1, -1].forEach((sign3, c) => {
        const th3 = sign3 * Math.acos(Math.min(1, Math.max(-1, arg3)));
        const th2 = Math.atan2(py, px) - Math.atan2(a3 * Math.sin(th3), a2 + a3 * Math.cos(th3));
        const t34 = mul4(
          inv4(mul4(dhLink(th2, 1, dh), dhLink(th3, 2, dh))),
          t14
        );
        const th4 = Math.atan2(t34[4], t34[0]);
        out[4 * a + 2 * b + c] = [th1, th2, th3, th4, th5, th6].map(wrap);
      });
    });
  });
  return out;
}
function minJerk(q0, q1, duration, t) {
  const s = Math.min(1, Math.max(0, t / duration));
  const p = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5;
  const v = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / duration;
  return [
    Array.from(q0, (a, i) => a + (q1[i] - a) * p),
    Array.from(q0, (a, i) => (q1[i] - a) * v)
  ];
}
function minJerkDuration(q0, q1, vel, fraction, minimum) {
  let t = 0;
  for (let i = 0; i < vel.length; i++)
    t = Math.max(t, 1.875 * Math.abs(q1[i] - q0[i]) / (fraction * vel[i]));
  return Math.max(minimum, Math.ceil(t / 0.25) * 0.25);
}
function sigmaMin6(j) {
  const a = new Array(36).fill(0);
  for (let r = 0; r < 6; r++)
    for (let c = 0; c < 6; c++) {
      let s = 0;
      for (let k = 0; k < 6; k++) s += j[6 * k + r] * j[6 * k + c];
      a[6 * r + c] = s;
    }
  for (let sweep = 0; sweep < 30; sweep++) {
    let off = 0;
    for (let p = 0; p < 6; p++)
      for (let q = p + 1; q < 6; q++) off += a[6 * p + q] ** 2;
    if (off < 1e-30) break;
    for (let p = 0; p < 6; p++)
      for (let q = p + 1; q < 6; q++) {
        const apq = a[6 * p + q];
        if (Math.abs(apq) < 1e-300) continue;
        const theta = (a[6 * q + q] - a[6 * p + p]) / (2 * apq);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < 6; k++) {
          const akp = a[6 * k + p];
          const akq = a[6 * k + q];
          a[6 * k + p] = c * akp - s * akq;
          a[6 * k + q] = s * akp + c * akq;
        }
        for (let k = 0; k < 6; k++) {
          const apk = a[6 * p + k];
          const aqk = a[6 * q + k];
          a[6 * p + k] = c * apk - s * aqk;
          a[6 * q + k] = s * apk + c * aqk;
        }
      }
  }
  let m = Infinity;
  for (let i = 0; i < 6; i++) m = Math.min(m, a[7 * i]);
  return Math.sqrt(Math.max(0, m));
}
function branchSigns(q, dh) {
  const sh = dh.a[1] * Math.cos(q[1]) + dh.a[2] * Math.cos(q[1] + q[2]) + dh.d[4] * Math.sin(q[1] + q[2] + q[3]);
  return [Math.sign(sh), Math.sign(Math.sin(q[2])), Math.sign(Math.sin(q[4]))];
}
function planDirect(qStart, meta) {
  const a = meta.approach;
  const t1 = minJerkDuration(
    meta.rest_q,
    qStart,
    meta.vel_limits,
    a.speed_fraction,
    a.minimum_s
  );
  return { qRest: meta.rest_q.slice(), qPre: qStart, qStart, t1, t2: 0 };
}
function planApproach(qStart, qPre, meta) {
  const a = meta.approach;
  const t1 = minJerkDuration(
    meta.rest_q,
    qPre,
    meta.vel_limits,
    a.speed_fraction,
    a.minimum_s
  );
  const t2 = minJerkDuration(
    qPre,
    qStart,
    meta.vel_limits,
    a.speed_fraction,
    a.last_segment_s
  );
  return { qRest: meta.rest_q.slice(), qPre, qStart, t1, t2 };
}
function sampleApproach(p, t) {
  if (p.t2 <= 0 || t < p.t1) return minJerk(p.qRest, p.qPre, p.t1, t);
  return minJerk(p.qPre, p.qStart, p.t2, t - p.t1);
}
function preStartPos(wristPos, objectCom, retreat, lift) {
  const u = [
    wristPos[0] - objectCom[0],
    wristPos[1] - objectCom[1],
    wristPos[2] - objectCom[2]
  ];
  const n = Math.hypot(u[0], u[1], u[2]);
  const d = n > 1e-9 ? u.map((v) => v / n) : [0, 0, 1];
  return [
    wristPos[0] + retreat * d[0],
    wristPos[1] + retreat * d[1],
    wristPos[2] + retreat * d[2] + lift
  ];
}

// ../../../../../../web/plugins/arm_env.ts
var GUARD_ZH = {
  goal_clamp: "\u62A4\u680F\uFF1AIK \u7684\u8F93\u5165\u8D85\u51FA\u81C2\u5C55\uFF08\u79BB\u80A9 > 1.6 m\uFF0C\u6216\u4E0D\u662F\u6709\u9650\u503C\uFF09",
  virtual_reset: "\u62A4\u680F\uFF1A\u5BFC\u7EB3\u7684\u865A\u62DF\u624B\u8155\u53D1\u6563\uFF08\u4E0D\u662F\u6709\u9650\u503C\uFF09",
  ik_no_step: "\u62A4\u680F\uFF1AIK \u8FD9\u4E00\u6B65\u4E0D\u662F\u6709\u9650\u503C",
  singular: "\u62A4\u680F\uFF1A\u673A\u68B0\u81C2\u63A5\u8FD1\u5947\u5F02\u4F4D\u5F62\uFF08\u03C3_min(J_W) < 0.03\uFF1A\u76F8\u8FD1\u7684\u624B\u8155\u4F4D\u59FF\u8981\u5F88\u5927\u7684\u5173\u8282\u8FD0\u52A8\uFF09",
  protective_stop: "UR \u4FDD\u62A4\u6027\u505C\u6B62\uFF1A\u5173\u8282\u7684\u5B9E\u6D4B\u901F\u5EA6\u8FDE\u7EED\u8D85\u8FC7 UR \u7684\u4E0A\u9650\uFF08\u771F\u673A\u7684\u5B89\u5168\u7CFB\u7EDF\u4F1A\u505C\u673A\uFF09",
  mujoco_reset: "\u6570\u503C\u53D1\u6563\uFF08MuJoCo \u81EA\u52A8\u590D\u4F4D\uFF1Abad qpos / qvel / qacc / ctrl\uFF09"
};
var SPEED_STOP_DEFAULT = { ratio: 1.05, substeps: 3 };
function guardText(kind, meta) {
  if (kind !== "protective_stop") return GUARD_ZH[kind];
  const stop = meta.speed_stop ?? SPEED_STOP_DEFAULT;
  return `UR \u4FDD\u62A4\u6027\u505C\u6B62\uFF1A\u5173\u8282\u7684\u5B9E\u6D4B\u901F\u5EA6\u8FDE\u7EED ${stop.substeps} \u4E2A MuJoCo \u5B50\u6B65\u8D85\u8FC7 UR \u4E0A\u9650\u7684 ${(100 * stop.ratio).toFixed(0)} %\uFF08\u771F\u673A\u7684\u5B89\u5168\u7CFB\u7EDF\u4F1A\u505C\u673A\uFF09`;
}
function armStartOf(qpos, qvel, fl) {
  const w = fl.wrist_qpos;
  const v = fl.wrist_qvel;
  const quat = [qpos[w + 3], qpos[w + 4], qpos[w + 5], qpos[w + 6]];
  const omega = quatApply(quat, [qvel[v + 3], qvel[v + 4], qvel[v + 5]]);
  const rCom = quatApply(quat, fl.wrist_ipos);
  const wxr = cross(omega, rCom);
  return {
    wristPos: [qpos[w], qpos[w + 1], qpos[w + 2]],
    wristQuat: quat,
    wristVelCom: [qvel[v] + wxr[0], qvel[v + 1] + wxr[1], qvel[v + 2] + wxr[2]],
    wristOmega: omega,
    joints: fl.finger_qpos.map((a) => qpos[a]),
    jointVel: fl.finger_qvel.map((a) => qvel[a]),
    objectQpos: Array.from(
      { length: 7 },
      (_, i) => qpos[fl.object_qpos + i]
    ),
    objectQvel: Array.from(
      { length: 6 },
      (_, i) => qvel[fl.object_qvel + i]
    )
  };
}
var BAD_WARNINGS = [1, 2, 3, 6];
function mat3(q) {
  const [w, x, y, z] = [q[0], q[1], q[2], q[3]];
  return [
    1 - 2 * (y * y + z * z),
    2 * (x * y - z * w),
    2 * (x * z + y * w),
    2 * (x * y + z * w),
    1 - 2 * (x * x + z * z),
    2 * (y * z - x * w),
    2 * (x * z - y * w),
    2 * (y * z + x * w),
    1 - 2 * (x * x + y * y)
  ];
}
var mv = (m, v) => [0, 1, 2].map(
  (i) => m[3 * i] * v[0] + m[3 * i + 1] * v[1] + m[3 * i + 2] * v[2]
);
var mtv = (m, v) => [0, 1, 2].map((i) => m[i] * v[0] + m[3 + i] * v[1] + m[6 + i] * v[2]);
function conj3(r, a) {
  const ra = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      ra[3 * i + j] = r[3 * i] * a[j] + r[3 * i + 1] * a[3 + j] + r[3 * i + 2] * a[6 + j];
  const out = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      out[3 * i + j] = ra[3 * i] * r[3 * j] + ra[3 * i + 1] * r[3 * j + 1] + ra[3 * i + 2] * r[3 * j + 2];
  return out;
}
var VirtualWrist = class {
  pos = [0, 0, 0];
  quat = [1, 0, 0, 0];
  vel = [0, 0, 0];
  omega = [0, 0, 0];
  reset(pos, quat, vel, omega) {
    const n = Math.hypot(quat[0], quat[1], quat[2], quat[3]);
    this.pos = Array.from(pos);
    this.quat = Array.from(quat, (v) => v / n);
    this.vel = Array.from(vel);
    this.omega = Array.from(omega);
  }
  /** `env.VirtualWrist.advance`: n steps of length h under a constant wrench; the COM by semi-implicit Euler, the
   *  rotation on the world angular momentum L (L += h torque, omega = I(q)^-1 L, then the rotation by omega h). */
  advance(force, torque, mass, cOff, iBody, h, n) {
    let ang = mv(conj3(mat3(this.quat), iBody), this.omega);
    for (let k = 0; k < n; k++) {
      this.vel = this.vel.map((v, i) => v + h * force[i] / mass);
      ang = ang.map((l, i) => l + h * torque[i]);
      const r = mat3(this.quat);
      this.omega = solve3(conj3(r, iBody), ang);
      const rc = mv(r, cOff);
      const com = [0, 1, 2].map((i) => this.pos[i] + rc[i] + h * this.vel[i]);
      const a = Math.hypot(this.omega[0], this.omega[1], this.omega[2]) * h;
      let dq = [1, 0, 0, 0];
      if (a >= 1e-15) {
        const wn = Math.hypot(this.omega[0], this.omega[1], this.omega[2]);
        const s = Math.sin(a / 2) / wn;
        dq = [
          Math.cos(a / 2),
          s * this.omega[0],
          s * this.omega[1],
          s * this.omega[2]
        ];
      }
      const q = quatMul(dq, this.quat);
      const qn = Math.hypot(q[0], q[1], q[2], q[3]);
      this.quat = q.map((v) => v / qn);
      const rc2 = mv(mat3(this.quat), cOff);
      this.pos = [0, 1, 2].map((i) => com[i] - rc2[i]);
    }
    this.omega = solve3(conj3(mat3(this.quat), iBody), ang);
  }
  finite() {
    return [...this.pos, ...this.quat, ...this.vel, ...this.omega].every(
      Number.isFinite
    );
  }
};
var ArmEnv = class {
  hand = "hand21";
  wristMode = "isaac";
  mujoco;
  model;
  data;
  L;
  arm;
  filter;
  ik;
  virtual = new VirtualWrist();
  substeps;
  decimation;
  h;
  dt;
  /** The arm adds no wrist wrench (the floating env's `wrench` / `base` stay zero; the viewer's pulls pass). */
  base = new Float64Array(6);
  wrench = new Float64Array(6);
  handPoints = new Float64Array(26 * 3);
  mode;
  /** The reference configuration of the IK branch for `armStart` (the selected placement's). */
  branchQ;
  unstable = false;
  /** Test hook (`window.__mjwebArm.forceFarGoal()`, arm_smoke.mjs): the next IK goal lies 10 m out, so the goal-clamp
   * guard ends the episode whatever the policy and the start do. */
  forceFarGoal = false;
  state;
  qpIters = 0;
  phase = "policy";
  /** Diagnostics of the current episode (the panel; parity). */
  log = {
    ikMm: 0,
    ikDeg: 0,
    ikMaxMm: 0,
    trackMm: 0,
    trackDeg: 0,
    speedFrac: new Array(6).fill(0),
    speedFracMax: 0,
    /** IK steps of the episode, and those whose velocity limit binds (a joint at UR's speed). */
    ikSteps: 0,
    velBoundSteps: 0,
    armObjectSubsteps: 0,
    settleMm: 0,
    settleDeg: 0,
    goalClamped: 0,
    virtualResets: 0,
    /** Control steps of the episode, and the first guard that fired (it ends the episode as unstable). */
    steps: 0,
    firstGuard: null,
    /** G-arm6: the last IK step's joint amplification rho (rad/m) and sigma_min(J_W), the episode's extremes, the
     *  IK steps beyond the thresholds and the IK-branch sign changes. */
    rho: Number.NaN,
    sigma: Number.NaN,
    rhoMax: 0,
    sigmaMinMin: Infinity,
    garm6Flagged: 0,
    branchFlips: 0,
    /** Measured joint speed / UR's limit: the episode's largest, and the substeps above the limit. */
    qdRatioMax: 0,
    qdOverSubsteps: 0,
    /** IK steps at the acceleration bound, and joint-steps where it gave way to a position / velocity limit. */
    accelBoundSteps: 0,
    accelRelaxed: 0,
    /** Admittance: the real wrist against the virtual one it follows (the tracking cost), now and the episode's max. */
    vdevMm: 0,
    vdevDeg: 0,
    vdevMaxMm: 0,
    vdevMaxDeg: 0
  };
  /** The measured contact wrench on the hand (force, torque about the hand's COM), mean of the last physics step. */
  contactWrench = new Float64Array(6);
  /** The IK goal of the current physics step (the drawn commanded frame). */
  goalPos = [0, 0, 0];
  goalQuat = [1, 0, 0, 0];
  acc = new Float64Array(6);
  gains;
  gravity;
  vmax;
  handBodies;
  handSet;
  armSet;
  masses;
  kp;
  kvb;
  warnings;
  bad0 = 0;
  /** `servo.ArmServo`: the C1 reference (position, velocity) and its acceleration over the current step. */
  ref = {
    q: [0, 0, 0, 0, 0, 0],
    v: [0, 0, 0, 0, 0, 0],
    a: [0, 0, 0, 0, 0, 0]
  };
  mulRes;
  /** Admittance: the virtual wrist's twist (r_wrist linear velocity, angular velocity; world), the IK's braking
   *  reference (`env._goal_twist`). */
  goalTwist = null;
  f6;
  jacp;
  jacr;
  qStart = [0, 0, 0, 0, 0, 0];
  plan = null;
  phaseT = 0;
  pending = null;
  phaseSettle = 0;
  signs = [0, 0, 0];
  stopStreak = 0;
  constructor(mujoco, model, data, layout, ctl, arm, alphaWrist, alphaFingers, mode) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    this.L = layout;
    this.arm = arm;
    this.mode = mode;
    this.branchQ = arm.branch_q.slice();
    this.h = model.opt.timestep;
    this.dt = ctl.isaac_dt ?? ISAAC_DT;
    this.substeps = Math.round(this.dt / this.h);
    if (Math.abs(this.substeps * this.h - this.dt) > 1e-6)
      throw new Error(`[mjweb] timestep ${this.h} does not divide 1/120 s`);
    this.decimation = 4 * this.substeps;
    const lo = new Float64Array(NUM_FINGERS);
    const hi = new Float64Array(NUM_FINGERS);
    for (let i = 0; i < NUM_FINGERS; i++) {
      lo[i] = layout.fingerRange[2 * i];
      hi[i] = layout.fingerRange[2 * i + 1];
    }
    this.filter = new TargetFilter(
      lo,
      hi,
      alphaWrist,
      alphaFingers,
      ctl.control_dt,
      ctl.max_ff_lin,
      ctl.max_ff_ang
    );
    this.vmax = Float64Array.from(ctl.finger_vel_limits);
    const g = ctl.wrist_gains;
    this.gains = [g[0], g[1], g[2], g[3]];
    const grav = model.opt.gravity;
    this.gravity = ctl.gravity ? [...ctl.gravity] : [grav[0], grav[1], grav[2]];
    this.handBodies = handBodyIds(model, layout.wristBody);
    this.handSet = new Set(Array.from(this.handBodies));
    const parent = model.body_parentid;
    const armSet = /* @__PURE__ */ new Set();
    let b = parent[layout.wristBody];
    while (b > 0) {
      armSet.add(b);
      b = parent[b];
    }
    this.armSet = armSet;
    const mass = model.body_mass;
    this.masses = Float64Array.from(this.handBodies, (k) => mass[k]);
    const gain = model.actuator_gainprm;
    const bias = model.actuator_biasprm;
    const damp = model.dof_damping;
    const nG = gain.length / model.nu;
    const nB = bias.length / model.nu;
    this.kp = arm.meta.arm_act.map((a) => gain[nG * a]);
    this.kvb = arm.meta.arm_act.map(
      (a, i) => -bias[nB * a + 2] + damp[arm.meta.arm_qvel[i]]
    );
    this.ik = new ArmIk(mujoco, model, arm.meta);
    const M = mujoco;
    this.f6 = new M.DoubleBuffer(6);
    this.mulRes = new M.DoubleBuffer(model.nv);
    this.jacp = new M.DoubleBuffer(3 * model.nv);
    this.jacr = new M.DoubleBuffer(3 * model.nv);
    this.warnings = data.warning;
    this.state = this.readState();
    globalThis.__mjwebArm = {
      forceFarGoal: () => {
        this.forceFarGoal = true;
      }
    };
  }
  dispose() {
    this.f6.delete();
    this.mulRes.delete();
    this.jacp.delete();
    this.jacr.delete();
    this.ik.dispose();
  }
  /** Numerical blow-ups (MuJoCo's automatic resets): mjWARN_BADQPOS / BADQVEL / BADQACC / BADCTRL. */
  badqacc() {
    let n = 0;
    for (const w of BAD_WARNINGS) {
      const stat = this.warnings.get(w);
      if (!stat) continue;
      n += stat.number;
      stat.delete();
    }
    return n;
  }
  // ------------------------------------------------------------------ start configuration
  /** The base's DH frame 0 in the world (4x4) from the compiled model (the panel may move the base). */
  baseFrame() {
    const d = this.data;
    this.mujoco.mj_kinematics(this.model, d);
    const b = this.arm.meta.base_body;
    const xpos = d.xpos;
    const xquat = d.xquat;
    const q = quatMul(
      [xquat[4 * b], xquat[4 * b + 1], xquat[4 * b + 2], xquat[4 * b + 3]],
      this.arm.meta.base_to_dh0
    );
    const r = mat3(q);
    return [
      r[0],
      r[1],
      r[2],
      xpos[3 * b],
      r[3],
      r[4],
      r[5],
      xpos[3 * b + 1],
      r[6],
      r[7],
      r[8],
      xpos[3 * b + 2],
      0,
      0,
      0,
      1
    ];
  }
  /** The flange pose that puts r_wrist at (pos, quat), in UR's base frame. */
  flangeInBase(pos, quat) {
    const yaw = String(this.mountYaw());
    const wf = this.arm.meta.wrist_in_flange[yaw];
    const qInv = quatConjugate(wf.quat);
    const pInv = quatApply(qInv, wf.pos).map((v) => -v);
    const qF = quatMul(quat, qInv);
    const pF = quatApply(quat, pInv).map((v, i) => v + pos[i]);
    const T = this.baseFrame();
    const R = [T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]];
    const p = mtv(R, [pF[0] - T[3], pF[1] - T[7], pF[2] - T[11]]);
    const rf = mat3(qF);
    const rb = [0, 1, 2].flatMap(
      (i) => [0, 1, 2].map(
        (j) => R[i] * rf[j] + R[3 + i] * rf[3 + j] + R[6 + i] * rf[6 + j]
      )
    );
    return [
      rb[0],
      rb[1],
      rb[2],
      p[0],
      rb[3],
      rb[4],
      rb[5],
      p[1],
      rb[6],
      rb[7],
      rb[8],
      p[2],
      0,
      0,
      0,
      1
    ];
  }
  /** Base pose and mount yaw (URL / panel): edits the base body and r_wrist's mounting in the model, moves the
   *  floor, and takes the placement's branch reference (`branchQ`, else the current one). */
  setPlacement(p, branchQ) {
    const m = this.model;
    const b = this.arm.meta.base_body;
    const pos = m.body_pos;
    const quat = m.body_quat;
    const a = p.yaw_deg * Math.PI / 180 / 2;
    const q = quatMul(
      [Math.cos(a), 0, 0, Math.sin(a)],
      quatConjugate(this.arm.meta.base_to_dh0)
    );
    pos.set([p.x, p.y, p.z], 3 * b);
    quat.set(q, 4 * b);
    const yaw = String(Math.round(p.mount_yaw_deg / 90) * 90 % 360);
    const wf = this.arm.meta.wrist_in_flange[yaw];
    if (!wf)
      throw new Error(
        `[mjweb] mount yaw ${p.mount_yaw_deg} is not one of 0 / 90 / 180 / 270`
      );
    const f = this.arm.meta.flange_in_wrist3;
    const w = this.arm.meta.wrist_body;
    const wp = quatApply(f.quat, wf.pos).map((v, i) => v + f.pos[i]);
    pos.set(wp, 3 * w);
    quat.set(quatMul(f.quat, wf.quat), 4 * w);
    if (this.arm.floor_mocap >= 0)
      this.data.mocap_pos.set(
        [p.x, p.y, p.floor_z ?? p.z - 0.8],
        3 * this.arm.floor_mocap
      );
    if (branchQ) this.branchQ = branchQ.slice();
    this.mujoco.mj_forward(m, this.data);
    this.ik.refreshModel();
  }
  /** Whether the model carries this placement (then the harness's precomputed configurations apply). */
  placementMatches(p) {
    const b = this.arm.meta.base_body;
    const pos = this.model.body_pos;
    const quat = this.model.body_quat;
    const a = p.yaw_deg * Math.PI / 180 / 2;
    const q = quatMul(
      [Math.cos(a), 0, 0, Math.sin(a)],
      quatConjugate(this.arm.meta.base_to_dh0)
    );
    const dp = Math.hypot(
      pos[3 * b] - p.x,
      pos[3 * b + 1] - p.y,
      pos[3 * b + 2] - p.z
    );
    const dq = Math.abs(
      q[0] * quat[4 * b] + q[1] * quat[4 * b + 1] + q[2] * quat[4 * b + 2] + q[3] * quat[4 * b + 3]
    );
    return dp < 1e-9 && dq > 1 - 1e-12 && this.mountYaw() === Math.round(p.mount_yaw_deg / 90) * 90 % 360;
  }
  /** The mount yaw of r_wrist on wrist 3 (the scene's, or what the panel set). */
  mountYaw() {
    const b = this.arm.meta.wrist_body;
    const bq = this.model.body_quat;
    const q = [bq[4 * b], bq[4 * b + 1], bq[4 * b + 2], bq[4 * b + 3]];
    const f = this.arm.meta.flange_in_wrist3;
    for (const yaw of [0, 90, 180, 270]) {
      const wf = this.arm.meta.wrist_in_flange[String(yaw)];
      const want = quatMul(f.quat, wf.quat);
      if (Math.abs(
        Math.abs(
          want[0] * q[0] + want[1] * q[1] + want[2] * q[2] + want[3] * q[3]
        ) - 1
      ) < 1e-9)
        return yaw;
    }
    throw new Error(
      "[mjweb] r_wrist is not at one of the four bolt-pattern yaws"
    );
  }
  /** `arm_start`: the closed-form solution nearest `branchQ`, polished by the IK on the scene's kinematics. */
  armStart(pos, quat, precomputed) {
    if (precomputed) return precomputed.slice();
    const t06 = this.flangeInBase(pos, quat);
    const ref = this.branchQ;
    let best = null;
    let cost = Infinity;
    for (const s of ikAnalytic(t06, this.arm.meta.dh)) {
      if (!s) continue;
      const c = s.map(
        (v, i) => ref[i] + ((v - ref[i] + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI
      );
      const l1 = c.reduce((a, v, i) => a + Math.abs(v - ref[i]), 0);
      if (l1 < cost) {
        cost = l1;
        best = c;
      }
    }
    if (!best) throw new Error("[mjweb] the start is out of the arm's reach");
    this.ik.reset(best);
    for (let k = 0; k < 50; k++) this.ik.step(pos, quat);
    return this.ik.q.slice();
  }
  /** `arm_pre_start`: 2 s of IK from the start configuration toward the pre-start pose. */
  armPreStart(s, qStart, precomputed) {
    if (precomputed) return precomputed.slice();
    const oq = s.objectQpos;
    const com = quatApply(
      [oq[3], oq[4], oq[5], oq[6]],
      this.arm.object_ipos
    ).map((v, i) => v + oq[i]);
    const a = this.arm.meta.approach;
    const pre = preStartPos(s.wristPos, com, a.retreat_m, a.lift_m);
    this.ik.reset(qStart);
    for (let k = 0; k < 240; k++) this.ik.step(pre, s.wristQuat);
    return this.ik.q.slice();
  }
  // ------------------------------------------------------------------ reset (phase 2) and the page's phases 0-1
  parkObject() {
    const qpos = this.data.qpos;
    const qvel = this.data.qvel;
    const o = this.L.objectQpos;
    qpos.set([0, 0, -50, 1, 0, 0, 0], o);
    for (let i = 0; i < 6; i++) qvel[this.L.objectQvel + i] = 0;
  }
  /** `ArmServo.substep`: ctrl for the next MuJoCo step from the reference (with the feed-forward), then the reference
   *  advances by h. */
  servoCtrl() {
    const ctrl = this.data.ctrl;
    const act = this.arm.meta.arm_act;
    const ff = this.feedforward();
    const { q, v, a } = this.ref;
    const h = this.h;
    for (let i = 0; i < 6; i++) {
      ctrl[act[i]] = q[i] + (this.kvb[i] * v[i] + ff[i]) / this.kp[i];
      q[i] = q[i] + h * v[i] + 0.5 * h * h * a[i];
      v[i] = v[i] + h * a[i];
    }
  }
  /** `ArmServo.feedforward_torque`: M(q) a + bias - gravcomp on the arm's joints (the last forward pass's). */
  feedforward() {
    const bias = this.data.qfrc_bias;
    const grav = this.data.qfrc_gravcomp;
    const dofs = this.arm.meta.arm_qvel;
    const tau = dofs.map((d) => bias[d] - grav[d]);
    if (this.ref.a.some((x) => x !== 0)) {
      const vec = new Array(this.model.nv).fill(0);
      dofs.forEach((d, i) => vec[d] = this.ref.a[i]);
      this.mujoco.mj_mulM(this.model, this.data, this.mulRes, vec);
      const r = this.mulRes.GetView();
      dofs.forEach((d, i) => tau[i] = tau[i] + r[d]);
    }
    return tau;
  }
  servoHold(q) {
    this.ref = {
      q: Array.from(q),
      v: [0, 0, 0, 0, 0, 0],
      a: [0, 0, 0, 0, 0, 0]
    };
  }
  /** `ArmServo.start_step`: the reference velocity reaches v at the end of the step. */
  servoStartStep(v) {
    const T = this.substeps * this.h;
    this.ref.a = this.ref.v.map((x, i) => (v[i] - x) / T);
  }
  fingerCtrl(joints) {
    const ctrl = this.data.ctrl;
    for (let i = 0; i < NUM_FINGERS; i++) ctrl[i] = joints[i];
  }
  /** One MuJoCo step with the object parked (settle and phases 0-1 outside mjswan's loop: Node / parity). */
  parkedStep(joints) {
    this.fingerCtrl(joints);
    this.servoCtrl();
    this.mujoco.mj_step(this.model, this.data);
    this.clampFingerSpeed();
    this.parkObject();
  }
  /** Phase 2 from a floating-layout start (`MjArmHand21Env.reset(phases=False)`): place, settle, then set. */
  reset(qpos, qvel, qStart) {
    const fl = this.arm.floating;
    if (qpos.length !== fl.nq || qvel.length !== fl.nv)
      throw new Error("[mjweb] start state has the wrong size");
    const s = armStartOf(qpos, qvel, fl);
    const { mujoco, model, data } = this;
    mujoco.mj_resetData(model, data);
    const q = this.armStart(s.wristPos, s.wristQuat, qStart);
    const joints = this.clampJoints(s.joints);
    this.arm.meta.arm_qpos.forEach(
      (a, i) => data.qpos[a] = q[i]
    );
    this.L.fingerQpos.forEach(
      (a, i) => data.qpos[a] = joints[i]
    );
    this.parkObject();
    this.servoHold(q);
    mujoco.mj_forward(model, data);
    const n = Math.round(this.arm.settle_s / this.h);
    for (let k = 0; k < n; k++) this.parkedStep(joints);
    return this.enterPolicy(s, q);
  }
  /** The end of every reset: the hand joints, the object and the arm speeds as the harness sets them. */
  enterPolicy(s, q) {
    const { mujoco, model, data } = this;
    const qp = data.qpos;
    const qv = data.qvel;
    const joints = this.clampJoints(s.joints);
    this.L.fingerQpos.forEach((a, i) => qp[a] = joints[i]);
    this.L.fingerQvel.forEach((a, i) => qv[a] = s.jointVel[i]);
    this.fingerCtrl(joints);
    this.arm.meta.arm_qvel.forEach((a) => qv[a] = 0);
    for (let i = 0; i < 7; i++) qp[this.L.objectQpos + i] = s.objectQpos[i];
    for (let i = 0; i < 6; i++) qv[this.L.objectQvel + i] = s.objectQvel[i];
    mujoco.mj_kinematics(model, data);
    const rates = this.armRates(s.wristVelCom, s.wristOmega);
    let scale = 1;
    rates.forEach((v, i) => {
      scale = Math.max(scale, Math.abs(v) / this.arm.meta.vel_limits[i]);
    });
    const qd = rates.map((v) => v / scale);
    this.arm.meta.arm_qvel.forEach((a, i) => qv[a] = qd[i]);
    mujoco.mj_forward(model, data);
    this.ik.reset(q, q, qd);
    this.ref = { q: q.slice(), v: qd.slice(), a: [0, 0, 0, 0, 0, 0] };
    this.virtual.reset(s.wristPos, s.wristQuat, s.wristVelCom, s.wristOmega);
    this.goalTwist = null;
    this.filter.reset();
    this.contactWrench.fill(0);
    this.unstable = false;
    this.bad0 = this.badqacc();
    this.phase = "policy";
    this.goalPos = s.wristPos.slice();
    this.goalQuat = s.wristQuat.slice();
    const err = this.wristErr(s);
    Object.assign(this.log, {
      ikMm: 0,
      ikDeg: 0,
      ikMaxMm: 0,
      trackMm: 0,
      trackDeg: 0,
      speedFracMax: 0,
      ikSteps: 0,
      velBoundSteps: 0,
      armObjectSubsteps: 0,
      settleMm: err[0],
      settleDeg: err[1],
      goalClamped: 0,
      virtualResets: 0,
      steps: 0,
      firstGuard: null,
      rho: Number.NaN,
      sigma: Number.NaN,
      rhoMax: 0,
      sigmaMinMin: Infinity,
      garm6Flagged: 0,
      branchFlips: 0,
      qdRatioMax: 0,
      qdOverSubsteps: 0,
      accelBoundSteps: 0,
      accelRelaxed: 0,
      vdevMm: 0,
      vdevDeg: 0,
      vdevMaxMm: 0,
      vdevMaxDeg: 0
    });
    this.stopStreak = 0;
    this.signs = branchSigns(q, this.arm.meta.dh);
    this.ik.fallbacks = 0;
    this.ik.accelRelaxed = 0;
    this.log.speedFrac.fill(0);
    this.state = this.readState();
    this.readHandPoints();
    return this.state;
  }
  clampJoints(j) {
    const r = this.L.fingerRange;
    return Array.from(
      j,
      (v, i) => Math.min(Math.max(v, r[2 * i]), r[2 * i + 1])
    );
  }
  wristErr(s) {
    this.mujoco.mj_kinematics(this.model, this.data);
    const b = this.L.wristBody;
    const xpos = this.data.xpos;
    const xq = this.data.xquat;
    const q = [xq[4 * b], xq[4 * b + 1], xq[4 * b + 2], xq[4 * b + 3]];
    const dq = quatMul(quatConjugate(q), s.wristQuat);
    return [
      1e3 * Math.hypot(
        xpos[3 * b] - s.wristPos[0],
        xpos[3 * b + 1] - s.wristPos[1],
        xpos[3 * b + 2] - s.wristPos[2]
      ),
      2 * Math.acos(Math.min(1, Math.abs(dq[0]))) * 180 / Math.PI
    ];
  }
  armRates(vCom, omega) {
    if (!vCom.some((v) => v !== 0) && !omega.some((v) => v !== 0))
      return [0, 0, 0, 0, 0, 0];
    this.mujoco.mj_comPos(this.model, this.data);
    this.mujoco.mj_jacBodyCom(
      this.model,
      this.data,
      this.jacp,
      this.jacr,
      this.L.wristBody
    );
    const jp = this.jacp.GetView();
    const jr = this.jacr.GetView();
    const nv = this.model.nv;
    const dof = this.arm.meta.arm_qvel;
    const j = new Array(36);
    for (let r = 0; r < 3; r++)
      for (let k = 0; k < 6; k++) {
        j[6 * r + k] = jp[nv * r + dof[k]];
        j[6 * (r + 3) + k] = jr[nv * r + dof[k]];
      }
    const out = solveSquare(j, [...vCom, ...omega], 6);
    return out.every(Number.isFinite) ? out : [0, 0, 0, 0, 0, 0];
  }
  /** Phases 0-1 of the page from a floating-layout start: rest (fingers open, the object parked), then the approach,
   * the fingers, the settling; `phaseSubstep` drives them one MuJoCo step at a time inside mjswan's loop. */
  beginPhases(qpos, qvel, qStart, qPre, approach = "direct") {
    const s = armStartOf(qpos, qvel, this.arm.floating);
    const { mujoco, model, data } = this;
    mujoco.mj_resetData(model, data);
    this.qStart = this.armStart(s.wristPos, s.wristQuat, qStart);
    if (approach === "direct")
      this.plan = planDirect(this.qStart, this.arm.meta);
    else {
      const pre = this.armPreStart(s, this.qStart, qPre);
      this.plan = planApproach(this.qStart, pre, this.arm.meta);
    }
    this.pending = s;
    const qp = data.qpos;
    this.arm.meta.arm_qpos.forEach((a, i) => qp[a] = this.arm.meta.rest_q[i]);
    this.L.fingerQpos.forEach((a) => qp[a] = 0);
    this.parkObject();
    this.servoHold(this.arm.meta.rest_q);
    mujoco.mj_forward(model, data);
    this.phase = "rest";
    this.phaseT = 0;
    this.phaseSettle = 0;
    this.unstable = false;
    this.bad0 = this.badqacc();
    this.state = this.readState();
    this.readHandPoints();
  }
  /** The approach plan of the pending start (the panel's phase bar). */
  get approachPlan() {
    return this.plan;
  }
  get phaseTime() {
    return this.phaseT;
  }
  /** Total phase 1 time (approach + fingers), s. */
  /** Phase 1's time without the settling: the approach, plus the finger close-up after a two-segment one. */
  get phaseDuration() {
    if (!this.plan) return 0;
    return this.plan.t2 <= 0 ? this.plan.t1 : this.plan.t1 + this.plan.t2 + this.arm.finger_s;
  }
  startPhases() {
    if (this.phase === "rest") this.phase = "approach";
  }
  /** Before mj_step during phases 0-1 (substep s of mjswan's control step): servo and finger commands. */
  phasePreSubstep(s) {
    const p = this.plan;
    const dt = this.dt;
    if (this.phase === "approach" && p && s % this.substeps === 0) {
      const t = this.phaseT;
      const [q0] = sampleApproach(p, t);
      const [q1] = sampleApproach(p, t + dt);
      this.servoStartStep(q0.map((v, i) => (q1[i] - v) / dt));
    }
    let joints = new Array(NUM_FINGERS).fill(0);
    if (this.phase === "approach" && p && p.t2 <= 0 && this.pending) {
      const k = Math.floor(this.phaseT / dt + 1e-9) + 1;
      const sfrac = Math.min(1, k * dt / p.t1);
      const blend = 10 * sfrac ** 3 - 15 * sfrac ** 4 + 6 * sfrac ** 5;
      joints = this.clampJoints(this.pending.joints).map((v) => v * blend);
    } else if ((this.phase === "fingers" || this.phase === "settle") && this.pending) {
      const tf = this.phase === "settle" ? this.arm.finger_s : this.phaseT - (p ? p.t1 + p.t2 : 0);
      const nf = Math.round(this.arm.finger_s / dt);
      const k = Math.min(nf, Math.floor(tf / dt + 1e-9) + 1);
      const sfrac = Math.min(1, k / nf);
      const blend = 10 * sfrac ** 3 - 15 * sfrac ** 4 + 6 * sfrac ** 5;
      joints = this.clampJoints(this.pending.joints).map((v) => v * blend);
    }
    this.fingerCtrl(joints);
    this.servoCtrl();
  }
  /** After mj_step during phases 0-1: clamp, park the object, advance the phase clock per physics step. */
  phasePostSubstep(s) {
    this.clampFingerSpeed();
    this.parkObject();
    if (s % this.substeps !== this.substeps - 1) return;
    if (this.phase === "rest" || !this.plan || !this.pending) return;
    this.phaseT += this.dt;
    const p = this.plan;
    if (this.phase === "approach" && this.phaseT >= p.t1 + p.t2 - 1e-9) {
      this.servoHold(this.qStart);
      this.phase = p.t2 <= 0 ? "settle" : "fingers";
    } else if (this.phase === "fingers" && this.phaseT >= p.t1 + p.t2 + this.arm.finger_s - 1e-9) {
      this.phase = "settle";
    } else if (this.phase === "settle") {
      this.phaseSettle += this.dt;
    }
    if (this.phase === "settle" && s === this.decimation - 1) {
      const [mm2, deg2] = this.wristErr(this.pending);
      const qv = this.data.qvel;
      const still = this.arm.meta.arm_qvel.every((a) => Math.abs(qv[a]) < 1e-3);
      if (mm2 <= 0.1 && deg2 <= 0.05 && still || this.phaseSettle >= this.arm.settle_max_s) {
        this.enterPolicy(this.pending, this.qStart);
        this.pending = null;
      }
    }
  }
  /** Display state during the phases (fresh kinematics). */
  refreshDisplay() {
    this.mujoco.mj_kinematics(this.model, this.data);
    this.state = this.readState();
    this.readHandPoints();
  }
  // ------------------------------------------------------------------ one control step (phase 2)
  beginStep(y) {
    this.filter.process(y);
  }
  /** Composite rigid body of the hand (`sim_h21.composite`, the HandEnv21 summation orders). */
  composite() {
    const d = this.data;
    const xipos = d.xipos;
    const ximat = d.ximat;
    const inertiaDiag = this.model.body_inertia;
    const bodies = this.handBodies;
    const m = this.masses;
    const mass = pairwiseSum(m);
    const sum = [0, 0, 0];
    for (let k = 0; k < bodies.length; k++) {
      const b = bodies[k];
      for (let i = 0; i < 3; i++)
        sum[i] = k === 0 ? m[k] * xipos[3 * b + i] : sum[i] + m[k] * xipos[3 * b + i];
    }
    const c = [sum[0] / mass, sum[1] / mass, sum[2] / mass];
    const inertia = new Array(9).fill(0);
    for (let k = 0; k < bodies.length; k++) {
      const b = bodies[k];
      const r = ximat.subarray(9 * b, 9 * b + 9);
      const di = inertiaDiag.subarray(3 * b, 3 * b + 3);
      const dd = [
        xipos[3 * b] - c[0],
        xipos[3 * b + 1] - c[1],
        xipos[3 * b + 2] - c[2]
      ];
      const d2 = dd[0] * dd[0] + dd[1] * dd[1] + dd[2] * dd[2];
      for (let i = 0; i < 3; i++)
        for (let j = 0; j < 3; j++) {
          let rot = 0;
          for (let a = 0; a < 3; a++)
            rot += r[3 * i + a] * di[a] * r[3 * j + a];
          const v = rot + m[k] * ((i === j ? d2 : 0) - dd[i] * dd[j]);
          inertia[3 * i + j] = k === 0 ? v : inertia[3 * i + j] + v;
        }
    }
    return { mass, c, inertia };
  }
  /** The IK goal of the next physics step; admittance: advance the virtual wrist by one step first. */
  goal() {
    const f = this.filter;
    this.goalTwist = null;
    if (this.mode === "stiff") {
      this.goalPos = f.pos.slice();
      this.goalQuat = f.quat.slice();
      return this.clampGoal();
    }
    const { mujoco, model, data } = this;
    mujoco.mj_kinematics(model, data);
    const { mass, c, inertia } = this.composite();
    const w = this.L.wristBody;
    const xpos = data.xpos;
    const xq = data.xquat;
    const xr = [xpos[3 * w], xpos[3 * w + 1], xpos[3 * w + 2]];
    const rr = mat3([xq[4 * w], xq[4 * w + 1], xq[4 * w + 2], xq[4 * w + 3]]);
    const cOff = mtv(rr, [c[0] - xr[0], c[1] - xr[1], c[2] - xr[2]]);
    const rrT = [rr[0], rr[3], rr[6], rr[1], rr[4], rr[7], rr[2], rr[5], rr[8]];
    const iBody = conj3(rrT, inertia);
    const v = this.virtual;
    const rv = mat3(v.quat);
    const cvr = mv(rv, cOff);
    const cV = [0, 1, 2].map((i) => v.pos[i] + cvr[i]);
    const iWorld = conj3(rv, iBody);
    const ipos = this.model.body_ipos;
    const comW = mv(rv, [ipos[3 * w], ipos[3 * w + 1], ipos[3 * w + 2]]);
    const pApp = [0, 1, 2].map((i) => v.pos[i] + comW[i]);
    const vApp = cross(v.omega, [
      pApp[0] - cV[0],
      pApp[1] - cV[1],
      pApp[2] - cV[2]
    ]).map((x, i) => x + v.vel[i]);
    const [force, torque] = this.trainingWrench(
      v.pos,
      v.quat,
      vApp,
      v.omega,
      mass,
      cV,
      iWorld,
      pApp
    );
    const lever = cross(
      [pApp[0] - cV[0], pApp[1] - cV[1], pApp[2] - cV[2]],
      force
    );
    const tq = torque.map((x, i) => x + lever[i]);
    const fo = force.map((x, i) => x + mass * this.gravity[i]);
    const rrv = [0, 1, 2].flatMap(
      (i) => [0, 1, 2].map(
        (j) => rv[3 * i] * rr[3 * j] + rv[3 * i + 1] * rr[3 * j + 1] + rv[3 * i + 2] * rr[3 * j + 2]
      )
    );
    const ef = mv(rrv, [
      this.contactWrench[0],
      this.contactWrench[1],
      this.contactWrench[2]
    ]);
    const et = mv(rrv, [
      this.contactWrench[3],
      this.contactWrench[4],
      this.contactWrench[5]
    ]);
    v.advance(
      fo.map((x, i) => x + ef[i]),
      tq.map((x, i) => x + et[i]),
      mass,
      cOff,
      iBody,
      this.h,
      this.substeps
    );
    if (!v.finite()) {
      v.reset(
        xr,
        [xq[4 * w], xq[4 * w + 1], xq[4 * w + 2], xq[4 * w + 3]],
        [0, 0, 0],
        [0, 0, 0]
      );
      this.log.virtualResets += 1;
      this.guard("virtual_reset");
    }
    const w1 = cross(v.omega, mv(mat3(v.quat), cOff));
    this.goalTwist = [0, 1, 2].map((i) => v.vel[i] - w1[i]).concat(v.omega);
    this.goalPos = v.pos.slice();
    this.goalQuat = v.quat.slice();
    this.clampGoal();
  }
  /** A guard fired: the episode ends as unstable at this control step's end (`env.MjArmHand21Env._guard`). */
  guard(kind) {
    this.unstable = true;
    if (!this.log.firstGuard)
      this.log.firstGuard = { kind, step: this.log.steps };
  }
  /** Deployment safety: the goal stays within `goal_radius_m` of the shoulder (beyond is out of reach anyway). */
  clampGoal() {
    if (this.forceFarGoal) {
      this.forceFarGoal = false;
      this.goalPos = [this.goalPos[0] + 10, this.goalPos[1], this.goalPos[2]];
    }
    const T = this.baseFrame();
    const sh = [
      T[3] + T[2] * this.arm.meta.dh.d[0],
      T[7] + T[6] * this.arm.meta.dh.d[0],
      T[11] + T[10] * this.arm.meta.dh.d[0]
    ];
    const d = [0, 1, 2].map((i) => this.goalPos[i] - sh[i]);
    const n = Math.hypot(d[0], d[1], d[2]);
    const r = this.arm.goal_radius_m;
    if (!(n <= r) || !this.goalQuat.every(Number.isFinite)) {
      this.log.goalClamped += 1;
      this.goalTwist = null;
      this.guard("goal_clamp");
      if (!Number.isFinite(n) || !this.goalQuat.every(Number.isFinite)) {
        const p = this.ik.wristPose();
        this.goalPos = p.pos;
        this.goalQuat = p.quat;
      } else this.goalPos = [0, 1, 2].map((i) => sh[i] + d[i] * r / n);
    }
  }
  /** `sim_h21.wrench_actions_py` (gains of the scene, no DR). */
  trainingWrench(pos, quat, vel, omega, mass, c, inertia, ffPoint) {
    const f = this.filter;
    const [kpP, kdP, kpR, kdR] = this.gains;
    const dt = this.dt;
    const qErr = quatPositiveReal(quatMul(f.quat, quatConjugate(quat)));
    const k = kdP * mass / (mass + kdP * dt);
    const force = [0, 1, 2].map(
      (i) => kpP * (f.pos[i] - pos[i]) + k * (f.ffLin[i] - vel[i])
    );
    const a = inertia.map((v, i) => i % 4 === 0 ? v + kdR * dt : v);
    const y = solve3(a, [
      f.ffAng[0] - omega[0],
      f.ffAng[1] - omega[1],
      f.ffAng[2] - omega[2]
    ]);
    const aa = axisAngleFromQuat(qErr);
    const iy = mv(inertia, y);
    const torque = [0, 1, 2].map((i) => kpR * aa[i] + kdR * iy[i]);
    const fg = [
      -mass * this.gravity[0],
      -mass * this.gravity[1],
      -mass * this.gravity[2]
    ];
    const tg = cross(
      [c[0] - ffPoint[0], c[1] - ffPoint[1], c[2] - ffPoint[2]],
      fg
    );
    return [force.map((v, i) => v + fg[i]), torque.map((v, i) => v + tg[i])];
  }
  preSubstep(s) {
    if (this.phase !== "policy") return this.phasePreSubstep(s);
    if (s % this.substeps === 0) {
      this.goal();
      const { q0, v } = this.ik.step(
        this.goalPos,
        this.goalQuat,
        this.goalTwist ?? void 0
      );
      const [mm2, deg2] = this.ik.residual(this.goalPos, this.goalQuat);
      this.log.ikMm = mm2;
      this.log.ikDeg = deg2;
      this.log.ikMaxMm = Math.max(this.log.ikMaxMm, mm2);
      let bound = false;
      v.forEach((x, i) => {
        const fr = Math.abs(x) / this.arm.meta.vel_limits[i];
        this.log.speedFrac[i] = fr;
        this.log.speedFracMax = Math.max(this.log.speedFracMax, fr);
        if (Math.abs(x) >= (1 - 1e-6) * this.ik.vmax[i]) bound = true;
      });
      this.log.ikSteps += 1;
      if (bound) this.log.velBoundSteps += 1;
      if (this.ik.accelBound) this.log.accelBoundSteps += 1;
      this.log.accelRelaxed = this.ik.accelRelaxed;
      if (this.mode === "admittance") this.measureVirtualDeviation();
      const amp = this.ik.amplification(q0, this.ik.q);
      const lim = this.arm.meta.garm6 ?? { rho_max: 25, sigma_min: 0.03 };
      this.log.rho = amp.rho;
      this.log.sigma = amp.sigma;
      if (Number.isFinite(amp.rho))
        this.log.rhoMax = Math.max(this.log.rhoMax, amp.rho);
      this.log.sigmaMinMin = Math.min(this.log.sigmaMinMin, amp.sigma);
      if (amp.rho > lim.rho_max || amp.sigma < lim.sigma_min)
        this.log.garm6Flagged += 1;
      if (amp.sigma < lim.sigma_min) this.guard("singular");
      const signs = branchSigns(this.ik.q, this.arm.meta.dh);
      if (signs.some((x, i) => x !== this.signs[i])) this.log.branchFlips += 1;
      this.signs = signs;
      this.servoStartStep(v);
      this.acc.fill(0);
    }
    this.fingerCtrl(this.filter.joints);
    this.servoCtrl();
  }
  postSubstep(s) {
    if (this.phase !== "policy") return this.phasePostSubstep(s);
    this.clampFingerSpeed();
    this.measureContacts();
    this.measureSpeed();
    if (s % this.substeps === this.substeps - 1)
      for (let i = 0; i < 6; i++)
        this.contactWrench[i] = this.acc[i] / this.substeps;
    if (s === this.decimation - 1) {
      this.log.steps += 1;
      if (this.ik.fallbacks > 0) this.guard("ik_no_step");
      if (this.badqacc() > this.bad0) this.guard("mujoco_reset");
      this.mujoco.mj_kinematics(this.model, this.data);
      this.state = this.readState();
      this.readHandPoints();
      const f = this.filter;
      const st = this.state;
      this.log.trackMm = 1e3 * Math.hypot(
        st.wristPos[0] - f.pos[0],
        st.wristPos[1] - f.pos[1],
        st.wristPos[2] - f.pos[2]
      );
      const dq = quatMul(
        [f.quat[0], -f.quat[1], -f.quat[2], -f.quat[3]],
        st.wristQuat
      );
      this.log.trackDeg = 2 * Math.acos(Math.min(1, Math.abs(dq[0]))) * 180 / Math.PI;
    }
  }
  /** `env.step`'s tracking cost in admittance: the real r_wrist against the virtual wrist at an IK step. */
  measureVirtualDeviation() {
    const b = this.arm.meta.wrist_body;
    const xpos = this.data.xpos;
    const xquat = this.data.xquat;
    const v = this.virtual;
    const mm2 = 1e3 * Math.hypot(
      v.pos[0] - xpos[3 * b],
      v.pos[1] - xpos[3 * b + 1],
      v.pos[2] - xpos[3 * b + 2]
    );
    let dot = 0;
    for (let i = 0; i < 4; i++) dot += v.quat[i] * xquat[4 * b + i];
    const deg2 = 360 / Math.PI * Math.acos(Math.min(1, Math.abs(dot)));
    this.log.vdevMm = mm2;
    this.log.vdevDeg = deg2;
    this.log.vdevMaxMm = Math.max(this.log.vdevMaxMm, mm2);
    this.log.vdevMaxDeg = Math.max(this.log.vdevMaxDeg, deg2);
  }
  /** `env._measure_speed`: the arm's measured joint speeds against UR's limits after a MuJoCo step; the protective
   *  stop once they stay above `speed_stop.ratio` for `speed_stop.substeps` consecutive steps. */
  measureSpeed() {
    const qv = this.data.qvel;
    const m = this.arm.meta;
    let r = 0;
    m.arm_qvel.forEach((a, i) => {
      r = Math.max(r, Math.abs(qv[a]) / m.vel_limits[i]);
    });
    this.log.qdRatioMax = Math.max(this.log.qdRatioMax, r);
    if (r > 1) this.log.qdOverSubsteps += 1;
    const stop = m.speed_stop ?? SPEED_STOP_DEFAULT;
    this.stopStreak = r > stop.ratio ? this.stopStreak + 1 : 0;
    if (this.stopStreak >= stop.substeps) this.guard("protective_stop");
  }
  /** The contact wrench on the hand (admittance input) and the arm-object contact count, after mj_step. */
  measureContacts() {
    const { mujoco, model, data } = this;
    const ncon = data.ncon;
    if (ncon === 0) return;
    const obj = this.L.objectBody;
    let c = null;
    let armObj = false;
    for (let i = 0; i < ncon; i++) {
      const con = data.contact.get(i);
      if (!con) continue;
      const gb = model.geom_bodyid;
      const b1 = gb[con.geom1];
      const b2 = gb[con.geom2];
      if (this.armSet.has(b1) && b2 === obj || this.armSet.has(b2) && b1 === obj)
        armObj = true;
      const in1 = this.handSet.has(b1);
      const in2 = this.handSet.has(b2);
      if (this.mode === "admittance" && in1 !== in2) {
        if (!c) c = this.composite().c;
        mujoco.mj_contactForce(model, data, i, this.f6);
        const f = this.f6.GetView();
        const fr = con.frame;
        let fw = [0, 1, 2].map(
          (k) => fr[k] * f[0] + fr[3 + k] * f[1] + fr[6 + k] * f[2]
        );
        if (in1) fw = fw.map((x) => -x);
        const pos = con.pos;
        const t = cross([pos[0] - c[0], pos[1] - c[1], pos[2] - c[2]], fw);
        for (let k = 0; k < 3; k++) {
          this.acc[k] += fw[k];
          this.acc[3 + k] += t[k];
        }
      }
      con.delete();
    }
    if (armObj) this.log.armObjectSubsteps += 1;
  }
  /** The whole control step with MuJoCo's own `mj_step` (Node / parity; mjswan steps in the browser). */
  step(y) {
    this.beginStep(y);
    for (let s = 0; s < this.decimation; s++) {
      this.preSubstep(s);
      this.mujoco.mj_step(this.model, this.data);
      this.postSubstep(s);
    }
    return this.state;
  }
  removeWrench() {
  }
  clampFingerSpeed() {
    const qvel = this.data.qvel;
    const dof = this.L.fingerQvel;
    for (let i = 0; i < NUM_FINGERS; i++) {
      const v = qvel[dof[i]];
      const m = this.vmax[i];
      qvel[dof[i]] = v < -m ? -m : v > m ? m : v;
    }
  }
  /** Arm joint positions (rad) and speeds as fractions of UR's limits (the panel's bars). */
  armJoints() {
    const qp = this.data.qpos;
    const qv = this.data.qvel;
    return {
      q: this.arm.meta.arm_qpos.map((a) => qp[a]),
      qd: this.arm.meta.arm_qvel.map(
        (a, i) => qv[a] / this.arm.meta.vel_limits[i]
      )
    };
  }
  readState() {
    const d = this.data;
    const L = this.L;
    const xpos = d.xpos;
    const xquat = d.xquat;
    const qpos = d.qpos;
    const w = L.wristBody;
    const o = L.objectBody;
    return {
      wristPos: [xpos[3 * w], xpos[3 * w + 1], xpos[3 * w + 2]],
      wristQuat: [
        xquat[4 * w],
        xquat[4 * w + 1],
        xquat[4 * w + 2],
        xquat[4 * w + 3]
      ],
      jointPos: Array.from(L.fingerQpos, (a) => qpos[a]),
      objectPos: [xpos[3 * o], xpos[3 * o + 1], xpos[3 * o + 2]],
      objectQuat: [
        xquat[4 * o],
        xquat[4 * o + 1],
        xquat[4 * o + 2],
        xquat[4 * o + 3]
      ]
    };
  }
  readHandPoints() {
    const xpos = this.data.xpos;
    const b = this.L.handPointBodies;
    for (let i = 0; i < b.length; i++)
      for (let k = 0; k < 3; k++)
        this.handPoints[3 * i + k] = xpos[3 * b[i] + k];
  }
};
function solveSquare(a, b, n) {
  const m = a.slice();
  const r = b.slice();
  for (let k = 0; k < n; k++) {
    let p = k;
    for (let i = k + 1; i < n; i++)
      if (Math.abs(m[n * i + k]) > Math.abs(m[n * p + k])) p = i;
    if (m[n * p + k] === 0) return new Array(n).fill(NaN);
    for (let j = 0; j < n; j++)
      [m[n * k + j], m[n * p + j]] = [m[n * p + j], m[n * k + j]];
    [r[k], r[p]] = [r[p], r[k]];
    for (let i = k + 1; i < n; i++) {
      const f = m[n * i + k] / m[n * k + k];
      for (let j = k; j < n; j++) m[n * i + j] -= f * m[n * k + j];
      r[i] -= f * r[k];
    }
  }
  for (let k = n - 1; k >= 0; k--) {
    let s = r[k];
    for (let j = k + 1; j < n; j++) s -= m[n * k + j] * r[j];
    r[k] = s / m[n * k + k];
  }
  return r;
}

// ../../../../../../web/plugins/arm_episode.ts
function armLayoutOf(mujoco, model, jointOrder) {
  const id2 = (t, n) => {
    const i = mujoco.mj_name2id(model, t, n);
    if (i < 0) throw new Error(`[mjweb] "${n}" not in the arm model`);
    return i;
  };
  const J = mujoco.mjtObj.mjOBJ_JOINT.value;
  const B = mujoco.mjtObj.mjOBJ_BODY.value;
  const A = mujoco.mjtObj.mjOBJ_ACTUATOR.value;
  const qadr = model.jnt_qposadr;
  const vadr = model.jnt_dofadr;
  const range = model.jnt_range;
  const jbody = model.jnt_bodyid;
  const finger = jointOrder.map((n) => id2(J, n));
  jointOrder.forEach((n, i) => {
    if (id2(A, `act_${n}`) !== i)
      throw new Error(`[mjweb] actuator act_${n} must have id ${i}`);
  });
  const fingerRange = new Float64Array(2 * finger.length);
  finger.forEach((j, i) => {
    fingerRange[2 * i] = range[2 * j];
    fingerRange[2 * i + 1] = range[2 * j + 1];
  });
  const oj = id2(J, "object_free");
  const wristBody = id2(B, WRIST_BODY);
  return {
    wristBody,
    objectBody: id2(B, OBJECT_BODY),
    wristQpos: -1,
    wristQvel: -1,
    objectQpos: qadr[oj],
    objectQvel: vadr[oj],
    fingerQpos: Int32Array.from(finger.map((j) => qadr[j])),
    fingerQvel: Int32Array.from(finger.map((j) => vadr[j])),
    fingerRange,
    handPointBodies: Int32Array.from([
      wristBody,
      ...finger.map((j) => jbody[j]),
      ...TIP_LINKS.map((n) => id2(B, n))
    ])
  };
}
function startLayoutOf(arm, armL) {
  const f = arm.floating;
  return {
    ...armL,
    wristQpos: f.wrist_qpos,
    wristQvel: f.wrist_qvel,
    objectQpos: f.object_qpos,
    objectQvel: f.object_qvel,
    fingerQpos: Int32Array.from(f.finger_qpos),
    fingerQvel: Int32Array.from(f.finger_qvel)
  };
}
var ArmEpisode = class extends Episode {
  arm;
  armSettings;
  /** Phases 0-1 are running (the evaluated episode has not started). */
  inPhases = false;
  /** The current control step drives phases 0-1. */
  phaseStep = false;
  pendingQpos = null;
  pendingQvel = null;
  constructor(mujoco, model, data, wd, policyId, settings, arm, armSettings) {
    const L = armLayoutOf(mujoco, model, wd.controller.joint_order);
    super(mujoco, model, data, wd, policyId, settings, "isaac", {
      layout: L,
      startLayout: startLayoutOf(arm, L),
      makeEnv: (layout, aw, af) => new ArmEnv(
        mujoco,
        model,
        data,
        layout,
        wd.controller,
        arm,
        aw,
        af,
        armSettings.mode
      )
    });
    this.arm = arm;
    this.armSettings = { ...armSettings };
  }
  get armEnv() {
    return this.env;
  }
  /** The harness's start / pre-start configurations of the selected start, while the base is exactly at the
   *  selected kind's placement and the start is unperturbed (otherwise the browser computes them). */
  precomputed() {
    const k = this.arm.placements[this.armSettings.kind];
    const exact = k !== void 0 && this.armEnv.placementMatches(k.placement) && !this.perturbation && this.startIndex >= 0;
    return exact ? k.starts[this.startIndex] ?? {} : {};
  }
  resetEnv(qpos, qvel) {
    const env = this.armEnv;
    env.mode = this.armSettings.mode;
    const pre = this.precomputed();
    if (this.armSettings.phases && this.policy) {
      env.beginPhases(
        qpos,
        qvel,
        pre.q_start,
        pre.q_pre,
        pre.approach ?? "direct"
      );
      this.inPhases = true;
      this.pendingQpos = Array.from(qpos);
      this.pendingQvel = Array.from(qvel);
      return env.state;
    }
    this.inPhases = false;
    return env.reset(qpos, qvel, pre.q_start);
  }
  /** Play: start phase 1 (the approach) if the episode waits at rest. */
  startPhases() {
    if (this.inPhases) this.armEnv.startPhases();
  }
  observe() {
    if (this.inPhases) return new Float32Array(201);
    return super.observe();
  }
  beginStep(action) {
    if (this.inPhases) {
      this.phaseStep = true;
      return;
    }
    this.phaseStep = false;
    super.beginStep(action);
  }
  preSubstep(s) {
    if (this.phaseStep) this.armEnv.preSubstep(s);
    else super.preSubstep(s);
  }
  postSubstep(s) {
    if (this.phaseStep) this.armEnv.postSubstep(s);
    else super.postSubstep(s);
  }
  endStep() {
    if (!this.phaseStep) return super.endStep();
    this.phaseStep = false;
    const env = this.armEnv;
    env.refreshDisplay();
    if (env.phase === "policy" && this.inPhases) {
      this.inPhases = false;
      const s = env.state;
      const noise = this.settings.sense ? this.wd.sense ?? EVAL_SENSE : EXACT;
      this.view = new StudentView(
        noise,
        this.settings.sense ? new Rng(this.settings.seed, 1) : null
      );
      this.startMetrics = this.metrics.sample(
        s,
        env.handPoints,
        this.refPoseAt(this.t)
      );
      this.view.push(this.view.measure(s, measuredTarget(s)));
      this.record = [];
    }
    return "ignored";
  }
  /** The reference row at the clock (the parent's private `refPose`). */
  refPoseAt(t) {
    const r = Math.max(0, Math.min(t, this.length - 1));
    const ref = this.ref;
    return {
      wristPos: ref.wrist_pos[r],
      wristQuat: ref.wrist_quat[r],
      objectPos: ref.object_pos[r],
      objectQuat: ref.object_quat[r]
    };
  }
  /** The guard that ended the episode (`guardText`), or null. */
  get unstableReason() {
    const g = this.armEnv.log.firstGuard;
    return g ? guardText(g.kind, this.armEnv.arm.meta) : null;
  }
  /** The panel's phase label. */
  get phaseLabel() {
    const p = this.armEnv.phase;
    return p === "rest" ? "\u9759\u6B62\u4F4D" : p === "policy" ? "\u7B56\u7565\u63A7\u5236" : "\u79FB\u5230\u8D77\u59CB\u72B6\u6001";
  }
  /** The start's qpos / qvel the phases are heading to (null outside phases 0-1). */
  get pendingStart() {
    return this.inPhases && this.pendingQpos && this.pendingQvel ? { qpos: this.pendingQpos, qvel: this.pendingQvel } : null;
  }
};

// ../../../../../../web/plugins/arm_panel.ts
var THREE2 = __toESM(require_plugin_three_shim());
var KIND_LABELS = {
  best: "\u6BCF\u7269\u4F53\u6700\u4F18",
  behind: "\u6BCF\u7269\u4F53\xB7\u81C2\u5728\u624B\u540E",
  common: "\u5171\u7528"
};
var ARM_PANEL_ID = "mjweb-arm-panel";
var P = `#${ARM_PANEL_ID}`;
var CSS2 = `
${P} { position: fixed; right: 12px; top: 12px; z-index: 1000; width: 352px; max-height: calc(100vh - 24px);
  overflow-y: auto; padding: 10px 12px; border-radius: 10px; background: rgba(16, 19, 26, 0.86); color: #eef1f6;
  font: 12.5px/1.45 -apple-system, "PingFang SC", "Hiragino Sans GB", "Noto Sans SC", "Microsoft YaHei", sans-serif;
  box-shadow: 0 4px 18px rgba(0,0,0,0.35); user-select: none; }
${P} h2 { font-size: 13.5px; margin: 0 0 4px; font-weight: 650; }
${P} .row { display: flex; align-items: center; gap: 6px; margin: 3px 0; flex-wrap: wrap; }
${P} .lab { width: 44px; flex: none; color: #9aa6b8; align-self: flex-start; }
${P} button { background: #2a3140; color: #eef1f6; border: 1px solid #3b4558; border-radius: 6px;
  padding: 2px 7px; font: inherit; cursor: pointer; }
${P} button:hover { background: #343d50; }
${P} button.on { background: #2f6fde; border-color: #4a86f0; }
${P} button:disabled { opacity: 0.45; cursor: default; }
${P} select { background: #222937; color: #eef1f6; border: 1px solid #3b4558; border-radius: 6px;
  padding: 2px 4px; font: inherit; }
${P} input[type=number] { width: 58px; background: #222937; color: #eef1f6; border: 1px solid #3b4558;
  border-radius: 6px; padding: 1px 4px; font: inherit; }
${P} label { display: inline-flex; align-items: center; gap: 4px; cursor: pointer; }
${P} .sub { color: #9aa6b8; font-size: 11.5px; }
${P} .dim { color: #9aa6b8; }
${P} .live div.bad { color: #ff6b6b; }
${P} .live { margin-top: 8px; padding-top: 6px; border-top: 1px solid #333c4d; font-variant-numeric: tabular-nums; }
${P} .live div { margin: 1px 0; }
${P} .phase { display: inline-block; padding: 1px 6px; border-radius: 6px; background: #222937; color: #9aa6b8;
  margin-right: 2px; }
${P} .phase.on { background: #2f6fde; color: #fff; }
${P} .bar { position: relative; height: 7px; background: #222937; border-radius: 3px; flex: 1; min-width: 60px;
  overflow: hidden; }
${P} .bar i { position: absolute; top: 0; bottom: 0; background: #4a86f0; }
${P} .bar i.warn { background: #ffb347; } ${P} .bar i.bad { background: #ff6b6b; }
${P} .jrow { display: flex; align-items: center; gap: 5px; margin: 1px 0; font-variant-numeric: tabular-nums; }
${P} .jrow .jn { width: 40px; color: #9aa6b8; flex: none; }
${P} .jrow .jv { width: 70px; text-align: right; flex: none; }
${P} .legend { font-size: 11.5px; color: #9aa6b8; }
`;
var JOINT_ZH = ["\u5E95\u5EA7", "\u80A9", "\u8098", "\u8155 1", "\u8155 2", "\u8155 3"];
function el2(tag, attrs = {}, text) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (text !== void 0) e.textContent = text;
  return e;
}
var ArmPanel = class {
  root;
  style;
  state;
  cb;
  live = {};
  phaseEls = [];
  bars = [];
  inputs = {};
  lastPaint = 0;
  constructor(state, cb) {
    this.state = { ...state };
    this.cb = cb;
    document.getElementById("mjweb-arm-style")?.remove();
    this.style = el2("style", { id: "mjweb-arm-style" });
    this.style.textContent = CSS2;
    document.head.appendChild(this.style);
    document.getElementById(ARM_PANEL_ID)?.remove();
    const root = el2("div", { id: ARM_PANEL_ID, "data-testid": "mjweb-arm" });
    this.root = root;
    root.append(
      el2("h2", {}, "\u673A\u68B0\u81C2\u90E8\u7F72 \xB7 UR10e + Wuji Hand 2.1"),
      el2(
        "div",
        { class: "sub" },
        "Hand 2.1 \u88C5\u5728 UR10e \u7684\u6CD5\u5170\u4E0A\uFF1B\u673A\u68B0\u81C2\u7528 IK \u5B9E\u73B0\u5B66\u751F\u7684\u624B\u8155\u6307\u4EE4\uFF0C\u624B\u6307\u7528\u624B\u81EA\u5DF1\u7684\u6267\u884C\u5668"
      )
    );
    const baseRow = this.row("\u5E95\u5EA7");
    for (const [b, label] of [
      ["float", "\u6D6E\u52A8\u624B"],
      ["arm", "UR10e \u673A\u68B0\u81C2 + Hand 2.1"]
    ]) {
      const btn = el2(
        "button",
        { type: "button", "data-testid": `mjweb-base-${b}` },
        label
      );
      if (b === state.base) btn.classList.add("on");
      btn.disabled = !state.hasPair && b !== state.base;
      btn.onclick = () => b !== this.state.base && this.cb.setBase(b);
      baseRow.append(btn);
    }
    if (state.base === "arm") {
      const modeRow = this.row("\u8DDF\u8E2A");
      for (const [m, label] of [
        ["stiff", "\u521A\u6027\u8DDF\u8E2A"],
        ["admittance", "\u5BFC\u7EB3\uFF08\u7B49\u6548\u8BAD\u7EC3\uFF09"]
      ]) {
        const btn = el2(
          "button",
          { type: "button", "data-testid": `mjweb-mode-${m}` },
          label
        );
        if (m === state.mode) btn.classList.add("on");
        btn.onclick = () => m !== this.state.mode && this.cb.setMode(m);
        modeRow.append(btn);
      }
      root.append(
        el2(
          "div",
          { class: "sub" },
          state.mode === "stiff" ? "\u521A\u6027\uFF1AIK \u7684\u8F93\u5165\u5C31\u662F\u5B66\u751F\u7684\u624B\u8155\u6307\u4EE4\uFF08\u6BCF\u4E2A\u63A7\u5236\u6B65\u4FDD\u6301 4 \u4E2A\u7269\u7406\u6B65\uFF09\uFF0C\u624B\u8155\u6CA1\u6709\u8BAD\u7EC3\u91CC\u7684\u6EDE\u540E\u548C\u4E0B\u5782" : "\u5BFC\u7EB3\uFF1AIK \u7684\u8F93\u5165\u662F\u4E00\u4E2A\u6309\u8BAD\u7EC3\u91CC\u7684\u624B\u8155\u5B9A\u5F8B\uFF08\u540C\u6837\u7684\u589E\u76CA\u3001\u590D\u5408\u521A\u4F53\u3001\u9690\u5F0F\u963B\u5C3C\u3001\u91CD\u91CF\u524D\u9988\uFF09\u79EF\u5206\u7684\u865A\u62DF\u6D6E\u52A8\u624B\u8155\uFF0C\u53D7\u624B\u4E0A\u5B9E\u6D4B\u7684\u63A5\u89E6\u529B\u9A71\u52A8\uFF08\u771F\u673A\u4E0A\u662F\u6CD5\u5170\u529B\u4F20\u611F\u5668\u51CF\u53BB\u624B\u7684\u91CD\u91CF\uFF09"
        )
      );
      const phaseRow = this.row("\u9636\u6BB5");
      for (const [i, label] of [
        "\u9759\u6B62\u4F4D",
        "\u79FB\u5230\u8D77\u59CB\u72B6\u6001",
        "\u7B56\u7565\u63A7\u5236"
      ].entries()) {
        const s = el2(
          "span",
          { class: "phase", "data-testid": `mjweb-phase-${i}` },
          label
        );
        this.phaseEls.push(s);
        phaseRow.append(s);
        if (i < 2) phaseRow.append(el2("span", { class: "dim" }, "\u2192"));
      }
      const ph = el2("label", {});
      const box = el2("input", {
        type: "checkbox",
        "data-testid": "mjweb-phases"
      });
      box.checked = state.phases;
      box.onchange = () => this.cb.setPhases(box.checked);
      ph.append(box, "\u64AD\u653E\u524D\u4E24\u4E2A\u9636\u6BB5");
      phaseRow.append(ph);
      const og = el2("label", {});
      const obox = el2("input", {
        type: "checkbox",
        "data-testid": "mjweb-object-ghost"
      });
      obox.checked = state.objectGhost;
      obox.onchange = () => this.cb.setObjectGhost(obox.checked);
      og.append(obox, "\u63A5\u8FD1\u65F6\u663E\u793A\u7269\u4F53\u6B8B\u5F71\uFF08\u4E0D\u53C2\u4E0E\u78B0\u649E\uFF09");
      this.row("").append(og);
      const wf = el2("label", {});
      const wbox = el2("input", {
        type: "checkbox",
        "data-testid": "mjweb-wframes"
      });
      wbox.checked = state.wristFrames;
      wbox.onchange = () => this.cb.setWristFrames(wbox.checked);
      wf.append(wbox, "\u624B\u8155\u5750\u6807\u8F74\uFF1AIK \u7684\u8F93\u5165\uFF08\u9EC4\uFF09/ \u5B9E\u9645 r_wrist\uFF08\u7EA2\u7EFF\u84DD\uFF09");
      this.row("").append(wf);
      this.live.phase = el2("div", {
        class: "sub",
        "data-testid": "mjweb-phase-note"
      });
      root.append(this.live.phase);
      root.append(
        el2(
          "div",
          { class: "sub" },
          "\u9759\u6B62\u4F4D\u548C\u79FB\u5230\u8D77\u59CB\u72B6\u6001\u4E0D\u5C5E\u4E8E\u8BC4\u6D4B\u7684 episode\uFF1A\u5230\u4E86\u8D77\u59CB\u72B6\u6001\u3001\u673A\u68B0\u81C2\u7A33\u5B9A\u540E\uFF08\u2264 0.1 mm / 0.05\xB0\uFF09\uFF0C\u7269\u4F53\u548C\u624B\u6307\u6309\u6D6E\u52A8\u624B\u8BC4\u6D4B\u7684\u65B9\u5F0F\u8BBE\u6210\u8D77\u59CB\u72B6\u6001\uFF0C\u5B66\u751F\u624D\u5F00\u59CB\u3002"
        )
      );
      this.buildPlacement();
      const live = el2("div", {
        class: "live",
        "data-testid": "mjweb-arm-live"
      });
      for (const k of [
        "guard",
        "ik",
        "track",
        "wrench",
        "contacts",
        "speed",
        "garm6"
      ]) {
        this.live[k] = el2("div", { "data-testid": `mjweb-arm-${k}` });
        live.append(this.live[k]);
      }
      for (let i = 0; i < 6; i++) {
        const r = el2("div", { class: "jrow" });
        r.append(el2("span", { class: "jn" }, JOINT_ZH[i]));
        const pos = el2("div", {
          class: "bar",
          title: "\u5173\u8282\u4F4D\u7F6E\u5728 UR \u7684\u8303\u56F4\u91CC"
        });
        const pi = el2("i", {});
        pos.append(pi);
        const vel = el2("div", {
          class: "bar",
          title: "\u5173\u8282\u901F\u5EA6 / UR \u7684\u901F\u5EA6\u4E0A\u9650"
        });
        const vi = el2("i", {});
        vel.append(vi);
        const val = el2("span", { class: "jv" });
        r.append(pos, vel, val);
        live.append(r);
        this.bars.push({ pos: pi, vel: vi, val });
      }
      live.append(
        el2(
          "div",
          { class: "legend" },
          "\u5DE6\u6761\uFF1A\u5173\u8282\u4F4D\u7F6E\uFF08UR \u8303\u56F4\uFF09\uFF1B\u53F3\u6761\uFF1A\u901F\u5EA6 / \u4E0A\u9650\u3002\u573A\u666F\u91CC\u9EC4\u8272\u7C97\u5750\u6807\u8F74 = IK \u7684\u8F93\u5165\uFF08\u521A\u6027\uFF1A\u5B66\u751F\u7684\u624B\u8155\u6307\u4EE4\uFF1B\u5BFC\u7EB3\uFF1A\u865A\u62DF\u624B\u8155\uFF09\uFF0C\u7EA2\u7EFF\u84DD\u7EC6\u5750\u6807\u8F74 = \u5B9E\u9645 r_wrist\u3002"
        )
      );
      root.append(live);
    }
    document.body.append(root);
  }
  row(label) {
    const r = el2("div", { class: "row" });
    r.append(el2("span", { class: "lab" }, label));
    this.root.append(r);
    return r;
  }
  buildPlacement() {
    const p = this.state.placement;
    if (!p) return;
    const kr = this.row("\u653E\u7F6E");
    for (const k of ["best", "behind", "common"]) {
      const btn = el2(
        "button",
        { type: "button", "data-testid": `mjweb-place-${k}` },
        KIND_LABELS[k]
      );
      if (k === this.state.kind) btn.classList.add("on");
      btn.disabled = !this.state.kinds[k];
      btn.onclick = () => this.cb.setKind(k);
      kr.append(btn);
    }
    const r = this.row("");
    const num = (key, label, v, step) => {
      const box = el2("label", {});
      const i = el2("input", {
        type: "number",
        step,
        "data-testid": `mjweb-place-${key}`
      });
      i.value = String(Math.round(v * 1e3) / 1e3);
      this.inputs[key] = i;
      box.append(label, i);
      r.append(box);
    };
    num("bx", "x", p.x, "0.05");
    num("by", "y", p.y, "0.05");
    num("bz", "z", p.z, "0.05");
    num("byaw", "yaw\xB0", p.yaw_deg, "5");
    const r2 = this.row("");
    const lab = el2("label", {}, "\u5B89\u88C5 yaw");
    const sel = el2("select", { "data-testid": "mjweb-place-myaw" });
    for (const y of [0, 90, 180, 270]) {
      const o = el2("option", { value: String(y) }, `${y}\xB0`);
      if (y === Math.round(p.mount_yaw_deg)) o.selected = true;
      sel.append(o);
    }
    this.inputs.myaw = sel;
    lab.append(sel);
    r2.append(lab);
    const apply = el2(
      "button",
      { type: "button", "data-testid": "mjweb-place-apply" },
      "\u5E94\u7528"
    );
    apply.onclick = () => this.cb.setPlacement(this.readPlacement());
    const reset = el2(
      "button",
      { type: "button", "data-testid": "mjweb-place-default" },
      "\u9ED8\u8BA4"
    );
    reset.onclick = () => this.cb.setKind(this.state.kind);
    r2.append(apply, reset);
    const entry = this.state.kinds[this.state.kind];
    const custom = entry !== void 0 && ["x", "y", "z", "yaw_deg", "mount_yaw_deg"].some(
      (k) => Math.abs(entry.placement[k] - p[k]) > 1e-9
    );
    const behind = entry?.behind_m === void 0 || custom ? "" : `\u5E95\u5EA7\u5728\u6700\u8FD1\u7684\u624B\u8155\u4F4D\u7F6E\u540E\u65B9 ${entry.behind_m.toFixed(2)} m\uFF08\u6CBF\u624B\u7684\u5E73\u5747\u6307\u5411\uFF1B\u8D1F\u6570 = \u5728\u624B\u6307\u524D\u65B9\uFF09\u3002`;
    this.root.append(
      el2(
        "div",
        { class: "sub", "data-testid": "mjweb-place-note" },
        `${custom ? "\u81EA\u5B9A\u4E49\u653E\u7F6E\uFF08\u5728\u300C" + KIND_LABELS[this.state.kind] + "\u300D\u4E0A\u4FEE\u6539\uFF09\u3002" : ""}${behind}UR \u5E95\u5EA7\u5750\u6807\u7CFB\uFF08DH frame 0\uFF09\u5728\u8BAD\u7EC3\u4E16\u754C\u7CFB\u91CC\u7684\u4F4D\u7F6E\uFF08m\uFF09\u548C\u7ED5 z \u7684 yaw\uFF1B\u5B89\u88C5 yaw = \u624B\u7ED5\u6CD5\u5170\u8F74\u7684\u89D2\u5EA6\uFF08\u87BA\u5B54\u5206\u5E03\uFF1A0 / 90 / 180 / 270\xB0\uFF09\u3002\u6BCF\u7269\u4F53\u6700\u4F18 = D4 \u6309\u5168\u90E8 reference \u9009\u51FA\u7684\u9ED8\u8BA4\uFF08\u6BCF\u4E2A\u7269\u4F53\u4E00\u7EC4\uFF09\uFF1B\u6BCF\u7269\u4F53\xB7\u81C2\u5728\u624B\u540E = \u540C\u6837\u7684\u6807\u51C6\uFF0C\u5E95\u5EA7\u9650\u5236\u5728\u624B\u6307\u7684\u53CD\u65B9\u5411\uFF08\u6CBF\u624B\u7684\u5E73\u5747\u6307\u5411\u79BB\u6BCF\u4E2A\u624B\u8155\u4F4D\u7F6E \u2265 0.3 m\uFF09\uFF1B\u5171\u7528 = 8 \u4E2A\u7269\u4F53\u5171\u7528\u7684\u4E00\u7EC4\u3002`
      )
    );
  }
  readPlacement() {
    const v = (k) => Number(this.inputs[k].value);
    return {
      x: v("bx"),
      y: v("by"),
      z: v("bz"),
      yaw_deg: v("byaw"),
      mount_yaw_deg: Number(this.inputs.myaw.value),
      floor_z: this.state.placement?.floor_z ?? null
    };
  }
  /** Redraw the live block (at most ~15 Hz while running). */
  update(ep, force = false) {
    if (!ep || this.state.base !== "arm") return;
    const now = performance.now();
    if (!force && now - this.lastPaint < 66) return;
    this.lastPaint = now;
    const env = ep.armEnv;
    const phase = env.phase;
    const idx = phase === "rest" ? 0 : phase === "policy" ? 2 : 1;
    this.phaseEls.forEach((e, i) => e.classList.toggle("on", i === idx));
    const plan = env.approachPlan;
    this.live.phase.textContent = !ep.policy ? `\u6CA1\u6709\u7B56\u7565\uFF1A\u673A\u68B0\u81C2\u505C\u5728\u8D77\u59CB\u6784\u578B\uFF08\u8D77\u59CB\u7684\u624B\u8155\u4F4D\u59FF\uFF0C\u79BB\u5B83 ${env.log.settleMm.toFixed(3)} mm / ${env.log.settleDeg.toFixed(3)}\xB0\uFF09\uFF0C\u9636\u6BB5\u4E0D\u8FD0\u884C\u3002` : ep.inPhases ? phase === "rest" ? "\u9759\u6B62\u4F4D\uFF1A\u673A\u68B0\u81C2\u5728 ManiSkill \u7684 rest \u5173\u952E\u5E27\uFF0C\u624B\u6307\u5F20\u5F00\uFF1B\u7269\u4F53\u8FD8\u4E0D\u5728\u573A\uFF08\u53EF\u9009\u663E\u793A\u6B8B\u5F71\uFF09\u3002\u6309\u64AD\u653E\u5F00\u59CB\u3002" : plan && plan.t2 <= 0 ? `\u79FB\u5230\u8D77\u59CB\u72B6\u6001\uFF1A${env.phaseTime.toFixed(2)} / ${env.phaseDuration.toFixed(2)} s\uFF08\u4E00\u6BB5\u6700\u5C0F\u6025\u52A8\u5EA6\u76F4\u63A5\u5230\u8D77\u59CB\u6784\u578B\uFF0C\u624B\u6307\u540C\u65F6\u5408\u5230\u8D77\u59CB\u5173\u8282\u89D2\uFF1B\u7269\u4F53\u4E0D\u5728\u573A\uFF0C\u673A\u68B0\u81C2\u505C\u7A33\u540E\u624D\u51FA\u73B0\u5728\u8D77\u59CB\u72B6\u6001\uFF09` : `\u79FB\u5230\u8D77\u59CB\u72B6\u6001\uFF1A${env.phaseTime.toFixed(2)} / ${env.phaseDuration.toFixed(2)} s\uFF08\u76F4\u63A5\u8DEF\u5F84\u4F1A\u78B0\u5230\u673A\u68B0\u81C2\u81EA\u8EAB / \u624B / \u53F0\u67F1\uFF0C\u6240\u4EE5\u8D70\u4E24\u6BB5\uFF1A\u5148\u5230\u8D77\u59CB\u72B6\u6001\u540E\u4E0A\u65B9\u7684\u9884\u5907\u4F4D\u7F6E\uFF0C\u518D\u8FDB\u6765\uFF1B\u7136\u540E\u624B\u6307\u5408\u5230\u8D77\u59CB\u5173\u8282\u89D2\uFF09${plan ? `\uFF0C\u7B2C\u4E00\u6BB5 ${plan.t1.toFixed(2)} s` : ""}` : `\u7B56\u7565\u63A7\u5236\uFF1Aepisode \u7B2C ${ep.samples} / ${ep.total} \u6B65\uFF1B\u8D77\u59CB\u65F6\u673A\u68B0\u81C2\u79BB\u8D77\u59CB\u624B\u8155\u4F4D\u59FF ${env.log.settleMm.toFixed(3)} mm / ${env.log.settleDeg.toFixed(3)}\xB0`;
    const log = env.log;
    this.live.ik.textContent = `IK \u6B8B\u5DEE\uFF08\u547D\u4EE4\u6784\u578B\u5BF9 IK \u7684\u8F93\u5165\uFF09\uFF1A${log.ikMm.toFixed(3)} mm / ${log.ikDeg.toFixed(3)}\xB0\uFF0C\u672C episode \u6700\u5927 ${log.ikMaxMm.toFixed(2)} mm`;
    this.live.track.textContent = `\u5B9E\u9645\u624B\u8155\u5BF9\u5B66\u751F\u7684\u624B\u8155\u6307\u4EE4\uFF1A${log.trackMm.toFixed(2)} mm / ${log.trackDeg.toFixed(2)}\xB0\uFF08\u5BFC\u7EB3\u4E0B\u5305\u542B\u8BAD\u7EC3\u91CC\u672C\u6765\u5C31\u6709\u7684\u6EDE\u540E\u548C\u4E0B\u5782\uFF09`;
    if (env.mode === "admittance") {
      const w = env.contactWrench;
      const f = Math.hypot(w[0], w[1], w[2]);
      const t = Math.hypot(w[3], w[4], w[5]);
      this.live.wrench.textContent = `\u5B9E\u6D4B\u63A5\u89E6\u529B\uFF08\u624B\u4E0A\uFF0C\u5BFC\u7EB3\u7684\u8F93\u5165\uFF09\uFF1A|F| ${f.toFixed(2)} N\uFF0C|\u03C4| ${t.toFixed(3)} N\xB7m\uFF1B\u5B9E\u9645\u624B\u8155\u5BF9\u865A\u62DF\u624B\u8155\uFF1A${log.vdevMm.toFixed(2)} mm / ${log.vdevDeg.toFixed(2)}\xB0\uFF08\u672C episode \u6700\u5927 ${log.vdevMaxMm.toFixed(1)} mm / ${log.vdevMaxDeg.toFixed(1)}\xB0\uFF09`;
    } else this.live.wrench.textContent = "";
    this.live.contacts.textContent = `\u673A\u68B0\u81C2\u2013\u7269\u4F53\u63A5\u89E6\uFF1A${log.armObjectSubsteps} \u4E2A\u5B50\u6B65${log.goalClamped ? `\uFF1BIK \u7684\u8F93\u5165\u88AB\u5B89\u5168\u8303\u56F4\uFF08\u79BB\u80A9 \u2264 1.6 m\uFF09\u622A\u65AD ${log.goalClamped} \u6B21` : ""}`;
    const stop = env.arm.meta.speed_stop ?? SPEED_STOP_DEFAULT;
    this.live.speed.textContent = `IK \u5173\u8282\u901F\u5EA6\u6700\u5927 ${(100 * log.speedFracMax).toFixed(0)} % \u7684 UR \u4E0A\u9650\uFF1B\u901F\u5EA6\u4E0A\u9650\u8D77\u4F5C\u7528\u7684 IK \u6B65 ${log.velBoundSteps} / ${log.ikSteps}\uFF0C\u52A0\u901F\u5EA6\u4E0A\u9650\u8D77\u4F5C\u7528\u7684 ${log.accelBoundSteps}${log.accelRelaxed ? `\uFF08\u8BA9\u4F4D\u4E8E\u4F4D\u7F6E / \u901F\u5EA6\u9650 ${log.accelRelaxed} \u6B21\uFF09` : ""}\uFF1B\u5B9E\u6D4B\u5173\u8282\u901F\u5EA6\u6700\u5927 ${(100 * log.qdRatioMax).toFixed(0)} %\uFF0C\u8D85\u8FC7\u4E0A\u9650\u7684\u5B50\u6B65 ${log.qdOverSubsteps}\uFF08\u4FDD\u62A4\u6027\u505C\u6B62\uFF1A\u8FDE\u7EED ${stop.substeps} \u4E2A\u5B50\u6B65\u8D85\u8FC7 ${(100 * stop.ratio).toFixed(0)} %\uFF09`;
    const lim = env.arm.meta.garm6 ?? { rho_max: 25, sigma_min: 0.03 };
    const over = log.rho > lim.rho_max || log.sigma < lim.sigma_min;
    this.live.garm6.textContent = `${over ? "\u26A0 " : ""}\u5173\u8282\u653E\u5927 \u03C1 ${Number.isFinite(log.rho) ? log.rho.toFixed(1) : "\u2014"} rad/m\uFF08\u672C episode \u6700\u5927 ${log.rhoMax.toFixed(1)}\uFF0C\u9608\u503C ${lim.rho_max}\uFF09\uFF1B\u03C3_min(J_W) ${Number.isFinite(log.sigma) ? log.sigma.toFixed(3) : "\u2014"}\uFF08\u6700\u5C0F ${Number.isFinite(log.sigmaMinMin) ? log.sigmaMinMin.toFixed(3) : "\u2014"}\uFF0C\u9608\u503C ${lim.sigma_min}\uFF09\uFF1B\u8D85\u8FC7\u9608\u503C\u7684 IK \u6B65 ${log.garm6Flagged}\uFF0CIK \u5206\u652F\u53D8\u5316 ${log.branchFlips}`;
    this.live.garm6.className = over || log.garm6Flagged > 0 || log.branchFlips > 0 ? "bad" : "";
    const g = log.firstGuard;
    this.live.guard.textContent = g ? `episode \u88AB\u7ED3\u675F\uFF1A${guardText(g.kind, env.arm.meta)}\uFF08\u7B2C ${g.step} \u4E2A\u63A7\u5236\u6B65\uFF09` : "";
    this.live.guard.className = g ? "bad" : "";
    const { q, qd } = env.armJoints();
    const range = env.model.jnt_range;
    env.arm.meta.joints.forEach((_, i) => {
      const j = env.arm.meta.arm_qpos[i];
      const lo = range[2 * j];
      const hi = range[2 * j + 1];
      const frac = (q[i] - lo) / (hi - lo);
      const b = this.bars[i];
      b.pos.style.left = `${Math.max(0, Math.min(1, frac)) * 100 - 1.5}%`;
      b.pos.style.width = "3%";
      b.pos.className = frac < 0.05 || frac > 0.95 ? "bad" : frac < 0.12 || frac > 0.88 ? "warn" : "";
      const s = Math.min(1, Math.abs(qd[i]));
      b.vel.style.left = "0";
      b.vel.style.width = `${s * 100}%`;
      b.vel.className = s > 0.95 ? "bad" : s > 0.8 ? "warn" : "";
      b.val.textContent = `${(q[i] * 180 / Math.PI).toFixed(0)}\xB0 ${(100 * Math.abs(qd[i])).toFixed(0)}%`;
    });
  }
  dispose() {
    this.root.remove();
    this.style.remove();
  }
};
function setPose2(obj, p, q) {
  obj.position.set(p[0], p[2], -p[1]);
  obj.quaternion.set(-q[1], -q[3], q[2], -q[0]);
}
var WristFrames = class {
  root;
  commanded;
  actual;
  materials = [];
  geometries = [];
  constructor(parent) {
    this.root = new THREE2.Group();
    this.root.name = "mjweb wrist frames";
    this.root.userData = { ignoreDragForce: true };
    this.commanded = this.axes(0.07, 35e-4, [16765503, 16765503, 16765503]);
    this.actual = this.axes(0.055, 18e-4, [16733525, 5627255, 5609983]);
    this.root.add(this.commanded, this.actual);
    this.root.visible = false;
    parent.add(this.root);
  }
  /** Three cylinders along the frame's x / y / z (MuJoCo axes, expressed in the group's local frame). */
  axes(len, r, colors) {
    const g = new THREE2.Group();
    g.userData = { ignoreDragForce: true };
    const dirs = [
      new THREE2.Vector3(1, 0, 0),
      new THREE2.Vector3(0, 0, -1),
      new THREE2.Vector3(0, 1, 0)
    ];
    dirs.forEach((d, i) => {
      const geo = new THREE2.CylinderGeometry(r, r, len, 8);
      const mat = new THREE2.MeshBasicMaterial({
        color: colors[i],
        depthTest: false,
        transparent: true,
        opacity: 0.9
      });
      this.geometries.push(geo);
      this.materials.push(mat);
      const m = new THREE2.Mesh(geo, mat);
      m.renderOrder = 20;
      m.quaternion.setFromUnitVectors(new THREE2.Vector3(0, 1, 0), d);
      m.position.copy(d.clone().multiplyScalar(len / 2));
      m.userData = { ignoreDragForce: true };
      g.add(m);
    });
    return g;
  }
  update(cmdPos, cmdQuat, actPos, actQuat) {
    setPose2(this.commanded, cmdPos, cmdQuat);
    setPose2(this.actual, actPos, actQuat);
  }
  setVisible(on) {
    this.root.visible = on;
  }
  get visible() {
    return this.root.visible;
  }
  dispose() {
    this.root.removeFromParent();
    for (const m of this.materials) m.dispose();
    for (const g of this.geometries) g.dispose();
  }
};

// ../../../../../../web/plugins/look.ts
var THREE3 = __toESM(require_plugin_three_shim());
var LOOK = {
  background: [236, 235, 231],
  // viewer.py BG, also the fog
  cell: [216, 214, 208],
  section: [201, 198, 191],
  shadowOpacity: 0.14,
  gridCell: 0.025,
  // m
  gridSection: 0.125,
  // m
  sun: { color: [255, 250, 241], intensity: 2.3 },
  sky: { sky: [255, 252, 246], ground: [196, 190, 178], intensity: 1.35 },
  handBlack: [0.17, 0.17, 0.18],
  handSilver: [0.8, 0.81, 0.83],
  roughness: { black: 0.62, silver: 0.55, other: 0.68 },
  fog: [0.35, 2.2],
  // m past the camera's distance to its target (render_clip.py)
  floorGap: 0.06,
  // m below the lowest point (render_clip.py FLOOR_GAP)
  fadeS: 0.3,
  // s: render_clip.py's FADE = 9 frames at 30 fps
  shadowMap: 2048,
  shadowExtent: 0.6,
  // m: half the sun's shadow box around the view's target
  shadowExtentArm: 1.6,
  // m: the UR10e view, base and pedestal in the box
  parkedZ: -10,
  // m: below this a body is parked (the arm view's phases 0-1 park the object at z = -50)
  insetSize: [320, 240],
  insetLabel: "functional grasp\uFF08\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF09"
};
function lookRequested() {
  if (typeof window === "undefined") return false;
  return new URLSearchParams(window.location.search).get("look") !== "mjswan";
}
function atBuiltIn(camera, target) {
  return camera.position.distanceTo(new THREE3.Vector3(2, 1.7, 1.7)) < 1e-6 && target.distanceTo(new THREE3.Vector3(0, 0.2, 0)) < 1e-6;
}
function runtime3d() {
  const rt = globalThis.__mjswanRuntime;
  return rt?.renderer && rt.camera && rt.controls ? rt : null;
}
var srgb = (c) => new THREE3.Color().setRGB(c[0] / 255, c[1] / 255, c[2] / 255, THREE3.SRGBColorSpace);
var srgb01 = (c) => new THREE3.Color().setRGB(c[0], c[1], c[2], THREE3.SRGBColorSpace);
var raw = (c) => new THREE3.Color(c[0] / 255, c[1] / 255, c[2] / 255);
function setPose3(obj, p, q) {
  obj.position.set(p[0], p[2], -p[1]);
  obj.quaternion.set(-q[1], -q[3], q[2], -q[0]);
}
var GRID_VERTEX = (
  /* glsl */
  `
  #include <common>
  #include <fog_pars_vertex>
  #include <shadowmap_pars_vertex>
  varying vec3 localPosition;
  varying vec4 worldPosition;
  uniform vec3 worldCamProjPosition;
  uniform vec3 worldPlanePosition;
  uniform float fadeDistance;
  uniform bool infiniteGrid;
  uniform bool followCamera;
  void main() {
    localPosition = position.xzy;
    if (infiniteGrid) localPosition *= 1.0 + fadeDistance;
    worldPosition = modelMatrix * vec4(localPosition, 1.0);
    if (followCamera) {
      worldPosition.xyz += (worldCamProjPosition - worldPlanePosition);
      localPosition = (inverse(modelMatrix) * worldPosition).xyz;
    }
    gl_Position = projectionMatrix * viewMatrix * worldPosition;
    vec3 transformedNormal = normalMatrix * vec3(0.0, 0.0, 1.0);
    #include <shadowmap_vertex>
    #ifdef USE_FOG
      vec4 mvPosition = viewMatrix * worldPosition;
      vFogDepth = -mvPosition.z;
    #endif
  }
`
);
var GRID_FRAGMENT = (
  /* glsl */
  `
  #include <common>
  #include <packing>
  #include <fog_pars_fragment>
  #include <bsdfs>
  #include <lights_pars_begin>
  #include <shadowmap_pars_fragment>
  #include <shadowmask_pars_fragment>
  varying vec3 localPosition;
  varying vec4 worldPosition;
  uniform vec3 worldCamProjPosition;
  uniform float cellSize;
  uniform float sectionSize;
  uniform vec3 cellColor;
  uniform vec3 sectionColor;
  uniform float fadeDistance;
  uniform float fadeStrength;
  uniform float fadeFrom;
  uniform float cellThickness;
  uniform float sectionThickness;
  uniform vec3 planeColor;
  uniform float planeOpacity;
  uniform float shadowOpacity;
  float getGrid(float size, float thickness) {
    vec2 r = localPosition.xz / size;
    vec2 grid = abs(fract(r - 0.5) - 0.5) / fwidth(r);
    float line = min(grid.x, grid.y) + 1.0 - thickness;
    return 1.0 - min(line, 1.0);
  }
  void main() {
    float g1 = getGrid(cellSize, cellThickness);
    float g2 = getGrid(sectionSize, sectionThickness);
    vec3 from = worldCamProjPosition * vec3(fadeFrom);
    float dist = distance(from, worldPosition.xyz);
    float d = 1.0 - min(dist / fadeDistance, 1.0);
    float fade = pow(d, fadeStrength);
    vec3 gridColor = mix(cellColor, sectionColor, min(1.0, sectionThickness * g2));
    float gridAlpha = (g1 + g2) * fade;
    gridAlpha = mix(0.75 * gridAlpha, gridAlpha, g2);
    gl_FragColor = vec4(gridColor, 1.0);
    #include <tonemapping_fragment>
    vec3 toneMappedGridColor = gl_FragColor.rgb;
    float shadowMask = getShadowMask();
    float shadowAlpha = !gl_FrontFacing ? (1.0 - shadowMask) * shadowOpacity * fade : 0.0;
    float fadedPlaneOpacity = planeOpacity * fade;
    vec3 premulRgb = planeColor * fadedPlaneOpacity;
    float alpha = fadedPlaneOpacity;
    premulRgb *= (1.0 - shadowAlpha);
    alpha = shadowAlpha + alpha * (1.0 - shadowAlpha);
    premulRgb = toneMappedGridColor * gridAlpha + premulRgb * (1.0 - gridAlpha);
    alpha = gridAlpha + alpha * (1.0 - gridAlpha);
    if (alpha <= 0.0) discard;
    gl_FragColor = vec4(premulRgb / alpha, alpha);
    #include <colorspace_fragment>
    #include <fog_fragment>
  }
`
);
function gridMesh() {
  const material = new THREE3.ShaderMaterial({
    uniforms: THREE3.UniformsUtils.merge([
      THREE3.UniformsLib.lights,
      THREE3.UniformsLib.fog,
      {
        cellSize: { value: LOOK.gridCell },
        sectionSize: { value: LOOK.gridSection },
        fadeDistance: { value: 3 },
        fadeStrength: { value: 1 },
        fadeFrom: { value: 1 },
        cellThickness: { value: 1 },
        sectionThickness: { value: 1 },
        cellColor: { value: srgb(LOOK.cell) },
        sectionColor: { value: srgb(LOOK.section) },
        infiniteGrid: { value: true },
        followCamera: { value: true },
        worldCamProjPosition: { value: new THREE3.Vector3() },
        worldPlanePosition: { value: new THREE3.Vector3() },
        planeColor: { value: new THREE3.Color(1, 1, 1) },
        planeOpacity: { value: 0 },
        shadowOpacity: { value: LOOK.shadowOpacity }
      }
    ]),
    vertexShader: GRID_VERTEX,
    fragmentShader: GRID_FRAGMENT,
    transparent: true,
    lights: true,
    fog: true,
    side: THREE3.DoubleSide
  });
  const mesh = new THREE3.Mesh(new THREE3.PlaneGeometry(6, 6), material);
  mesh.name = "mjweb look floor";
  mesh.frustumCulled = false;
  mesh.receiveShadow = true;
  mesh.userData = { ignoreDragForce: true, mjwebLook: true };
  return mesh;
}
function updateGrid(grid, camera, far) {
  const u = grid.material.uniforms;
  grid.updateMatrixWorld();
  const plane = new THREE3.Plane().setFromNormalAndCoplanarPoint(new THREE3.Vector3(0, 1, 0), new THREE3.Vector3());
  plane.applyMatrix4(grid.matrixWorld);
  plane.projectPoint(camera.getWorldPosition(new THREE3.Vector3()), u.worldCamProjPosition.value);
  u.worldPlanePosition.value.set(0, 0, 0).applyMatrix4(grid.matrixWorld);
  u.fadeDistance.value = far;
}
function setFloorStrength(grid, f) {
  const bg = LOOK.background;
  const mix = (c) => [0, 1, 2].map((i) => Math.round(bg[i] + (c[i] - bg[i]) * f));
  const u = grid.material.uniforms;
  u.cellColor.value.copy(srgb(mix(LOOK.cell)));
  u.sectionColor.value.copy(srgb(mix(LOOK.section)));
  u.shadowOpacity.value = LOOK.shadowOpacity * f;
  grid.visible = f > 0;
}
function sunDirection(camera, target) {
  const fwd = target.clone().sub(camera.getWorldPosition(new THREE3.Vector3())).normalize();
  const up = new THREE3.Vector3(0, 1, 0);
  const fh = new THREE3.Vector3(fwd.x, 0, fwd.z).normalize();
  const right = fwd.clone().cross(up).normalize();
  return new THREE3.Vector3(0, -1, 0).addScaledVector(fh, 0.85).addScaledVector(right, 0.3).normalize();
}
function sunLight(dir, target, e = LOOK.shadowExtent) {
  const sun = new THREE3.DirectionalLight(srgb(LOOK.sun.color), LOOK.sun.intensity);
  sun.name = "mjweb look sun";
  sun.castShadow = true;
  sun.shadow.mapSize.set(LOOK.shadowMap, LOOK.shadowMap);
  Object.assign(sun.shadow.camera, { left: -e, right: e, top: e, bottom: -e, near: 0.05, far: 2 + 2 * e });
  sun.shadow.camera.updateProjectionMatrix();
  sun.shadow.bias = -2e-4;
  sun.shadow.normalBias = 15e-4;
  sun.position.copy(target).addScaledVector(dir, -2);
  sun.target.position.copy(target);
  return sun;
}
function fitView(pts, dir, fovy, aspect, fill = 0.84) {
  const box = new THREE3.Box3().setFromPoints(pts);
  const center = box.getCenter(new THREE3.Vector3());
  const fwd = dir.clone().normalize();
  const right = fwd.clone().cross(new THREE3.Vector3(0, 1, 0)).normalize();
  const up = right.clone().cross(fwd).normalize();
  const ty = Math.tan(THREE3.MathUtils.degToRad(fovy) / 2) * fill;
  const tx = ty * aspect;
  const rel = pts.map((p) => p.clone().sub(center));
  const fits = (d) => rel.every((q) => {
    const z = q.dot(fwd) + d;
    return z > 1e-3 && Math.abs(q.dot(right)) <= tx * z && Math.abs(q.dot(up)) <= ty * z;
  });
  let lo = 0.01;
  let hi = 5;
  for (let k = 0; k < 40; k++) {
    const mid = (lo + hi) / 2;
    if (fits(mid)) hi = mid;
    else lo = mid;
  }
  center.addScaledVector(up, 0.06 * hi * ty);
  return { center, dist: hi };
}
function skyLight() {
  const sky = new THREE3.HemisphereLight(srgb(LOOK.sky.sky), srgb(LOOK.sky.ground), LOOK.sky.intensity);
  sky.name = "mjweb look sky";
  sky.position.set(0, 1, 0);
  return sky;
}
var ViserLook = class {
  mujoco;
  model;
  data;
  scene;
  parent;
  bodies;
  ep;
  fk;
  drawn;
  saved = [];
  lights = [];
  hidden = [];
  grid;
  sun;
  /** The default view's direction (camera -> target), taken at the first frame (the viewer config lands after us). */
  viewDir = new THREE3.Vector3(0, -1, 0);
  aimed = false;
  framesUnaimed = 0;
  cameraState0;
  /** The UR10e view: the hand on the arm (no free wrist), the floor a real plane under the pedestal. */
  arm;
  floorGeom;
  floorMeshes = [];
  /** The arm view: nothing of the arm below the plane is drawn (the pedestal runs on below it; the opaque plane of
   *  mjswan's look hid that part, the grid would not). */
  clip = new THREE3.Plane(new THREE3.Vector3(0, 1, 0), 0);
  /** The arm view's reference rows: the hand carried rigidly from its FK wrist to the reference wrist. */
  handMove = null;
  onAim;
  previous;
  style;
  floorZ = 0;
  // MuJoCo z of the floor
  strength = 1;
  lastZ = Number.NaN;
  lastVz = 0;
  steps = 0;
  /** consecutive steps whose prediction says the floor would be crossed within FADE_S */
  closing = 0;
  insetRenderer = null;
  insets = /* @__PURE__ */ new Map();
  constructor(mujoco, model, data, scene, parent, bodies, ep, onAim = null) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    this.scene = scene;
    this.parent = parent;
    this.bodies = bodies;
    this.ep = ep;
    this.onAim = onAim;
    this.fk = new mujoco.MjData(model);
    this.drawn = this.collect(ep.L);
    this.arm = ep.L.wristQpos < 0;
    this.floorGeom = this.arm ? mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, "floor") : -1;
    const rt = runtime3d();
    this.cameraState0 = rt?.cameraState;
    this.previous = {
      background: scene.background,
      fog: scene.fog,
      onBeforeRender: scene.onBeforeRender,
      toneMapping: rt?.renderer.toneMapping,
      exposure: rt?.renderer.toneMappingExposure,
      colorSpace: rt?.renderer.outputColorSpace,
      clipping: rt?.renderer.localClippingEnabled
    };
    if (rt) {
      rt.renderer.toneMapping = THREE3.ACESFilmicToneMapping;
      rt.renderer.toneMappingExposure = 1;
      rt.renderer.outputColorSpace = THREE3.SRGBColorSpace;
      if (this.arm) rt.renderer.localClippingEnabled = true;
    }
    scene.background = srgb(LOOK.background);
    scene.fog = new THREE3.Fog(raw(LOOK.background), 1, 3);
    this.restyle();
    scene.traverse((o) => {
      const light = o;
      if (light.isLight && light.visible) {
        light.visible = false;
        this.hidden.push(light);
      }
    });
    if (this.floorGeom >= 0) {
      const fb = model.geom_bodyid[this.floorGeom];
      this.bodies[fb]?.traverse((n) => {
        const mesh = n;
        if (mesh.isMesh && mesh.visible) {
          mesh.visible = false;
          this.floorMeshes.push(mesh);
        }
      });
    }
    this.sun = sunLight(
      new THREE3.Vector3(0, -1, 0),
      new THREE3.Vector3(),
      this.arm ? LOOK.shadowExtentArm : LOOK.shadowExtent
    );
    const sky = skyLight();
    this.grid = gridMesh();
    this.lights.push(this.sun, sky);
    scene.add(this.sun, this.sun.target, sky);
    parent.add(this.grid);
    const look = this;
    const prev = this.previous.onBeforeRender;
    scene.onBeforeRender = function(...args) {
      look.beforeRender(args[2]);
      prev.apply(this, args);
    };
    this.style = injectCss();
    this.reset();
  }
  /** The hand and object bodies, their drawn mesh geoms and the geoms' thinned vertices. */
  collect(L) {
    const m = this.model;
    const parentId = m.body_parentid;
    const inHand = (b) => {
      for (let p = b; p > 0; p = parentId[p]) if (p === L.wristBody) return true;
      return false;
    };
    const bodies = [];
    const hand = /* @__PURE__ */ new Set();
    for (let b = 1; b < m.nbody; b++)
      if ((inHand(b) || b === L.objectBody) && this.bodies[b]) {
        bodies.push(b);
        if (b !== L.objectBody) hand.add(b);
      }
    const geoms = [];
    const meshType = this.mujoco.mjtGeom.mjGEOM_MESH.value;
    for (const b of bodies)
      this.bodies[b].traverse((n) => {
        const mesh = n;
        const g = mesh.userData?.geomId;
        if (mesh.isMesh && g !== void 0 && mesh.visible && m.geom_type[g] === meshType)
          geoms.push(g);
      });
    const vert = m.mesh_vert;
    const adr = m.mesh_vertadr;
    const num = m.mesh_vertnum;
    const dataid = m.geom_dataid;
    const verts = geoms.map((g) => {
      const k = dataid[g];
      const out = [];
      for (let v = 0; v < num[k]; v += 7) {
        const i = 3 * (adr[k] + v);
        out.push(vert[i], vert[i + 1], vert[i + 2]);
      }
      return new Float32Array(out);
    });
    return { bodies, hand, geoms, verts };
  }
  /** Matte standard materials in the MJCF colours (viewer.py add_meshes), every live mesh casting and taking shadows. */
  restyle() {
    const m = this.model;
    const matid = m.geom_matid;
    const matRgba = m.mat_rgba;
    const geomRgba = m.geom_rgba;
    const done = /* @__PURE__ */ new Map();
    for (const [id2, body] of Object.entries(this.bodies)) {
      const clipped = this.arm && !this.drawn.bodies.includes(Number(id2));
      body.traverse((n) => {
        const mesh = n;
        const g = mesh.userData?.geomId;
        if (!mesh.isMesh || g === void 0 || mesh.userData?.mjwebGhost) return;
        const mid = matid[g];
        const name = mid >= 0 ? this.mujoco.mj_id2name(m, this.mujoco.mjtObj.mjOBJ_MATERIAL.value, mid) : "";
        const rgba = mid >= 0 ? matRgba.subarray(4 * mid, 4 * mid + 4) : geomRgba.subarray(4 * g, 4 * g + 4);
        const old = mesh.material;
        const key = `${clipped}|${name}|${Array.from(rgba).join(",")}|${old.map?.uuid ?? ""}`;
        let mat = done.get(key);
        if (!mat) {
          const [col, rough] = name === "matte_black" ? [LOOK.handBlack, LOOK.roughness.black] : name === "metal_silver" ? [LOOK.handSilver, LOOK.roughness.silver] : [[rgba[0], rgba[1], rgba[2]], LOOK.roughness.other];
          mat = new THREE3.MeshStandardMaterial({
            color: srgb01(col),
            roughness: rough,
            metalness: 0,
            transparent: rgba[3] < 1,
            opacity: rgba[3],
            map: old.map ?? null
          });
          if (clipped) {
            mat.clippingPlanes = [this.clip];
            mat.clipShadows = true;
          }
          done.set(key, mat);
        }
        this.saved.push({ mesh, material: mesh.material, cast: mesh.castShadow, receive: mesh.receiveShadow });
        mesh.material = mat;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
      });
    }
  }
  /**
   * The drawn hand and object vertices (the thinned set, every ``every``-th) at the FK state ``this.fk`` holds, MuJoCo
   * world coordinates; the hand carried by ``handMove`` when it is set; a parked body (below ``parkedZ``) left out.
   */
  points(every = 1) {
    const xpos = this.fk.geom_xpos;
    const xmat = this.fk.geom_xmat;
    const gbody = this.model.geom_bodyid;
    const mv2 = this.handMove;
    const rot = mv2 ? quatMul(mv2.rq, quatInv(mv2.fq)) : null;
    const out = [];
    this.drawn.geoms.forEach((g, i) => {
      if (xpos[3 * g + 2] < LOOK.parkedZ) return;
      const v = this.drawn.verts[i];
      const R = xmat.subarray(9 * g, 9 * g + 9);
      const p = xpos.subarray(3 * g, 3 * g + 3);
      const moved = rot !== null && this.drawn.hand.has(gbody[g]);
      for (let j = 0; j < v.length; j += 3 * every) {
        const [a, b, c] = [v[j], v[j + 1], v[j + 2]];
        let w = [0, 1, 2].map((k) => R[3 * k] * a + R[3 * k + 1] * b + R[3 * k + 2] * c + p[k]);
        if (moved && mv2 && rot) {
          const d = quatApply(rot, [w[0] - mv2.fp[0], w[1] - mv2.fp[1], w[2] - mv2.fp[2]]);
          w = [d[0] + mv2.rp[0], d[1] + mv2.rp[1], d[2] + mv2.rp[2]];
        }
        out.push(w);
      }
    });
    return out;
  }
  /** The lowest MuJoCo z of the drawn hand and object (``points``). */
  lowest() {
    let low = Infinity;
    for (const w of this.points()) low = Math.min(low, w[2]);
    return low;
  }
  drawnPoints() {
    return this.points(3);
  }
  lowestLive() {
    this.fk.qpos.set(this.data.qpos);
    this.mujoco.mj_kinematics(this.model, this.fk);
    this.handMove = null;
    return this.lowest();
  }
  /**
   * FK of a reference row on ``this.fk``: the floating hand at the reference's wrist (ghosts.ts kinematics); on the arm,
   * the joints and the object from the reference and the hand carried rigidly to the reference's wrist (``handMove``,
   * ghosts.ts moveHand).
   */
  referenceRow(ref, row) {
    const L = this.ep.L;
    const r = Math.max(0, Math.min(row, ref.rows - 1));
    const q = this.fk.qpos;
    q.set(this.data.qpos);
    if (!this.arm) {
      q.set(ref.wrist_pos[r], L.wristQpos);
      q.set(ref.wrist_quat[r], L.wristQpos + 3);
    }
    for (let i = 0; i < 20; i++) q[L.fingerQpos[i]] = ref.joints[r][i];
    q.set(ref.object_pos[r], L.objectQpos);
    q.set(ref.object_quat[r], L.objectQpos + 3);
    this.mujoco.mj_kinematics(this.model, this.fk);
    if (this.arm) {
      const xpos = this.fk.xpos;
      const xquat = this.fk.xquat;
      const w = L.wristBody;
      this.handMove = {
        fp: Array.from(xpos.subarray(3 * w, 3 * w + 3)),
        fq: Array.from(xquat.subarray(4 * w, 4 * w + 4)),
        rp: ref.wrist_pos[r],
        rq: ref.wrist_quat[r]
      };
    } else this.handMove = null;
  }
  /** A body's world pose at ``this.fk``, carried by ``handMove`` when it is one of the hand's. */
  bodyPose(b) {
    const xpos = this.fk.xpos;
    const xquat = this.fk.xquat;
    const pos = xpos.subarray(3 * b, 3 * b + 3);
    const quat = xquat.subarray(4 * b, 4 * b + 4);
    const mv2 = this.handMove;
    if (!mv2 || !this.drawn.hand.has(b)) return { pos, quat };
    const rel = poseInFrame(mv2.fp, mv2.fq, pos, quat);
    return poseFromFrame(mv2.rp, mv2.rq, rel.pos, rel.quat);
  }
  /**
   * A new episode (start, reference, reset), the floor at full strength: on the floating view FLOOR_GAP under the hand
   * and object over the start and the reference; on the arm view at the plane under the pedestal, so it never cuts
   * through the base.
   */
  reset() {
    let low = this.lowestLive();
    if (this.floorGeom >= 0) {
      this.floorZ = this.fk.geom_xpos[3 * this.floorGeom + 2];
    } else {
      const ref = this.ep.ref;
      const t0 = this.ep.start.start_t;
      for (let r = t0; r < ref.rows; r += 6) {
        this.referenceRow(ref, r);
        low = Math.min(low, this.lowest());
      }
      this.handMove = null;
      this.floorZ = low - LOOK.floorGap;
    }
    this.grid.position.set(0, this.floorZ, 0);
    this.grid.updateMatrixWorld();
    this.clip.constant = -this.grid.getWorldPosition(new THREE3.Vector3()).y;
    this.strength = 1;
    this.lastZ = Number.NaN;
    this.lastVz = 0;
    this.steps = 0;
    this.closing = 0;
    setFloorStrength(this.grid, 1);
  }
  /**
   * After each control step: fade the floor out before anything comes down through it, then hide it. Unlike the clips,
   * the page cannot look ahead: the time left before the lowest point would cross the floor is predicted from its
   * downward speed, and from free fall while it moves down and accelerates downwards at more than half of g (a dropped
   * object); the floor's strength follows that time over FADE_S once two steps in a row predict it (not in the first
   * steps, where the grasp settles), back up slowly when it passes (a push), and stays 0 from the step anything is below
   * it.
   */
  step(dt = 1 / 30) {
    if (this.strength <= 0) return;
    const z = this.lowestLive();
    const vz = Number.isNaN(this.lastZ) ? 0 : (z - this.lastZ) / dt;
    const settled = this.steps > 5;
    const falling = vz < 0 && (vz - this.lastVz) / dt < -4.9;
    this.lastZ = z;
    this.lastVz = vz;
    this.steps += 1;
    const gap = z - this.floorZ;
    let f;
    if (gap <= 0) f = 0;
    else {
      const s = Math.max(-vz, 0);
      const t = falling ? (-s + Math.sqrt(s * s + 2 * 9.81 * gap)) / 9.81 : s > 0 ? gap / s : Infinity;
      const want = Math.min(1, t / LOOK.fadeS);
      this.closing = settled && want < 1 ? this.closing + 1 : 0;
      f = this.closing >= 2 && want < this.strength ? Math.max(want, 0.02) : Math.min(1, this.strength + 0.1);
    }
    if (f !== this.strength) {
      this.strength = f;
      setFloorStrength(this.grid, f);
      if (f <= 0) this.clip.constant = 1e9;
    }
  }
  /**
   * The sun from above and behind the default camera, and the view the functional-grasp card is drawn from: at the first
   * frame, once mjswan has applied the scene's viewer config (the camera then follows the wrist without turning).
   */
  aim(camera, target) {
    this.viewDir = target.clone().sub(camera.getWorldPosition(new THREE3.Vector3())).normalize();
    const dir = sunDirection(camera, target);
    this.sun.position.copy(target).addScaledVector(dir, -2);
    this.sun.target.position.copy(target);
    this.sun.target.updateMatrixWorld();
    this.aimed = true;
    if (this.onAim) setTimeout(this.onAim, 0);
  }
  /** The fog from the camera's distance to its target (render_clip.py: near + 0.35 to near + 2.2 m). */
  beforeRender(camera) {
    const rt = runtime3d();
    const target = rt?.controls.target ?? new THREE3.Vector3();
    if (!this.aimed && rt && (rt.cameraState !== this.cameraState0 || (rt.cameraState?.trackBodyId ?? null) !== null || !atBuiltIn(camera, target) || ++this.framesUnaimed > 120))
      this.aim(camera, target);
    const near = camera.getWorldPosition(new THREE3.Vector3()).distanceTo(target);
    const fog = this.scene.fog;
    fog.near = near + LOOK.fog[0];
    fog.far = near + LOOK.fog[1];
    updateGrid(this.grid, camera, fog.far);
  }
  // ------------------------------------------------------------------------------------------ the functional grasp
  /**
   * The functional-grasp card's image of ``ref`` in this look: the hand and object at the demo's last motion row, from
   * the default view's direction, fitted to the frame, on its own floor; replaces ``ref.fg_inset`` (the old renderer's
   * image) for this page, once per reference.
   */
  ensureInset(ref) {
    if (!this.aimed) return;
    const done = this.insets.get(ref.case);
    if (done) {
      ref.fg_inset = done;
      return;
    }
    const url = this.renderInset(ref);
    if (url) {
      this.insets.set(ref.case, url);
      ref.fg_inset = url;
    }
  }
  renderInset(ref) {
    if (typeof document === "undefined") return null;
    const [w, h] = LOOK.insetSize;
    if (!this.insetRenderer) {
      const r = new THREE3.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
      r.setPixelRatio(2);
      r.setSize(w, h, false);
      r.toneMapping = THREE3.ACESFilmicToneMapping;
      r.outputColorSpace = THREE3.SRGBColorSpace;
      r.shadowMap.enabled = true;
      r.shadowMap.type = THREE3.PCFSoftShadowMap;
      this.insetRenderer = r;
    }
    this.referenceRow(ref, ref.motion_steps - 1);
    const scene = new THREE3.Scene();
    scene.background = srgb(LOOK.background);
    const root = new THREE3.Group();
    root.matrixAutoUpdate = false;
    this.parent.updateMatrixWorld();
    root.matrix.copy(this.parent.matrixWorld);
    scene.add(root);
    for (const b of this.drawn.bodies) {
      const c = this.bodies[b].clone(true);
      const pose = this.bodyPose(b);
      setPose3(c, pose.pos, pose.quat);
      c.visible = true;
      root.add(c);
    }
    scene.updateMatrixWorld(true);
    const pts = this.drawnPoints().map((p) => root.localToWorld(new THREE3.Vector3(p[0], p[2], -p[1])));
    const { center, dist } = fitView(pts, this.viewDir, 45, w / h);
    const camera = new THREE3.PerspectiveCamera(45, w / h, 0.01, 20);
    camera.position.copy(center).addScaledVector(this.viewDir, -dist);
    camera.lookAt(center);
    camera.updateMatrixWorld();
    const sun = sunLight(sunDirection(camera, center), center);
    scene.add(sun, sun.target, skyLight());
    const grid = gridMesh();
    root.add(grid);
    grid.position.set(0, this.lowest() - LOOK.floorGap, 0);
    this.handMove = null;
    scene.fog = new THREE3.Fog(raw(LOOK.background), dist + LOOK.fog[0], dist + LOOK.fog[1]);
    root.updateMatrixWorld(true);
    updateGrid(grid, camera, dist + LOOK.fog[1]);
    this.insetRenderer.render(scene, camera);
    const url = this.insetRenderer.domElement.toDataURL("image/jpeg", 0.92);
    grid.geometry.dispose();
    grid.material.dispose();
    sun.dispose();
    return url;
  }
  /** Test hook: the floor (three.js / MuJoCo height), its strength, the live hand + object's lowest point, the sun. */
  status() {
    const rt = runtime3d();
    return {
      floorZ: this.floorZ,
      strength: this.strength,
      gridVisible: this.grid.visible,
      aimed: this.aimed,
      viewDir: this.viewDir.toArray(),
      lowestLive: this.lowestLive(),
      sun: this.sun.position.toArray(),
      sunTarget: this.sun.target.position.toArray(),
      camera: rt?.camera.position.toArray() ?? null,
      target: rt?.controls.target.toArray() ?? null,
      fog: [this.scene.fog.near, this.scene.fog.far],
      toneMapping: rt?.renderer.toneMapping ?? null,
      insets: [...this.insets.keys()]
    };
  }
  /** The card around the image: site-2's .inset (rounded, warm, shadowed) with its label chip. */
  decorateInset(box) {
    box.classList.add("mjweb-look-inset");
    if (!box.querySelector("i.mjweb-look-label")) {
      const label = document.createElement("i");
      label.className = "mjweb-look-label";
      label.textContent = LOOK.insetLabel;
      box.append(label);
    }
  }
  dispose() {
    const rt = runtime3d();
    if (rt) {
      if (this.previous.toneMapping !== void 0) rt.renderer.toneMapping = this.previous.toneMapping;
      if (this.previous.exposure !== void 0) rt.renderer.toneMappingExposure = this.previous.exposure;
      if (this.previous.colorSpace !== void 0) rt.renderer.outputColorSpace = this.previous.colorSpace;
      if (this.previous.clipping !== void 0) rt.renderer.localClippingEnabled = this.previous.clipping;
    }
    this.scene.background = this.previous.background;
    this.scene.fog = this.previous.fog;
    this.scene.onBeforeRender = this.previous.onBeforeRender;
    for (const s of this.saved) {
      s.mesh.material = s.material;
      s.mesh.castShadow = s.cast;
      s.mesh.receiveShadow = s.receive;
    }
    for (const l of this.hidden) l.visible = true;
    for (const m of this.floorMeshes) m.visible = true;
    for (const l of this.lights) {
      l.removeFromParent();
      l.target?.removeFromParent();
      l.dispose();
    }
    this.grid.removeFromParent();
    this.grid.geometry.dispose();
    this.grid.material.dispose();
    this.style?.remove();
    this.insetRenderer?.dispose();
    this.fk.delete();
  }
};
var CSS3 = `
#mjweb-panel, #mjweb-arm-panel { background: rgba(255,255,255,.66) !important; color: #1d1d1f !important;
  -webkit-backdrop-filter: blur(18px) saturate(1.4); backdrop-filter: blur(18px) saturate(1.4);
  box-shadow: 0 0 0 1px rgba(0,0,0,.06), 0 8px 24px rgba(60,50,30,.08) !important; border-radius: 14px !important;
  font-family: -apple-system, BlinkMacSystemFont, "SF Pro Text", "PingFang SC", "Helvetica Neue", sans-serif !important;
  -webkit-font-smoothing: antialiased; }
#mjweb-panel h1, #mjweb-arm-panel h1 { color: #1d1d1f; letter-spacing: -.005em; }
#mjweb-panel .note, #mjweb-arm-panel .note { color: #8a5a00 !important; }
#mjweb-panel .lab, #mjweb-panel .sub, #mjweb-panel .dim, #mjweb-arm-panel .lab, #mjweb-arm-panel .sub,
  #mjweb-arm-panel .dim { color: #6e6e73 !important; }
#mjweb-panel .prow .rl { color: #3a3a3c !important; }
#mjweb-panel button, #mjweb-arm-panel button { background: rgba(255,255,255,.72) !important; color: #1d1d1f !important;
  border: 1px solid rgba(0,0,0,.12) !important; }
#mjweb-panel button:hover, #mjweb-arm-panel button:hover { background: #fff !important; }
#mjweb-panel button.on, #mjweb-arm-panel button.on { background: #0071e3 !important; border-color: #0071e3 !important;
  color: #fff !important; }
#mjweb-panel select, #mjweb-panel input[type=number], #mjweb-arm-panel select, #mjweb-arm-panel input[type=number] {
  background: rgba(255,255,255,.8) !important; color: #1d1d1f !important; border: 1px solid rgba(0,0,0,.14) !important; }
#mjweb-panel .live { border-top-color: rgba(0,0,0,.08) !important; }
#mjweb-panel .ok, #mjweb-arm-panel .ok { color: #1a7f37 !important; }
#mjweb-panel .bad, #mjweb-arm-panel .bad { color: #c9372c !important; }
#mjweb-panel .banner { background: #fde8e7 !important; color: #8e1b14 !important; border-color: #f2b8b4 !important; }
#mjweb-panel .nopolicy { background: #e8f0fd !important; color: #0b3d91 !important; border-color: #b8cdf2 !important; }
#mjweb-panel .dry { background: #fdf3dc !important; color: #6b4c00 !important; border-color: #ecd59c !important; }
#mjweb-arm-panel .phase { background: rgba(0,0,0,.06) !important; color: #6e6e73 !important; }
#mjweb-arm-panel .phase.on { background: #0071e3 !important; color: #fff !important; }
#mjweb-arm-panel .bar { background: rgba(0,0,0,.08) !important; }
.mjweb-look-inset { border-radius: 16px !important; overflow: hidden; background: #f6f5f2 !important;
  box-shadow: 0 0 0 1px rgba(0,0,0,.09), 0 12px 32px rgba(60,50,30,.13) !important; }
.mjweb-look-inset img { width: 320px !important; height: 240px !important; }
.mjweb-look-inset i.mjweb-look-label { position: absolute; left: 12px; top: 12px; padding: 5px 9px 6px;
  border-radius: 9px; font: normal 12.5px/1.2 -apple-system, "SF Pro Text", "PingFang SC", sans-serif; color: #3a3a3c;
  background: rgba(255,255,255,.82); box-shadow: 0 0 0 1px rgba(0,0,0,.05); }
`;
function injectCss() {
  if (typeof document === "undefined") return null;
  document.getElementById("mjweb-look-css")?.remove();
  const style = document.createElement("style");
  style.id = "mjweb-look-css";
  style.textContent = CSS3;
  document.head.appendChild(style);
  return style;
}

// ../../../../../../web/plugins/StudentController.ts
function runtime() {
  return globalThis.__mjswanRuntime ?? null;
}
function params() {
  return new URLSearchParams(
    typeof window === "undefined" ? "" : window.location.search
  );
}
function flag(p, key, fallback) {
  const v = p.get(key);
  return v === null ? fallback : v !== "0" && v !== "false";
}
async function fetchJson(url) {
  const res = await fetch(new URL(url, document.baseURI).toString());
  if (!res.ok) throw new Error(`[mjweb] ${url}: HTTP ${res.status}`);
  return await res.json();
}
function toParity(records) {
  return {
    t: records.map((r) => r.t),
    wrist_pos: records.map((r) => r.wristPos),
    wrist_quat: records.map((r) => r.wristQuat),
    joints: records.map((r) => r.joints),
    object_pos: records.map((r) => r.objectPos),
    object_quat: records.map((r) => r.objectQuat),
    target: records.map((r) => r.target),
    obs: records.map((r) => r.obs),
    action: records.map((r) => r.action),
    rms: records.map((r) => r.rms),
    hand_max: records.map((r) => r.handMax),
    passed: records.map((r) => r.passed),
    hand26: records.map((r) => r.hand26),
    fg_near: records.map((r) => r.fgNear),
    fg_rot: records.map((r) => r.fgRot),
    fg_pos: records.map((r) => r.fgPos),
    fg_contact: records.map((r) => r.fgContact),
    fg_passed: records.map((r) => r.fgPassed),
    rel_kp: records.map((r) => r.objectRelKpErr),
    wrist_object_dist: records.map((r) => r.wristObjectDist),
    nonfinite: records.map((r) => r.nonfinite)
  };
}
var StudentController = class {
  context;
  objectId;
  /** The mjswan scene id (the object id on the default page, `<object>_arm` for the arm). */
  sceneId;
  policyId;
  policyKey;
  wristMode;
  hand;
  /** No policy for this scene yet: show the start grasps and the ghosts, never step. */
  viewer;
  cmd = new Float32Array(1);
  zeros = new Float32Array(OBS_DIM);
  ready;
  episode = null;
  panel = null;
  /** The arm bundle: this scene's base and the other base's scene id ("" outside the arm bundle). */
  base;
  pairScene;
  armPanel = null;
  frames = null;
  /** Draw ``frames`` (URL `wframes`, off by default: team-lead 10-20 23:05, the live hand and object only). */
  wristFrames = false;
  ghosts = null;
  /** The viser look of both views (look.ts; URL `look=mjswan` keeps mjswan's; the legacy beta1 page keeps it). */
  look = null;
  previousBackground = void 0;
  /** Restores the arm view's floor material (``applyCheckerFloor``). */
  restoreFloor = null;
  visibility = { ...DEFAULT_VISIBILITY };
  /** The functional-grasp image in the lower-right corner (the demo's last row of the current reference). */
  insetEl = null;
  pauseOnFirst;
  disposed = false;
  loadError = null;
  /** Test hook: blow MuJoCo up (huge force on the object) in the next control step. */
  forceBlowUp = false;
  decimationChecked = false;
  // stats
  windowStart = 0;
  windowSteps = 0;
  controlRate = 0;
  physicsMs = 0;
  inferMs = 0;
  markObs = 0;
  markPhysics = 0;
  totalSteps = 0;
  episodesDone = 0;
  /** Wall clock of the current episode's first and latest step (real-time factor). */
  episodeT0 = 0;
  episodeT1 = 0;
  minWindowRate = Infinity;
  constructor(_termName, config, context) {
    this.context = context;
    this.objectId = String(config.object_id ?? "");
    this.sceneId = String(config.scene_id ?? this.objectId);
    this.policyId = String(config.policy_id ?? "");
    this.policyKey = String(config.policy_key ?? "");
    const mode = String(config.wrist_mode ?? "");
    if (mode !== "implicit" && mode !== "substep" && mode !== "isaac") {
      throw new Error(
        `[mjweb] unknown wrist_mode ${JSON.stringify(mode)} in the scene config (expected the harness's wrist_wrench_rate)`
      );
    }
    this.wristMode = mode;
    const physics = String(config.physics ?? "beta1");
    if (physics !== "hand21" && physics !== "beta1")
      throw new Error(
        `[mjweb] unknown physics ${JSON.stringify(physics)} in the scene config`
      );
    if (physics === "hand21" && mode !== "isaac")
      throw new Error(
        '[mjweb] Hand 2.1 holds the official wrench per 1/120 s physics step (wrist_mode "isaac")'
      );
    this.hand = physics;
    this.base = config.base === "arm" ? "arm" : "float";
    this.pairScene = String(config.pair_scene ?? "");
    this.viewer = config.viewer === true || this.policyId === "";
    const p = params();
    this.pauseOnFirst = this.viewer || !flag(p, "play", true);
    globalThis.__mjswanUnpaced = flag(p, "fast", false);
    globalThis.__mjswanRenderOnChange = p.get("render") !== "all";
    this.visibility = {
      reference: flag(p, "ghost", DEFAULT_VISIBILITY.reference),
      inset: flag(p, "fginset", DEFAULT_VISIBILITY.inset)
    };
    this.wristFrames = flag(p, "wframes", false);
    session.controller = this;
    this.ready = this.load(
      String(config.webdata_url),
      String(config.catalog_url)
    ).catch((error) => {
      this.loadError = String(error);
      console.error("[mjweb] could not start:", error);
    });
    this.exposeHooks();
  }
  // ------------------------------------------------------------------ setup
  async load(webdataUrl, catalogUrl) {
    const [wd, catalog] = await Promise.all([
      fetchJson(webdataUrl),
      fetchJson(catalogUrl)
    ]);
    if (this.disposed) return;
    const { mujoco, mjModel, mjData } = this.context;
    if (!mjModel || !mjData) throw new Error("[mjweb] no model");
    if (wd.object_id !== this.objectId)
      throw new Error(
        `[mjweb] webdata is for ${wd.object_id}, scene is ${this.objectId}`
      );
    const p = params();
    if ((wd.hand ?? "beta1") !== this.hand)
      throw new Error(
        `[mjweb] webdata hand ${wd.hand ?? "beta1"} != scene physics ${this.hand}`
      );
    const settings = {
      sense: flag(p, "sense", true),
      perturb: flag(p, "pert", false),
      seed: Math.trunc(Number(p.get("pseed") ?? 0)) || 0
    };
    const arm = wd.arm;
    if (this.base === "arm" && !arm)
      throw new Error("[mjweb] an arm scene needs the arm web data");
    const ep = this.base === "arm" && arm ? new ArmEpisode(
      mujoco,
      mjModel,
      mjData,
      wd,
      this.viewer ? null : this.policyId,
      settings,
      arm,
      {
        mode: p.get("mode") === "stiff" ? "stiff" : arm.default_mode,
        phases: flag(p, "phases", true),
        kind: this.urlKind(p, arm),
        objectGhost: flag(p, "oghost", false)
      }
    ) : new Episode(
      mujoco,
      mjModel,
      mjData,
      wd,
      this.viewer ? null : this.policyId,
      settings,
      this.wristMode
    );
    if (ep instanceof ArmEpisode) {
      const entry = ep.arm.placements[ep.armSettings.kind];
      const pl = this.urlPlacement(p, entry?.placement ?? ep.arm.placement);
      const bq = entry?.branch_q ?? ep.arm.branch_q;
      if (!ep.armEnv.placementMatches(pl)) {
        ep.armEnv.setPlacement(pl, bq);
        this.syncStaticArmBodies(ep);
      } else ep.armEnv.branchQ = bq.slice();
    }
    ep.recording = flag(p, "record", false);
    const start = Number(p.get("start"));
    if (Number.isInteger(start) && start >= 0 && start < wd.starts.length)
      ep.selectStart(start);
    this.episode = ep;
    this.previousBackground = applyBackdrop(this.context.scene);
    if (ep instanceof ArmEpisode && this.context.bodies && !(this.hand === "hand21" && lookRequested())) {
      const floor = mujoco.mj_name2id(
        mjModel,
        mujoco.mjtObj.mjOBJ_BODY.value,
        "floor"
      );
      if (floor >= 0 && this.context.bodies[floor])
        this.restoreFloor = applyCheckerFloor(this.context.bodies[floor]);
    }
    const parent = this.context.mujocoRoot ?? this.context.scene;
    if (this.context.bodies) {
      this.ghosts = new Ghosts(
        mujoco,
        mjModel,
        this.context.bodies,
        parent,
        ep.L
      );
      this.ghosts.setVisibility(this.visibility);
      if (this.hand === "hand21" && lookRequested())
        this.look = new ViserLook(
          mujoco,
          mjModel,
          mjData,
          this.context.scene,
          parent,
          this.context.bodies,
          ep,
          () => this.updateInset()
        );
    }
    this.updateInset();
    const rt = runtime();
    const modes = rt?.interactionModes() ?? [];
    const mouse = p.get("mouse") ?? "pull";
    const mouseMode = modes.some((m) => m.id === mouse && m.available) ? mouse : "view";
    rt?.setInteractionMode(mouseMode);
    this.panel = new MjwebPanel(
      catalog,
      ep,
      {
        sceneId: this.sceneId,
        policyKey: this.policyKey,
        viewer: this.viewer,
        sense: ep.settings.sense,
        perturb: ep.settings.perturb,
        seed: ep.settings.seed,
        ...this.visibility,
        mouseMode
      },
      {
        navigate: (sceneId, policyKey) => this.navigate(sceneId, policyKey),
        selectStart: (i) => this.selectStart(i),
        setPerturb: (on, seed) => this.setSettings({ perturb: on, seed }),
        setSense: (on) => this.setSettings({ sense: on }),
        setVisibility: (v) => this.setVisibility(v),
        playPause: () => runtime()?.isRunning ? this.pause() : this.play(),
        reset: () => this.requestReset(),
        setMouseMode: (m) => {
          runtime()?.setInteractionMode(m);
          this.writeUrl({ mouse: m });
        }
      },
      modes.map((m) => ({ id: m.id, available: m.available }))
    );
    if (this.pairScene || this.base === "arm") {
      const armEp = ep instanceof ArmEpisode ? ep : null;
      const kind = armEp?.armSettings.kind ?? "best";
      const entry = armEp?.arm.placements[kind];
      this.armPanel = new ArmPanel(
        {
          base: this.base,
          mode: armEp?.armSettings.mode ?? "admittance",
          phases: armEp?.armSettings.phases ?? true,
          objectGhost: armEp?.armSettings.objectGhost ?? false,
          wristFrames: this.wristFrames,
          kind,
          kinds: armEp?.arm.placements ?? {},
          placement: armEp ? this.urlPlacement(p, entry?.placement ?? armEp.arm.placement) : null,
          hasPair: this.pairScene !== ""
        },
        {
          setBase: (b) => this.navigate(this.pairScene, this.policyKey, { base: b }),
          setMode: (m) => this.setArm({ mode: m }),
          setPhases: (on) => this.setArm({ phases: on }),
          setObjectGhost: (on) => this.setArm({ objectGhost: on }),
          setWristFrames: (on) => {
            this.wristFrames = on;
            this.writeUrl({ wframes: on ? "1" : "0" });
            this.syncArm(true);
            runtime()?.requestRender?.();
          },
          setKind: (k) => this.setArmKind(k),
          setPlacement: (pl) => this.setArmPlacement(pl)
        }
      );
      if (armEp) this.frames = new WristFrames(parent);
    }
  }
  /** mjswan draws a body without a joint above it (the arm's base with its pedestal, the floor) where it was at
   *  load; after a placement change their three.js groups move to the new poses (mjswan's own swizzle). */
  syncStaticArmBodies(ep) {
    const { mujoco, mjModel: m, mjData: d, bodies } = this.context;
    if (!bodies || !m || !d) return;
    const floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY.value, "floor");
    const xpos = d.xpos;
    const xquat = d.xquat;
    for (const b of [ep.arm.meta.base_body, floor]) {
      const g = b >= 0 ? bodies[b] : void 0;
      if (!g) continue;
      g.position.set(xpos[3 * b], xpos[3 * b + 2], -xpos[3 * b + 1]);
      g.quaternion.set(
        -xquat[4 * b + 1],
        -xquat[4 * b + 3],
        xquat[4 * b + 2],
        -xquat[4 * b]
      );
    }
    runtime()?.requestRender?.();
  }
  /** The placement kind in the URL (`place`), else the web data's default. */
  urlKind(p, arm) {
    const k = p.get("place");
    return k && arm.placements[k] ? k : arm.default_kind;
  }
  /** The placement in the URL (`bx` `by` `bz` `byaw` `myaw`) on top of the kind's. */
  urlPlacement(p, d) {
    const num = (k, v) => {
      const x = Number(p.get(k));
      return p.has(k) && Number.isFinite(x) ? x : v;
    };
    return {
      x: num("bx", d.x),
      y: num("by", d.y),
      z: num("bz", d.z),
      yaw_deg: num("byaw", d.yaw_deg),
      mount_yaw_deg: num("myaw", d.mount_yaw_deg),
      floor_z: d.floor_z
    };
  }
  setArm(s) {
    const ep = this.episode;
    if (!(ep instanceof ArmEpisode)) return;
    ep.armSettings = { ...ep.armSettings, ...s };
    this.writeUrl({
      mode: ep.armSettings.mode,
      phases: ep.armSettings.phases ? "1" : "0",
      oghost: ep.armSettings.objectGhost ? "1" : "0"
    });
    window.location.search = params().toString();
  }
  /** A placement kind: its own pose (custom edits dropped); the page reloads (everything is in the URL). */
  setArmKind(kind) {
    const p = params();
    p.set("place", kind);
    for (const k of ["bx", "by", "bz", "byaw", "myaw"]) p.delete(k);
    window.location.search = p.toString();
  }
  /** A custom base pose on top of the selected kind (the URL keeps it); the page reloads. */
  setArmPlacement(pl) {
    const p = params();
    for (const [k, v] of [
      ["bx", pl.x],
      ["by", pl.y],
      ["bz", pl.z],
      ["byaw", pl.yaw_deg],
      ["myaw", pl.mount_yaw_deg]
    ])
      p.set(k, String(v));
    window.location.search = p.toString();
  }
  exposeHooks() {
    if (typeof window === "undefined") return;
    globalThis.__mjweb = {
      status: () => this.status(),
      selectStart: (i) => this.selectStart(i),
      setSense: (on) => this.setSettings({ sense: on }),
      setPerturb: (on, seed = 0) => this.setSettings({ perturb: on, seed }),
      setRecording: (on) => {
        if (this.episode) this.episode.recording = on;
      },
      setUnpaced: (on) => {
        globalThis.__mjswanUnpaced = on;
      },
      reset: () => this.requestReset(),
      play: () => this.play(),
      pause: () => this.pause(),
      record: () => this.recordForParity(),
      starts: () => this.episode?.wd.starts.map((s, i) => ({
        index: i,
        kind: s.kind,
        case: s.case,
        start_t: s.start_t,
        total: (this.episode?.wd.references[s.motion].rows ?? 0) - s.start_t,
        label: startLabel(s, this.viewer ? null : this.policyId)
      })) ?? [],
      ready: () => this.episode !== null && this.panel !== null,
      forceBlowUp: () => {
        this.forceBlowUp = true;
      },
      overlays: () => this.overlays(),
      inset: () => this.insetStatus(),
      look: () => this.look?.status() ?? null,
      arm: () => this.armStatus()
    };
  }
  // ------------------------------------------------------------------ CommandTerm
  getCommand() {
    return this.cmd;
  }
  /** mjswan's reset chain (first load, its Reset button, `R`, and our `requestReset`). */
  async reset() {
    await this.ready;
    if (this.disposed) return;
    this.applyReset();
  }
  /** StudentObs: the observation for this control step. */
  observe() {
    this.markObs = performance.now();
    const ep = this.episode;
    if (!ep) return this.zeros;
    if (this.viewer) {
      runtime()?.pause();
      this.panel?.setPaused(true);
      return this.zeros;
    }
    if (ep.done) this.applyReset();
    if (this.pauseOnFirst) {
      this.pauseOnFirst = false;
      runtime()?.pause();
      return this.zeros;
    }
    if (ep instanceof ArmEpisode) ep.startPhases();
    return ep.observe();
  }
  onSubstep(context, substep, decimation) {
    const ep = this.episode;
    if (!ep) return;
    if (substep === 0) {
      if (!this.decimationChecked) {
        this.decimationChecked = true;
        if (decimation !== ep.env.decimation)
          console.error(
            `[mjweb] mjswan decimation ${decimation} != ${ep.env.decimation}`
          );
      }
      const now = performance.now();
      if (this.markObs > 0)
        this.inferMs = this.ema(this.inferMs, now - this.markObs);
      this.markPhysics = now;
      const inferenceError = runtime()?.lastInferenceError;
      if (inferenceError && !ep.done) {
        ep.failInference(String(inferenceError));
        runtime()?.pause();
        this.panel?.update(this.liveStats(true), true);
      }
      ep.beginStep(context.readOnnxSlot?.({ action: "action" }) ?? null);
    }
    ep.preSubstep(substep);
    if (this.forceBlowUp && substep === 5 && context.mjData) {
      this.forceBlowUp = false;
      context.mjData.xfrc_applied[6 * ep.L.objectBody + 3] = 1e300;
    }
  }
  onSubstepEnd(_context, substep, decimation) {
    const ep = this.episode;
    if (!ep) return;
    ep.postSubstep(substep);
    if (substep === decimation - 1 && this.markPhysics > 0) {
      this.physicsMs = this.ema(
        this.physicsMs,
        performance.now() - this.markPhysics
      );
    }
  }
  update(_dt) {
    const ep = this.episode;
    if (!ep) return;
    const result = ep.endStep();
    if (result === "stale") this.applyReset();
    if (result === "step" || result === "done" || result === "unstable") {
      this.totalSteps += 1;
      this.countRate();
      const now = performance.now();
      if (ep.samples === 1) {
        this.episodeT0 = now;
        this.minWindowRate = Infinity;
      }
      this.episodeT1 = now;
    }
    const ended = result === "done" || result === "unstable";
    if (ended) {
      this.episodesDone += 1;
      if (result === "unstable")
        console.warn(
          `[mjweb] MuJoCo became numerically unstable at sample ${ep.samples}: episode ended`
        );
      runtime()?.pause();
    }
    this.syncGhosts();
    this.look?.step();
    this.panel?.update(this.liveStats(ended), ended);
    this.syncArm(ended);
  }
  /** The arm's panel block and the wrist frames (commanded vs actual). */
  syncArm(force) {
    const ep = this.episode;
    if (!(ep instanceof ArmEpisode)) return;
    this.armPanel?.update(ep, force);
    const env = ep.armEnv;
    const s = env.state;
    this.frames?.setVisible(this.wristFrames && env.phase === "policy");
    this.frames?.update(env.goalPos, env.goalQuat, s.wristPos, s.wristQuat);
  }
  armStatus() {
    const ep = this.episode;
    if (!(ep instanceof ArmEpisode))
      return { base: this.base, pairScene: this.pairScene };
    const env = ep.armEnv;
    const { q, qd } = env.armJoints();
    return {
      base: this.base,
      pairScene: this.pairScene,
      objectGhost: ep.armSettings.objectGhost,
      objectGhostVisible: this.ghosts?.objectGhostVisible ?? null,
      wristFrames: this.wristFrames,
      wristFramesVisible: this.frames?.visible ?? null,
      unstableReason: ep.unstableReason,
      mode: env.mode,
      phases: ep.armSettings.phases,
      kind: ep.armSettings.kind,
      phase: env.phase,
      phaseLabel: ep.phaseLabel,
      inPhases: ep.inPhases,
      phaseTime: env.phaseTime,
      phaseDuration: env.phaseDuration,
      log: { ...env.log, speedFrac: [...env.log.speedFrac] },
      contactWrench: Array.from(env.contactWrench),
      q,
      qd,
      goal: { pos: env.goalPos, quat: env.goalQuat },
      placementDefault: env.placementMatches(
        ep.arm.placements[ep.armSettings.kind]?.placement ?? ep.arm.placement
      ),
      branchQ: env.branchQ.slice(),
      mountYaw: env.mountYaw()
    };
  }
  debugVisEnabled() {
    return null;
  }
  dispose() {
    this.disposed = true;
    this.armPanel?.dispose();
    this.frames?.dispose();
    if (session.controller === this) session.controller = null;
    this.panel?.dispose();
    this.ghosts?.dispose();
    this.look?.dispose();
    this.look = null;
    this.insetEl?.remove();
    this.insetEl = null;
    this.episode?.dispose();
    this.restoreFloor?.();
    this.restoreFloor = null;
    if (this.previousBackground !== void 0) {
      this.context.scene.background = this.previousBackground;
    }
    this.panel = null;
    this.ghosts = null;
    this.episode = null;
  }
  // ------------------------------------------------------------------ actions
  applyReset() {
    const ep = this.episode;
    if (!ep) return;
    ep.reset();
    this.look?.reset();
    this.syncGhosts();
    this.panel?.refresh();
    this.panel?.update(this.liveStats(false), true);
  }
  /** Through mjswan's reset chain, so a paused view is redrawn as well. */
  requestReset() {
    if (this.context.requestReset) this.context.requestReset();
    else this.applyReset();
  }
  play() {
    if (this.viewer) return;
    const ep = this.episode;
    if (ep?.done) this.requestReset();
    if (ep instanceof ArmEpisode) ep.startPhases();
    runtime()?.play();
    this.panel?.setPaused(false);
  }
  pause() {
    runtime()?.pause();
    this.panel?.setPaused(true);
  }
  selectStart(index) {
    const ep = this.episode;
    if (!ep) return;
    ep.selectStart(index);
    this.writeUrl({ start: String(index) });
    this.requestReset();
  }
  setSettings(s) {
    const ep = this.episode;
    if (!ep) return;
    ep.settings = { ...ep.settings, ...s };
    this.writeUrl({
      sense: ep.settings.sense ? "1" : "0",
      pert: ep.settings.perturb ? "1" : "0",
      pseed: String(ep.settings.seed)
    });
    this.requestReset();
  }
  setVisibility(v) {
    this.visibility = { ...v };
    this.ghosts?.setVisibility(this.visibility);
    this.syncGhosts();
    this.writeUrl(
      { ghost: v.reference ? "1" : "0", fginset: v.inset ? "1" : "0" },
      ["goal", "grasp", "demo"]
      // the removed overlays' old parameters
    );
  }
  /**
   * The functional-grasp image of the current reference in the lower-right corner (beside the arm panel in the
   * arm view): the report clips' own inset, pre-rendered per reference into the web data. Without one (the
   * legacy bundle) nothing is shown.
   */
  updateInset() {
    if (this.look && this.episode && this.visibility.inset) this.look.ensureInset(this.episode.ref);
    const src = this.episode?.ref.fg_inset;
    if (!this.visibility.inset || !src) {
      if (this.insetEl) this.insetEl.style.display = "none";
      return;
    }
    if (!this.insetEl) {
      const box = document.createElement("div");
      box.setAttribute("data-testid", "mjweb-fginset-image");
      box.style.cssText = `position: fixed; bottom: 12px; z-index: 900; pointer-events: none; right: ${this.base === "arm" ? 376 : 12}px; box-shadow: 0 2px 10px rgba(0, 0, 0, 0.25);`;
      const img2 = document.createElement("img");
      img2.alt = "functional grasp\uFF08\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF09";
      img2.style.cssText = "display: block; width: 320px; height: auto;";
      box.append(img2);
      this.look?.decorateInset(box);
      document.body.append(box);
      this.insetEl = box;
    }
    const img = this.insetEl.querySelector("img");
    if (img && img.getAttribute("src") !== src) img.setAttribute("src", src);
    this.insetEl.style.display = "block";
  }
  /** Test hook: is the corner image shown, and for which reference. */
  insetStatus() {
    const img = this.insetEl?.querySelector("img") ?? null;
    return {
      visible: !!this.insetEl && this.insetEl.style.display !== "none",
      case: this.episode?.ref.case ?? null,
      matches: !!img && img.getAttribute("src") === this.episode?.ref.fg_inset,
      srcLength: img?.getAttribute("src")?.length ?? 0,
      available: !!this.episode?.ref.fg_inset
    };
  }
  /** Object / variant / seed switch: a new page for the scene and policy (mjswan's URL ids). */
  navigate(sceneId, policyKey, extra = {}) {
    const p = params();
    const current = p.get("scene") ?? this.sceneId;
    const sameObject = sceneId === current || sceneId === this.pairScene || sceneId.replace(/_arm$/, "") === this.objectId;
    p.set("scene", sceneId);
    p.set("policy", policyKey);
    for (const [k, v] of Object.entries(extra)) p.set(k, v);
    if (!sameObject) p.delete("start");
    window.location.search = p.toString();
  }
  writeUrl(values, drop = []) {
    if (typeof window === "undefined") return;
    const p = params();
    for (const [k, v] of Object.entries(values)) p.set(k, v);
    for (const k of drop) p.delete(k);
    window.history.replaceState(
      {},
      "",
      `${window.location.pathname}?${p.toString()}${window.location.hash}`
    );
  }
  // ------------------------------------------------------------------ display
  /**
   * The overlays for the displayed state: the two relational ones follow the live bodies (the
   * object relative to the live wrist, the hand relative to the live object), as drawn (`xpos` /
   * `xquat` after mjswan's `mj_forward`).
   */
  syncGhosts() {
    this.updateInset();
    const ep = this.episode;
    const g = this.ghosts;
    if (!ep || !g) return;
    if (ep instanceof ArmEpisode) {
      const pending = ep.pendingStart;
      if (pending) {
        g.setVisibility({ reference: false, inset: this.visibility.inset });
        const o = ep.startLayout.objectQpos;
        if (ep.armSettings.objectGhost)
          g.setObjectGhost(
            pending.qpos.slice(o, o + 3),
            pending.qpos.slice(o + 3, o + 7)
          );
        runtime()?.requestRender?.();
        return;
      }
      g.setVisibility(this.visibility);
    }
    if (this.visibility.reference)
      g.setReferenceRow(ep.ref, ep.start.start_t + ep.samples);
    runtime()?.requestRender?.();
  }
  /** Test hook: the overlays drawn now (`Ghosts.drawn`): the reference ghost only, since 10-20. */
  overlays() {
    const ep = this.episode;
    const g = this.ghosts;
    if (!ep || !g) return null;
    this.syncGhosts();
    return {
      ...g.drawn(),
      visibility: { ...this.visibility },
      case: ep.ref.case
    };
  }
  ema(old, v) {
    return old === 0 ? v : 0.9 * old + 0.1 * v;
  }
  countRate() {
    const now = performance.now();
    if (this.windowStart === 0) this.windowStart = now;
    this.windowSteps += 1;
    const span = now - this.windowStart;
    if (span >= 1e3) {
      this.controlRate = this.windowSteps * 1e3 / span;
      if (runtime()?.isRunning && this.episode && this.episode.samples > 30)
        this.minWindowRate = Math.min(this.minWindowRate, this.controlRate);
      this.windowStart = now;
      this.windowSteps = 0;
    }
  }
  liveStats(paused) {
    return {
      controlRate: this.controlRate,
      realtime: this.controlRate / 30,
      physicsMs: this.physicsMs,
      inferMs: this.inferMs,
      paused: paused || !(runtime()?.isRunning ?? true),
      unpaced: !!globalThis.__mjswanUnpaced
    };
  }
  status() {
    const ep = this.episode;
    return {
      ready: ep !== null && this.panel !== null,
      error: this.loadError,
      objectId: this.objectId,
      policyId: this.policyId,
      policyKey: this.policyKey,
      hand: this.hand,
      viewer: this.viewer,
      physicsTag: ep?.wd.physics.tag ?? null,
      row: ep?.policy?.row ?? null,
      checkpoint: ep?.policy?.checkpoint_name ?? null,
      startIndex: ep?.startIndex ?? -1,
      start: ep ? {
        kind: ep.start.kind,
        case: ep.start.case,
        start_t: ep.start.start_t,
        seed: ep.start.seed ?? null
      } : null,
      samples: ep?.samples ?? 0,
      total: ep?.total ?? 0,
      done: ep?.done ?? false,
      success: ep?.success ?? false,
      fgSuccess: ep?.fgSuccess ?? false,
      mainOutcome: ep?.mainOutcome ?? null,
      fgOutcome: ep?.fgOutcome ?? null,
      unstable: ep?.unstable ?? false,
      unstableSample: ep?.unstableSample ?? -1,
      firstLost: ep?.score.firstLost ?? -1,
      wristMode: this.wristMode,
      counts: ep?.score.counts ?? 0,
      fgCounts: ep?.score.fgCounts ?? 0,
      epoch: ep?.epoch ?? 0,
      settings: ep?.settings ?? null,
      perturbed: !!ep?.perturbation,
      running: runtime()?.isRunning ?? false,
      unpaced: !!globalThis.__mjswanUnpaced,
      controlRate: this.controlRate,
      physicsMs: this.physicsMs,
      inferMs: this.inferMs,
      totalSteps: this.totalSteps,
      episodesDone: this.episodesDone,
      // simulated / wall time from the episode's first to its latest step, and the slowest 1-s window
      episodeRealtime: ep && ep.samples > 1 && this.episodeT1 > this.episodeT0 ? (ep.samples - 1) / 30 / ((this.episodeT1 - this.episodeT0) / 1e3) : null,
      minWindowRate: Number.isFinite(this.minWindowRate) ? this.minWindowRate : null,
      isaac: ep?.isaacOutcome() ?? null,
      last: ep?.last ?? null,
      mouseMode: runtime()?.getInteractionMode() ?? null,
      external: this.externalWrenches()
    };
  }
  /**
   * Bodies whose `xfrc_applied` row holds a force from someone else (the viewer's Pull /
   * Push): every non-zero row, except that the wrist row counts only what was there before our
   * wrench was added (`HandEnv.base`). Lets a test check the wrench never overwrites a pull.
   */
  externalWrenches() {
    const ep = this.episode;
    const d = this.context.mjData;
    if (!ep || !d) return [];
    const xfrc = d.xfrc_applied;
    const out = [];
    const n = xfrc.length / 6;
    for (let b = 1; b < n; b++) {
      const row = b === ep.L.wristBody ? ep.env.base : xfrc.subarray(6 * b, 6 * b + 6);
      const f = Math.sqrt(row[0] * row[0] + row[1] * row[1] + row[2] * row[2]);
      if (f > 0) out.push({ body: b, force: f });
    }
    return out;
  }
  recordForParity() {
    const ep = this.episode;
    if (!ep) return null;
    return {
      object_id: this.objectId,
      policy_id: this.policyId,
      start_index: ep.startIndex,
      kind: ep.start.kind,
      case: ep.start.case,
      start_t: ep.start.start_t,
      samples: ep.samples,
      done: ep.done,
      success: ep.success,
      fg_success: ep.fgSuccess,
      main_outcome: ep.mainOutcome,
      fg_outcome: ep.fgOutcome,
      first_lost: ep.score.firstLost,
      unstable: ep.unstable,
      unstable_sample: ep.unstableSample,
      wrist_mode: this.wristMode,
      settings: ep.settings,
      ...toParity(ep.record)
    };
  }
};

// src/.plugins-entry.ts
var observations = { StudentObs };
var commands = { StudentController };
export {
  commands,
  observations
};

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
    var THREE2 = globalThis.__mjswanThree;
    if (!THREE2) {
      throw new Error(
        "mjswan: the engine must be loaded before a custom-MDP plugin (three not initialized)."
      );
    }
    module.exports = THREE2;
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
  constructor(mujoco, model, data, wd, policyId, settings, wristMode = "isaac") {
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
    this.L = layoutOf(mujoco, model, wd.controller.joint_order);
    this.env = makeEnv(
      mujoco,
      model,
      data,
      wd,
      this.L,
      policy?.target_ema.wrist ?? 0.3,
      policy?.target_ema.fingers ?? 0.3,
      wristMode
    );
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
        this.L,
        this.perturbation
      ));
    }
    const noise = this.settings.sense ? this.wd.sense ?? EVAL_SENSE : EXACT;
    this.view = new StudentView(
      noise,
      this.settings.sense ? new Rng(this.settings.seed, 1) : null
    );
    const s = this.env.reset(qpos, qvel);
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
  reference: 4892927,
  goal: 3134074,
  grasp: 16757596,
  demo: 11758591
};
function setPose(obj, p, q) {
  obj.position.set(p[0], p[2], -p[1]);
  obj.quaternion.set(-q[1], -q[3], q[2], -q[0]);
}
function getPose(obj) {
  const p = obj.position;
  const q = obj.quaternion;
  return { pos: [p.x, -p.z, p.y], quat: [-q.w, -q.x, q.z, -q.y] };
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
  g.addColorStop(0, "#59647a");
  g.addColorStop(0.55, "#323a49");
  g.addColorStop(1, "#1b1f27");
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 4, 256);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  scene.background = tex;
  return previous;
}
var Ghosts = class {
  mujoco;
  model;
  fk;
  L;
  root;
  handBodies;
  handClones;
  graspClones;
  /** Pose of every hand body in the demo object frame at the demo's last row. */
  graspRel = [];
  refObject;
  goalObject;
  demoObject;
  materials;
  visibility = {
    reference: true,
    goal: true,
    grasp: true,
    demo: false
  };
  constructor(mujoco, model, bodies, parent, L) {
    this.mujoco = mujoco;
    this.model = model;
    this.L = L;
    this.fk = new mujoco.MjData(model);
    this.root = new THREE.Group();
    this.root.name = "mjweb ghosts";
    this.root.userData = { ignoreDragForce: true };
    const refMat = ghostMaterial(GHOST_COLORS.reference, 0.22);
    const goalMat = ghostMaterial(GHOST_COLORS.goal, 0.38);
    const graspMat = ghostMaterial(GHOST_COLORS.grasp, 0.24);
    const demoMat = ghostMaterial(GHOST_COLORS.demo, 0.25);
    this.materials = [refMat, goalMat, graspMat, demoMat];
    const rootId = model.body_rootid;
    this.handBodies = [];
    for (let b = 1; b < model.nbody; b++)
      if (rootId[b] === L.wristBody && bodies[b]) this.handBodies.push(b);
    this.handClones = this.handBodies.map(
      (b) => cloneBody(bodies[b], refMat, `ghost ${bodies[b].name}`)
    );
    this.graspClones = this.handBodies.map(
      (b) => cloneBody(
        bodies[b],
        graspMat,
        `ghost ${bodies[b].name} (demo last row, relative to the object)`
      )
    );
    const src = bodies[L.objectBody];
    this.refObject = src ? cloneBody(src, refMat, "ghost object (reference)") : null;
    this.goalObject = src ? cloneBody(
      src,
      goalMat,
      "ghost object (demo last row, relative to the wrist)"
    ) : null;
    this.demoObject = src ? cloneBody(src, demoMat, "ghost object (demo last row, world)") : null;
    for (const c of [
      ...this.handClones,
      ...this.graspClones,
      this.refObject,
      this.goalObject,
      this.demoObject
    ])
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
    for (const c of this.graspClones) c.visible = this.visibility.grasp;
    if (this.refObject) this.refObject.visible = this.visibility.reference;
    if (this.goalObject) this.goalObject.visible = this.visibility.goal;
    if (this.demoObject) this.demoObject.visible = this.visibility.demo;
  }
  /** Forward kinematics of hand + object on the ghost `MjData`. */
  kinematics(wristPos, wristQuat, joints, objectPos, objectQuat) {
    const L = this.L;
    const qpos = this.fk.qpos;
    qpos.fill(0);
    qpos.set(wristPos, L.wristQpos);
    qpos.set(wristQuat, L.wristQpos + 3);
    for (let i = 0; i < 20; i++) qpos[L.fingerQpos[i]] = joints[i];
    qpos.set(objectPos, L.objectQpos);
    qpos.set(objectQuat, L.objectQpos + 3);
    this.mujoco.mj_kinematics(this.model, this.fk);
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
  /** The demo's last-row hand in the demo object frame (once per reference). */
  setGraspReference(goal) {
    this.kinematics(
      goal.wrist_pos,
      goal.wrist_quat,
      goal.joints,
      goal.object_pos,
      goal.object_quat
    );
    const xpos = this.fk.xpos;
    const xquat = this.fk.xquat;
    this.graspRel = this.handBodies.map(
      (b) => poseInFrame(
        goal.object_pos,
        goal.object_quat,
        xpos.subarray(3 * b, 3 * b + 3),
        xquat.subarray(4 * b, 4 * b + 4)
      )
    );
  }
  /** The demo's last-row hand around the live object (its relation to the object = the demo's). */
  setGrasp(objectPos, objectQuat) {
    this.graspRel.forEach((rel, i) => {
      const w = poseFromFrame(objectPos, objectQuat, rel.pos, rel.quat);
      setPose(this.graspClones[i], w.pos, w.quat);
    });
  }
  /** The demo's last-row object attached to the live wrist (its relation to the wrist = the demo's). */
  setGoal(wristPos, wristQuat, goal) {
    if (!this.goalObject) return;
    const off = quatApply(wristQuat, goal.object_pos_in_wrist);
    const p = [
      wristPos[0] + off[0],
      wristPos[1] + off[1],
      wristPos[2] + off[2]
    ];
    setPose(this.goalObject, p, quatMul(wristQuat, goal.object_quat_in_wrist));
  }
  setDemo(goal) {
    if (this.demoObject)
      setPose(this.demoObject, goal.object_pos, goal.object_quat);
  }
  /**
   * Test hook: read the drawn wrist ghost back from three.js, express it in the live object frame
   * and compare with the demo's wrist in the demo's object frame (straight from the goal row).
   */
  graspCheck(objectPos, objectQuat, goal) {
    const i = this.handBodies.indexOf(this.L.wristBody);
    const want = poseInFrame(
      goal.object_pos,
      goal.object_quat,
      goal.wrist_pos,
      goal.wrist_quat
    );
    let got = null;
    if (i >= 0) {
      const drawn = getPose(this.graspClones[i]);
      got = poseInFrame(objectPos, objectQuat, drawn.pos, drawn.quat);
    }
    const posErr = got ? Math.sqrt(got.pos.reduce((a, v, k) => a + (v - want.pos[k]) ** 2, 0)) : NaN;
    return {
      visible: this.graspClones.length > 0 && this.graspClones.every((c) => c.visible),
      bodies: this.graspClones.length,
      wristPosErrM: posErr,
      wristRotErrRad: got ? quatAngle(got.quat, want.quat) : NaN
    };
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
  goal: "\u793A\u8303\u6700\u540E\u4E00\u5E27\u91CC\u7269\u4F53\u5728\u624B\u8155\u5750\u6807\u7CFB\u4E2D\u7684\u4F4D\u7F6E\u548C\u59FF\u6001\uFF0C\u6302\u5728\u5F53\u524D\u624B\u8155\u4E0A\uFF08\u7EFF\uFF09\uFF1Amain \u7684\u7269\u4F53\u5173\u952E\u70B9\u6D4B\u8BD5\u548C FG-v1 \u7684\u4F4D\u7F6E / \u65CB\u8F6C\u6D4B\u8BD5\u6BD4\u7684\u5C31\u662F\u8FD9\u4E2A\u76F8\u5BF9\u5173\u7CFB\u3002",
  grasp: "\u793A\u8303\u6700\u540E\u4E00\u5E27\u7684\u624B\uFF08\u624B\u8155\u4F4D\u59FF + 20 \u4E2A\u5173\u8282\uFF0C\u540C\u53C2\u8003\u6B8B\u5F71\u7684 FK\uFF09\uFF0C\u6309\u5B83\u76F8\u5BF9\u7269\u4F53\u7684\u4F4D\u7F6E\u653E\u5728\u5F53\u524D\u7269\u4F53\u5468\u56F4\uFF08\u6A59\uFF09\uFF1Amain \u7684\u624B\u90E8 26 \u70B9\u6D4B\u8BD5\u548C FG-v1 \u7684\u8FD1\u90BB\u5BF9 / \u63A5\u89E6\u6D4B\u8BD5\u6BD4\u7684\u5C31\u662F\u8FD9\u4E2A\u76F8\u5BF9\u5173\u7CFB\u3002",
  demo: "\u793A\u8303\u6700\u540E\u4E00\u5E27\u7269\u4F53\u5728\u4E16\u754C\u91CC\u7684\u4F4D\u7F6E\uFF08\u7D2B\uFF09\uFF1A\u53EA\u4F5C\u53C2\u8003\uFF0C\u4E24\u4E2A\u5224\u636E\u90FD\u4E0D\u7528\u4E16\u754C\u5750\u6807\u3002",
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
function rowLabel(catalog, row) {
  if (row === null || row === void 0) return "\u6846\u67B6 \u2014";
  const what = catalog.rows?.[String(row)];
  return what ? `\u6846\u67B6 ${row}\uFF08${what}\uFF09` : `\u6846\u67B6 ${row}`;
}
var pct = (v) => v === null || v === void 0 ? "\u2014" : `${v}%`;
var NO_CHECKPOINT = "\u8FD8\u6CA1\u6709 Hand 2.1 checkpoint\uFF1A\u8FD9\u91CC\u53EA\u663E\u793A Hand 2.1 \u573A\u666F\u3001\u6BCF\u6761 reference \u7684\u8D77\u59CB\u6293\u53D6\u548C functional-grasp \u6B8B\u5F71\uFF0C\u4E0D\u8FD0\u884C\u7B56\u7565\uFF08\u5B98\u65B9\u7269\u7406\u4E0A\u91CD\u8BAD\u7684\u5B66\u751F\u51FA\u6765\u540E\u81EA\u52A8\u52A0\u5165\uFF09\u3002";
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
        const k = p.row === null || p.row === void 0 ? "\u2014" : String(p.row);
        if (!byRow.has(k)) byRow.set(k, []);
        byRow.get(k).push(p);
      }
      for (const [row, ps] of byRow) {
        const line = el("div", { class: "prow", "data-row": row });
        line.append(
          el(
            "span",
            { class: "rl", title: rowLabel(catalog, ps[0].row) },
            row === "\u2014" ? "\u5176\u4ED6" : `\u6846\u67B6 ${row}`
          )
        );
        const many = new Set(ps.map((p) => p.seed)).size !== ps.length;
        for (const p of ps) {
          const text = `seed ${p.seed}${many ? ` \xB7 it ${p.iteration}` : ""}${p.default ? "\uFF08\u9ED8\u8BA4\uFF09" : ""}`;
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
      goal: this.state.goal,
      grasp: this.state.grasp,
      demo: this.state.demo
    });
    const overlays = [
      [
        "reference",
        "\u53C2\u8003\u6B8B\u5F71\uFF1Areference \u56DE\u653E",
        "mjweb-ghost",
        GHOST_COLORS.reference,
        TIP.ghost
      ],
      [
        "goal",
        "\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF1A\u7269\u4F53\u76F8\u5BF9\u624B\u8155",
        "mjweb-goal",
        GHOST_COLORS.goal,
        TIP.goal
      ],
      [
        "grasp",
        "functional grasp\uFF1A\u793A\u8303\u6700\u540E\u4E00\u5E27\u7684\u624B\uFF08\u76F8\u5BF9\u7269\u4F53\uFF09",
        "mjweb-grasp",
        GHOST_COLORS.grasp,
        TIP.grasp
      ],
      [
        "demo",
        "\u793A\u8303\u6700\u540E\u4E00\u5E27\uFF1A\u7269\u4F53\u5728\u4E16\u754C\u91CC\uFF08\u4E0D\u53C2\u4E0E\u5224\u636E\uFF09",
        "mjweb-demo",
        GHOST_COLORS.demo,
        TIP.demo
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
          "MuJoCo\uFF1A\u6CA1\u6709\u7B56\u7565\uFF08\u8FD8\u6CA1\u6709 Hand 2.1 checkpoint\uFF09\uFF0C\u4E0D\u8FD0\u884C\uFF1B\u6B8B\u5F71\u662F\u8FD9\u6761 reference \u793A\u8303\u6700\u540E\u4E00\u5E27\u7684\u624B\u2013\u7269\u4F53\u5173\u7CFB"
        )
      );
      return;
    }
    if (ep.done && ep.inferenceError) {
      line(span(`MuJoCo\uFF1A\u7B56\u7565\u63A8\u7406\u5931\u8D25\uFF08ONNX\uFF09\uFF1A${ep.inferenceError}`, false));
    } else if (ep.done && mo === "unstable") {
      line(
        span(
          `MuJoCo\uFF1A${UNSTABLE}\uFF0C\u7B2C ${ep.unstableSample} \u4E2A\u6837\u672C\u540E episode \u7ED3\u675F\uFF08main\u3001FG-v1 \u90FD\u7B97\u5931\u8D25\uFF09`,
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
          dim(`\uFF08\u7269\u4F53\u6389\u4E86\u4EE5\u540E\uFF0C\u7B2C ${ep.unstableSample} \u4E2A\u6837\u672C\u540E\u53C8\u6570\u503C\u53D1\u6563\uFF09`)
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
  ghosts = null;
  /** The reference whose last row the hand ghost currently shows. */
  graspRef = null;
  previousBackground = void 0;
  visibility = {
    reference: true,
    goal: true,
    grasp: true,
    demo: false
  };
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
    this.viewer = config.viewer === true || this.policyId === "";
    const p = params();
    this.pauseOnFirst = this.viewer || !flag(p, "play", true);
    globalThis.__mjswanUnpaced = flag(p, "fast", false);
    globalThis.__mjswanRenderOnChange = p.get("render") !== "all";
    this.visibility = {
      reference: flag(p, "ghost", true),
      goal: flag(p, "goal", true),
      grasp: flag(p, "grasp", true),
      demo: flag(p, "demo", false)
    };
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
    const ep = new Episode(
      mujoco,
      mjModel,
      mjData,
      wd,
      this.viewer ? null : this.policyId,
      {
        sense: flag(p, "sense", true),
        perturb: flag(p, "pert", false),
        seed: Math.trunc(Number(p.get("pseed") ?? 0)) || 0
      },
      this.wristMode
    );
    ep.recording = flag(p, "record", false);
    const start = Number(p.get("start"));
    if (Number.isInteger(start) && start >= 0 && start < wd.starts.length)
      ep.selectStart(start);
    this.episode = ep;
    this.previousBackground = applyBackdrop(this.context.scene);
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
    }
    const rt = runtime();
    const modes = rt?.interactionModes() ?? [];
    const mouse = p.get("mouse") ?? "pull";
    const mouseMode = modes.some((m) => m.id === mouse && m.available) ? mouse : "view";
    rt?.setInteractionMode(mouseMode);
    this.panel = new MjwebPanel(
      catalog,
      ep,
      {
        sceneId: this.objectId,
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
      graspCheck: () => this.graspCheck()
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
    this.panel?.update(this.liveStats(ended), ended);
  }
  debugVisEnabled() {
    return null;
  }
  dispose() {
    this.disposed = true;
    if (session.controller === this) session.controller = null;
    this.panel?.dispose();
    this.ghosts?.dispose();
    this.episode?.dispose();
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
    this.writeUrl({
      ghost: v.reference ? "1" : "0",
      goal: v.goal ? "1" : "0",
      grasp: v.grasp ? "1" : "0",
      demo: v.demo ? "1" : "0"
    });
  }
  /** Object / variant / seed switch: a new page for the scene and policy (mjswan's URL ids). */
  navigate(sceneId, policyKey) {
    const p = params();
    const sameScene = sceneId === this.objectId;
    p.set("scene", sceneId);
    p.set("policy", policyKey);
    if (!sameScene) p.delete("start");
    window.location.search = p.toString();
  }
  writeUrl(values) {
    if (typeof window === "undefined") return;
    const p = params();
    for (const [k, v] of Object.entries(values)) p.set(k, v);
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
    const ep = this.episode;
    const g = this.ghosts;
    if (!ep || !g) return;
    if (this.visibility.reference)
      g.setReferenceRow(ep.ref, ep.start.start_t + ep.samples);
    const d = this.context.mjData;
    if (d) {
      const xpos = d.xpos;
      const xquat = d.xquat;
      if (this.visibility.goal) {
        const b = ep.L.wristBody;
        g.setGoal(
          xpos.subarray(3 * b, 3 * b + 3),
          xquat.subarray(4 * b, 4 * b + 4),
          ep.ref.goal
        );
      }
      if (this.visibility.grasp) {
        if (this.graspRef !== ep.ref) {
          g.setGraspReference(ep.ref.goal);
          this.graspRef = ep.ref;
        }
        const o = ep.L.objectBody;
        g.setGrasp(
          xpos.subarray(3 * o, 3 * o + 3),
          xquat.subarray(4 * o, 4 * o + 4)
        );
      }
    }
    if (this.visibility.demo) g.setDemo(ep.ref.goal);
    runtime()?.requestRender?.();
  }
  /** Test hook: the drawn hand ghost's relation to the live object against the demo's (`Ghosts.graspCheck`). */
  graspCheck() {
    const ep = this.episode;
    const g = this.ghosts;
    const d = this.context.mjData;
    if (!ep || !g || !d) return null;
    this.syncGhosts();
    const o = ep.L.objectBody;
    const xpos = d.xpos;
    const xquat = d.xquat;
    return {
      ...g.graspCheck(
        xpos.subarray(3 * o, 3 * o + 3),
        xquat.subarray(4 * o, 4 * o + 4),
        ep.ref.goal
      ),
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

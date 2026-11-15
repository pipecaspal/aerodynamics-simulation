
/**
 * Aerodynamics Lab
 * Hess-Smith-Style Panel Method Solver
 *
 * Computes 2D inviscid potential flow
 * around a closed airfoil.
 *
 * Model:
 * - Constant-strength source panels
 * - One uniform vortex-sheet strength
 * - No-penetration boundary condition
 * - Trailing-edge Kutta condition
 *
 * Preserves the numerical formulation
 * previously used in App.tsx.
 */

import type { Panel } from './airfoil'

// =====================================
// SOLVER RESULT TYPE
// =====================================

export type PanelSolution = {
  cp: number[]
  vt: number[]
  cl: number
  strengths: number[]
  normalResidual: number
  sourceFlux: number
  kuttaResidual: number
}

// =====================================
// GAUSSIAN ELIMINATION
// WITH PARTIAL PIVOTING
// =====================================

export function solveLinear(
  input: number[][],
  values: number[]
): number[] {
  const n = values.length

  const a = input.map(row => [...row])
  const b = [...values]

  // Forward elimination

  for (let k = 0; k < n; k++) {
    let pivot = k

    // Select the largest pivot
    // in the current column.

    for (let i = k + 1; i < n; i++) {
      if (
        Math.abs(a[i][k]) >
        Math.abs(a[pivot][k])
      ) {
        pivot = i
      }
    }

    if (Math.abs(a[pivot][k]) < 1e-12) {
      throw new Error(
        'Singular panel matrix'
      )
    }

    // Swap pivot rows.

    ;[a[k], a[pivot]] = [
      a[pivot],
      a[k],
    ]

    ;[b[k], b[pivot]] = [
      b[pivot],
      b[k],
    ]

    // Eliminate entries below pivot.

    for (let i = k + 1; i < n; i++) {
      const factor =
        a[i][k] / a[k][k]

      a[i][k] = 0

      for (let j = k + 1; j < n; j++) {
        a[i][j] -=
          factor * a[k][j]
      }

      b[i] -= factor * b[k]
    }
  }

  // Back substitution

  const solution =
    Array(n).fill(0) as number[]

  for (let i = n - 1; i >= 0; i--) {
    let total = b[i]

    for (let j = i + 1; j < n; j++) {
      total -=
        a[i][j] * solution[j]
    }

    solution[i] =
      total / a[i][i]
  }

  return solution
}

// =====================================
// SOURCE PANEL INFLUENCE
// =====================================

export function sourceVelocity(
  x: number,
  y: number,
  panel: Panel
) {
  // Convert the evaluation point
  // into panel-local coordinates.

  const dx = x - panel.ax
  const dy = y - panel.ay

  const xi =
    dx * panel.tx +
    dy * panel.ty

  const eta =
    dx * panel.nx +
    dy * panel.ny

  // Distances to panel endpoints,
  // expressed as squared distances.

  const r1 = Math.max(
    xi * xi + eta * eta,
    1e-14
  )

  const r2 = Math.max(
    (xi - panel.len) ** 2 +
    eta ** 2,
    1e-14
  )

  // Local tangential and normal
  // velocity influence coefficients.

  const ut =
    Math.log(r1 / r2) /
    (4 * Math.PI)

  const un = Math.atan2(
    eta * panel.len,
    xi * (xi - panel.len) +
      eta * eta
  ) / (2 * Math.PI)

  // Convert back to global
  // Cartesian velocity components.

  return {
    u:
      ut * panel.tx +
      un * panel.nx,

    v:
      ut * panel.ty +
      un * panel.ny,
  }
}

// =====================================
// MAIN PANEL METHOD SOLVER
// =====================================

export function solvePanelMethod(
  panels: Panel[],
  half: number,
  angle: number
): PanelSolution {
  const n = panels.length

  // Influence matrices:
  // n source strengths plus
  // one shared vortex strength.

  const normal = Array.from(
    { length: n + 1 },
    () =>
      Array(n + 1).fill(0) as number[]
  )

  const tangent = Array.from(
    { length: n },
    () =>
      Array(n + 1).fill(0) as number[]
  )

  // =====================================
  // BUILD INFLUENCE MATRICES
  // =====================================

  for (let i = 0; i < n; i++) {
    const pi = panels[i]

    for (let j = 0; j < n; j++) {
      const pj = panels[j]

      // Retain the existing diagonal
      // self-influence treatment.

      const source = i === j
        ? {
            u: 0.5 * pj.nx,
            v: 0.5 * pj.ny,
          }
        : sourceVelocity(
            pi.cx,
            pi.cy,
            pj
          )

      const sx = source.u
      const sy = source.v

      // A vortex-sheet influence is
      // perpendicular to the source
      // influence in this formulation.

      const vx = -sy
      const vy = sx

      normal[i][j] =
        sx * pi.nx +
        sy * pi.ny

      tangent[i][j] =
        sx * pi.tx +
        sy * pi.ty

      normal[i][n] +=
        vx * pi.nx +
        vy * pi.ny

      tangent[i][n] +=
        vx * pi.tx +
        vy * pi.ty
    }
  }

  // =====================================
  // FREESTREAM
  // =====================================

  const alpha =
    angle * Math.PI / 180

  const ux = Math.cos(alpha)
  const uy = Math.sin(alpha)

  // Right-hand side for the
  // no-penetration conditions.

  const rhs =
    Array(n + 1).fill(0) as number[]

  for (let i = 0; i < n; i++) {
    rhs[i] = -(
      ux * panels[i].nx +
      uy * panels[i].ny
    )
  }

  // =====================================
  // KUTTA CONDITION
  // =====================================

  const upperTE = half - 1
  const lowerTE = half

  for (let j = 0; j <= n; j++) {
    normal[n][j] =
      tangent[upperTE][j] +
      tangent[lowerTE][j]
  }

  const a = panels[upperTE]
  const b = panels[lowerTE]

  rhs[n] = -(
    ux * (a.tx + b.tx) +
    uy * (a.ty + b.ty)
  )

  // =====================================
  // SOLVE FOR PANEL STRENGTHS
  // =====================================

  const strengths =
    solveLinear(normal, rhs)

  // =====================================
  // NO-PENETRATION RESIDUAL
  // =====================================

  let normalResidual = 0

  for (let i = 0; i < n; i++) {
    let evaluated = 0

    for (let j = 0; j <= n; j++) {
      evaluated +=
        normal[i][j] *
        strengths[j]
    }

    normalResidual = Math.max(
      normalResidual,
      Math.abs(
        evaluated - rhs[i]
      )
    )
  }

  // =====================================
  // NET SOURCE FLUX
  // =====================================

  let sourceFlux = 0

  for (let i = 0; i < n; i++) {
    sourceFlux +=
      strengths[i] *
      panels[i].len
  }

  // =====================================
  // PRESSURE DISTRIBUTION
  // =====================================

  const cp: number[] = []
  const vt: number[] = []

  let fx = 0
  let fy = 0

  for (let i = 0; i < n; i++) {
    const panel = panels[i]

    let tangential =
      ux * panel.tx +
      uy * panel.ty

    for (let j = 0; j <= n; j++) {
      tangential +=
        tangent[i][j] *
        strengths[j]
    }

    vt.push(tangential)

    // Incompressible Bernoulli:
    //
    // Cp = 1 - (Vt / V∞)^2
    //
    // Here V∞ = 1.

    const pressure =
      1 - tangential ** 2

    cp.push(pressure)

    // Integrate pressure force
    // over the discretized surface.

    fx -=
      pressure *
      panel.nx *
      panel.len

    fy -=
      pressure *
      panel.ny *
      panel.len
  }

  // =====================================
  // SECTION LIFT COEFFICIENT
  // =====================================

  const cl =
    -fx * Math.sin(alpha) +
    fy * Math.cos(alpha)

  // =====================================
  // KUTTA RESIDUAL
  // =====================================

  const kuttaResidual = Math.abs(
    vt[upperTE] +
    vt[lowerTE]
  )

  // =====================================
  // RETURN SOLUTION
  // =====================================

  return {
    cp,
    vt,
    cl,
    strengths,
    normalResidual,
    sourceFlux,
    kuttaResidual,
  }
}

// =====================================
// VELOCITY AT ANY FIELD POINT
// =====================================

export function fieldVelocity(
  x: number,
  y: number,
  panels: Panel[],
  strengths: number[],
  ux: number,
  uy: number
) {
  let u = ux
  let v = uy

  const gamma =
    strengths[panels.length]

  for (let j = 0; j < panels.length; j++) {
    const source = sourceVelocity(
      x,
      y,
      panels[j]
    )

    const sigma = strengths[j]

    u +=
      sigma * source.u -
      gamma * source.v

    v +=
      sigma * source.v +
      gamma * source.u
  }

  return { u, v }
}

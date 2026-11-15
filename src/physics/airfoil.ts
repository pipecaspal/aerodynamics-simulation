
/**
 * Aerodynamics Lab
 * NACA Four-Digit Airfoil Geometry
 *
 * Generates airfoil coordinates and
 * surface panels for the numerical solver.
 */

export type Point = {
  x: number
  y: number
}

export type Panel = {
  ax: number
  ay: number
  cx: number
  cy: number
  len: number
  tx: number
  ty: number
  nx: number
  ny: number
}

export type NacaParameters = {
  code: string
  m: number
  p: number
  t: number
}

export type AirfoilGeometry = NacaParameters & {
  points: Point[]
  panels: Panel[]
  half: number
}

// =====================================
// VALIDATE AND DECODE NACA CODE
// =====================================

export function parseNaca4(
  input: string
): NacaParameters {
  const code = input.trim()

  if (!/^\d{4}$/.test(code)) {
    throw new Error(
      'Enter exactly four digits, such as 2412.'
    )
  }

  const m = Number(code[0]) / 100
  const p = Number(code[1]) / 10
  const t = Number(code.slice(2)) / 100

  if (m > 0 && p === 0) {
    throw new Error(
      'A cambered four-digit NACA airfoil ' +
      'must have a nonzero camber position.'
    )
  }

  if (m === 0 && p !== 0) {
    throw new Error(
      'For a symmetric four-digit NACA ' +
      'airfoil, use 00 as the first two digits.'
    )
  }

  if (t === 0) {
    throw new Error(
      'Airfoil thickness must be greater than zero.'
    )
  }

  return { code, m, p, t }
}

// =====================================
// MEAN CAMBER LINE
// =====================================

export function camber(
  x: number,
  m: number,
  p: number
) {
  if (m === 0 || p === 0) {
    return {
      y: 0,
      slope: 0,
    }
  }

  if (x < p) {
    return {
      y: m / (p * p) *
        (2 * p * x - x * x),

      slope: 2 * m / (p * p) *
        (p - x),
    }
  }

  return {
    y: m / ((1 - p) ** 2) *
      (1 - 2 * p + 2 * p * x - x * x),

    slope: 2 * m / ((1 - p) ** 2) *
      (p - x),
  }
}

// =====================================
// NACA THICKNESS DISTRIBUTION
// =====================================

export function thickness(
  x: number,
  t: number
) {
  // -0.1036 gives a closed trailing edge.

  return 5 * t * (
    0.2969 * Math.sqrt(x)
    - 0.1260 * x
    - 0.3516 * x ** 2
    + 0.2843 * x ** 3
    - 0.1036 * x ** 4
  )
}

// =====================================
// GENERATE AIRFOIL AND PANELS
// =====================================

export function createAirfoil(
  code: string,
  half = 40
): AirfoilGeometry {
  const parameters = parseNaca4(code)
  const { m, p, t } = parameters

  if (!Number.isInteger(half) || half < 2) {
    throw new Error(
      'The number of panels per surface ' +
      'must be an integer of at least 2.'
    )
  }

  const upper: Point[] = []
  const lower: Point[] = []

  // Cosine spacing provides more
  // resolution near the leading edge.

  for (let i = 0; i <= half; i++) {
    const x =
      (1 - Math.cos(Math.PI * i / half)) / 2

    const { y, slope } = camber(x, m, p)
    const yt = thickness(x, t)
    const theta = Math.atan(slope)

    upper.push({
      x: x - yt * Math.sin(theta),
      y: y + yt * Math.cos(theta),
    })

    lower.push({
      x: x + yt * Math.sin(theta),
      y: y - yt * Math.cos(theta),
    })
  }

  const points: Point[] = [
    ...upper,
    ...lower.slice(1, -1).reverse(),
  ]

  const panels: Panel[] = points.map(
    (a, i) => {
      const b =
        points[(i + 1) % points.length]

      const dx = b.x - a.x
      const dy = b.y - a.y

      const len = Math.hypot(dx, dy)
      const tx = dx / len
      const ty = dy / len

      return {
        ax: a.x,
        ay: a.y,
        cx: (a.x + b.x) / 2,
        cy: (a.y + b.y) / 2,
        len,
        tx,
        ty,
        nx: -ty,
        ny: tx,
      }
    }
  )

  return {
    ...parameters,
    points,
    panels,
    half,
  }
}

// =====================================
// THIN-AIRFOIL THEORY
// =====================================

export function zeroLiftAngle(
  m: number,
  p: number
) {
  if (m === 0) return 0

  const steps = 2000
  let sum = 0

  for (let i = 0; i < steps; i++) {
    const theta =
      Math.PI * (i + 0.5) / steps

    const x =
      (1 - Math.cos(theta)) / 2

    sum += camber(x, m, p).slope *
      (1 - Math.cos(theta))
  }

  return sum / steps
}

export function theoreticalCl(
  angle: number,
  m: number,
  p: number
) {
  const alpha =
    angle * Math.PI / 180

  return 2 * Math.PI *
    (alpha - zeroLiftAngle(m, p))
}

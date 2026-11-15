
import { useEffect, useMemo, useRef, useState } from 'react'

import {
  createAirfoil,
  parseNaca4,
  zeroLiftAngle,
  type Point,
  type Panel,
} from './physics/airfoil'

import {
  solvePanelMethod,
  fieldVelocity,
} from './physics/panelMethod'

// =====================================
// TYPES
// =====================================

type ConvergenceRow = {
  count: number
  cl: number
  change: number | null
  percentage: number | null
}

type LiftPoint = {
  alpha: number
  panel: number
  thin: number
}

type SweepData = {
  '0012': LiftPoint[]
  '2412': LiftPoint[]
  customCode: string | null
  custom: LiftPoint[]
}

type ValidationResult = {
  normalResidual: number
  kuttaResidual: number
  sourceFlux: number
  symmetryLift: number
  symmetryCp: number
  references: {
    alpha: number
    reference: number
    calculated: number
  }[]
}

// =====================================
// CONFIGURATION
// =====================================

const meshSizes = [40, 80, 120, 160]

const reference2412 = [
  { alpha: 1, cl: 0.3763 },
  { alpha: 2, cl: 0.4969 },
  { alpha: 3, cl: 0.6174 },
  { alpha: 4, cl: 0.7378 },
  { alpha: 5, cl: 0.8579 },
]

const referenceUrl =
  'https://www.researchgate.net/publication/' +
  '332093964_Pressure_Distribution_and_Lift_of_' +
  '4-series_NACA_Aerofoils_using_Constant_' +
  'Strength_Vortex_Panel_Method'

const cardStyle = {
  background: '#101e30',
  padding: '20px',
  borderRadius: '12px',
  marginTop: '20px',
}

const buttonStyle = {
  background: '#0284c7',
  color: 'white',
  padding: '12px 20px',
  border: 'none',
  borderRadius: '8px',
  fontSize: '15px',
  fontWeight: 600,
  cursor: 'pointer',
}

const rad = (degrees: number) =>
  degrees * Math.PI / 180

const screenX = (x: number) => 240 + 500 * x
const screenY = (y: number) => 230 - 500 * y

const pause = () =>
  new Promise<void>(resolve => {
    window.setTimeout(resolve, 12)
  })

// =====================================
// AIRFOIL COLLISION DETECTION
// =====================================

function pointInside(
  x: number,
  y: number,
  points: Point[]
) {
  let inside = false

  for (
    let i = 0, j = points.length - 1;
    i < points.length;
    j = i++
  ) {
    const a = points[i]
    const b = points[j]

    if (
      (a.y > y) !== (b.y > y) &&
      x < (b.x - a.x) * (y - a.y) /
        (b.y - a.y) + a.x
    ) {
      inside = !inside
    }
  }

  return inside
}

// =====================================
// STREAMLINE INTEGRATION
//
// Uses fieldVelocity from panelMethod.ts
// =====================================

function traceStreamline(
  seedY: number,
  points: Point[],
  panels: Panel[],
  strengths: number[],
  angle: number
) {
  let x = -0.38
  let y = seedY

  const ux = Math.cos(rad(angle))
  const uy = Math.sin(rad(angle))

  const stepSize = 0.017
  const path: Point[] = []

  for (let step = 0; step < 170; step++) {
    if (
      x > 1.43 ||
      x < -0.55 ||
      Math.abs(y) > 0.52 ||
      pointInside(x, y, points)
    ) break

    path.push({ x, y })

    const v1 = fieldVelocity(
      x,
      y,
      panels,
      strengths,
      ux,
      uy
    )

    const speed1 = Math.hypot(v1.u, v1.v)

    if (speed1 < 0.025) break

    const mx =
      x + 0.5 * stepSize * v1.u / speed1

    const my =
      y + 0.5 * stepSize * v1.v / speed1

    if (pointInside(mx, my, points)) break

    const v2 = fieldVelocity(
      mx,
      my,
      panels,
      strengths,
      ux,
      uy
    )

    const speed2 = Math.hypot(v2.u, v2.v)

    if (speed2 < 0.025) break

    const nextX =
      x + stepSize * v2.u / speed2

    const nextY =
      y + stepSize * v2.v / speed2

    if (pointInside(nextX, nextY, points)) break

    x = nextX
    y = nextY
  }

  if (path.length < 3) return ''

  return path.map((point, i) =>
    `${i === 0 ? 'M' : 'L'} ` +
    `${screenX(point.x).toFixed(2)} ` +
    `${screenY(point.y).toFixed(2)}`
  ).join(' ')
}

// =====================================
// VELOCITY HEATMAP COLORS
// =====================================

const colorStops: {
  speed: number
  rgb: [number, number, number]
}[] = [
  { speed: 0, rgb: [35, 65, 145] },
  { speed: 0.6, rgb: [45, 122, 205] },
  { speed: 0.85, rgb: [32, 183, 207] },
  { speed: 1.0, rgb: [67, 198, 116] },
  { speed: 1.25, rgb: [231, 208, 69] },
  { speed: 1.5, rgb: [248, 137, 52] },
  { speed: 2.0, rgb: [229, 58, 65] },
]

function velocityColor(
  speed: number
): [number, number, number] {
  const value = Math.max(
    0,
    Math.min(
      Number.isFinite(speed) ? speed : 2,
      2
    )
  )

  for (let i = 0; i < colorStops.length - 1; i++) {
    const a = colorStops[i]
    const b = colorStops[i + 1]

    if (value <= b.speed) {
      const t =
        (value - a.speed) /
        (b.speed - a.speed)

      return [
        Math.round(
          a.rgb[0] + (b.rgb[0] - a.rgb[0]) * t
        ),
        Math.round(
          a.rgb[1] + (b.rgb[1] - a.rgb[1]) * t
        ),
        Math.round(
          a.rgb[2] + (b.rgb[2] - a.rgb[2]) * t
        ),
      ]
    }
  }

  return [229, 58, 65]
}

// =====================================
// OPTIMIZED VELOCITY HEATMAP
//
// Uses fieldVelocity from panelMethod.ts
// =====================================

function VelocityHeatmap({
  panels,
  strengths,
  angle,
  visible,
}: {
  panels: Panel[]
  strengths: number[]
  angle: number
  visible: boolean
}) {
  const canvasRef =
    useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const canvas = canvasRef.current

    if (!canvas || !visible) return

    const ctx = canvas.getContext('2d')

    if (!ctx) return

    const width = canvas.width
    const height = canvas.height

    const image =
      ctx.createImageData(width, height)

    const pixels = image.data

    const cos = Math.cos(rad(angle))
    const sin = Math.sin(rad(angle))

    let row = 0
    let active = true
    let frameId = 0

    const renderChunk = () => {
      if (!active) return

      const endRow =
        Math.min(height, row + 16)

      for (; row < endRow; row++) {
        for (let ix = 0; ix < width; ix++) {
          const sx =
            (ix + 0.5) * 1000 / width

          const sy =
            (row + 0.5) * 460 / height

          const dx = sx - 490
          const dy = sy - 230

          const bodyX =
            490 + cos * dx + sin * dy

          const bodyY =
            230 - sin * dx + cos * dy

          const x = (bodyX - 240) / 500
          const y = (230 - bodyY) / 500

          const velocity = fieldVelocity(
            x,
            y,
            panels,
            strengths,
            cos,
            sin
          )

          const rgb = velocityColor(
            Math.hypot(
              velocity.u,
              velocity.v
            )
          )

          const index =
            (row * width + ix) * 4

          pixels[index] = rgb[0]
          pixels[index + 1] = rgb[1]
          pixels[index + 2] = rgb[2]
          pixels[index + 3] = 255
        }
      }

      if (row < height) {
        frameId =
          requestAnimationFrame(renderChunk)
      } else {
        ctx.putImageData(image, 0, 0)
      }
    }

    frameId =
      requestAnimationFrame(renderChunk)

    return () => {
      active = false
      cancelAnimationFrame(frameId)
    }
  }, [panels, strengths, angle, visible])

  return (
    <canvas
      ref={canvasRef}
      width={160}
      height={74}
      aria-label="Velocity magnitude heatmap"
      style={{
        position: 'absolute',
        inset: 0,
        width: '100%',
        height: '100%',
        pointerEvents: 'none',
        opacity: visible ? 0.88 : 0,
        imageRendering: 'auto',
        filter: 'blur(2.5px)',
      }}
    />
  )
}

// =====================================
// LIFT CURVE GRAPH
// =====================================

function LiftCurveGraph({
  data,
  selectedAngle,
  selectedAirfoil,
}: {
  data: SweepData
  selectedAngle: number
  selectedAirfoil: string
}) {
  if (
    data['0012'].length === 0 ||
    data['2412'].length === 0
  ) return null

  const showCustom =
    data.customCode !== null &&
    data.customCode === selectedAirfoil &&
    data.custom.length === 21

  const allValues = [
    ...data['0012'].flatMap(
      p => [p.panel, p.thin]
    ),
    ...data['2412'].flatMap(
      p => [p.panel, p.thin]
    ),
    ...reference2412.map(p => p.cl),
    ...(showCustom
      ? data.custom.flatMap(
          p => [p.panel, p.thin]
        )
      : []),
  ]

  const minValue =
    Math.min(...allValues) - 0.15

  const maxValue =
    Math.max(...allValues) + 0.15

  const left = 76
  const right = 755
  const top = 32
  const bottom = 310

  const gx = (a: number) =>
    left + (a + 10) / 20 *
      (right - left)

  const gy = (value: number) =>
    bottom - (value - minValue) /
      (maxValue - minValue) *
      (bottom - top)

  const makePath = (
    rows: LiftPoint[],
    key: 'panel' | 'thin'
  ) => rows.map((p, i) =>
    `${i === 0 ? 'M' : 'L'} ` +
    `${gx(p.alpha)} ${gy(p[key])}`
  ).join(' ')

  const ticks = Array.from(
    { length: 7 },
    (_, i) =>
      minValue +
      i * (maxValue - minValue) / 6
  )

  return (
    <svg
      viewBox="0 0 820 370"
      role="img"
      aria-label="Airfoil lift curve comparison"
      style={{
        width: '100%',
        display: 'block',
      }}
    >
      {ticks.map((value, i) => (
        <g key={`y-${i}`}>
          <line
            x1={left}
            x2={right}
            y1={gy(value)}
            y2={gy(value)}
            stroke="#334155"
          />

          <text
            x="65"
            y={gy(value) + 5}
            textAnchor="end"
            fontSize="14"
            fill="#cbd5e1"
          >
            {value.toFixed(2)}
          </text>
        </g>
      ))}

      {[-10, -5, 0, 5, 10].map(a => (
        <g key={`x-${a}`}>
          <line
            x1={gx(a)}
            x2={gx(a)}
            y1={top}
            y2={bottom}
            stroke="#334155"
          />

          <text
            x={gx(a)}
            y="335"
            textAnchor="middle"
            fontSize="15"
            fill="#cbd5e1"
          >
            {a}°
          </text>
        </g>
      ))}

      <line
        x1={left}
        x2={right}
        y1={gy(0)}
        y2={gy(0)}
        stroke="#94a3b8"
        strokeWidth="1.5"
      />

      {/* NACA 0012 */}

      <path
        d={makePath(data['0012'], 'thin')}
        stroke="#7dd3fc"
        strokeWidth="2"
        strokeDasharray="8 7"
        fill="none"
      />

      <path
        d={makePath(data['0012'], 'panel')}
        stroke="#38bdf8"
        strokeWidth="3.5"
        fill="none"
      />

      {/* NACA 2412 */}

      <path
        d={makePath(data['2412'], 'thin')}
        stroke="#fdba74"
        strokeWidth="2"
        strokeDasharray="8 7"
        fill="none"
      />

      <path
        d={makePath(data['2412'], 'panel')}
        stroke="#fb923c"
        strokeWidth="3.5"
        fill="none"
      />

      {/* CUSTOM AIRFOIL */}

      {showCustom && (
        <>
          <path
            d={makePath(data.custom, 'thin')}
            stroke="#86efac"
            strokeWidth="2"
            strokeDasharray="8 7"
            fill="none"
          />

          <path
            d={makePath(data.custom, 'panel')}
            stroke="#22c55e"
            strokeWidth="3.5"
            fill="none"
          />
        </>
      )}

      {/* XFOIL REFERENCE POINTS */}

      {reference2412.map(p => {
        const x = gx(p.alpha)
        const y = gy(p.cl)

        return (
          <path
            key={p.alpha}
            d={
              `M ${x} ${y - 6} ` +
              `L ${x + 6} ${y} ` +
              `L ${x} ${y + 6} ` +
              `L ${x - 6} ${y} Z`
            }
            fill="#c084fc"
            stroke="white"
          />
        )
      })}

      {/* CURRENT ANGLE */}

      {selectedAngle >= -10 &&
        selectedAngle <= 10 && (
        <line
          x1={gx(selectedAngle)}
          x2={gx(selectedAngle)}
          y1={top}
          y2={bottom}
          stroke="#fbbf24"
          strokeWidth="2"
          strokeDasharray="5 5"
        />
      )}

      <text
        x="410"
        y="363"
        textAnchor="middle"
        fontSize="17"
        fill="white"
      >
        Angle of Attack (degrees)
      </text>

      <text
        x="16"
        y="20"
        fontSize="16"
        fill="white"
      >
        cl
      </text>
    </svg>
  )
}

// =====================================
// CONVERGENCE GRAPH
// =====================================

function ConvergenceGraph({
  rows,
}: {
  rows: ConvergenceRow[]
}) {
  if (!rows.length) return null

  const values = rows.map(r => r.cl)

  const min = Math.min(...values)
  const max = Math.max(...values)

  const padding = Math.max(
    0.001,
    (max - min) * 0.25
  )

  const low = min - padding
  const high = max + padding

  const gx = (n: number) =>
    75 + (n - 40) / 120 * 640

  const gy = (value: number) =>
    250 - (value - low) /
      (high - low) * 200

  return (
    <svg
      viewBox="0 0 800 315"
      role="img"
      aria-label="Panel convergence graph"
      style={{ width: '100%' }}
    >
      {[0, 1, 2, 3, 4].map(i => {
        const value =
          low + i * (high - low) / 4

        return (
          <g key={i}>
            <line
              x1="75"
              x2="715"
              y1={gy(value)}
              y2={gy(value)}
              stroke="#334155"
            />

            <text
              x="65"
              y={gy(value) + 5}
              fontSize="13"
              fill="#cbd5e1"
              textAnchor="end"
            >
              {value.toFixed(4)}
            </text>
          </g>
        )
      })}

      <polyline
        points={rows.map(r =>
          `${gx(r.count)},${gy(r.cl)}`
        ).join(' ')}
        stroke="#38bdf8"
        strokeWidth="3"
        fill="none"
      />

      {rows.map(r => (
        <g key={r.count}>
          <circle
            cx={gx(r.count)}
            cy={gy(r.cl)}
            r="6"
            fill="#fbbf24"
          />

          <text
            x={gx(r.count)}
            y="280"
            fill="white"
            fontSize="14"
            textAnchor="middle"
          >
            {r.count}
          </text>
        </g>
      ))}

      <text
        x="400"
        y="309"
        fill="white"
        textAnchor="middle"
        fontSize="15"
      >
        Number of Panels
      </text>
    </svg>
  )
}

// =====================================
// MAIN APPLICATION
// =====================================

function App() {
  // =====================================
  // CUSTOM NACA AIRFOIL
  // =====================================

  const [airfoil, setAirfoil] =
    useState('0012')

  const [draftCode, setDraftCode] =
    useState('0012')

  const [inputError, setInputError] =
    useState('')

  // =====================================
  // ANGLE OF ATTACK
  // =====================================

  const [dragAngle, setDragAngle] =
    useState(0)

  const [angle, setAngle] =
    useState(0)

  // =====================================
  // VISUALIZATION CONTROLS
  // =====================================

  const [showHeatmap, setShowHeatmap] =
    useState(true)

  const [showFlow, setShowFlow] =
    useState(true)

  const [animateFlow, setAnimateFlow] =
    useState(true)

  // =====================================
  // NUMERICAL ANALYSIS STATE
  // =====================================

  const [busy, setBusy] = useState(false)
  const [progress, setProgress] = useState('')
  const [error, setError] = useState('')

  const [convergence, setConvergence] =
    useState<ConvergenceRow[]>([])

  const [convergenceCase, setConvergenceCase] =
    useState('')

  const [validation, setValidation] =
    useState<ValidationResult | null>(null)

  const [validationCase, setValidationCase] =
    useState('')

  const [sweep, setSweep] =
    useState<SweepData>({
      '0012': [],
      '2412': [],
      customCode: null,
      custom: [],
    })

  const [sweepComplete, setSweepComplete] =
    useState(false)

  // =====================================
  // CUSTOM AIRFOIL CONTROLS
  // =====================================

  function selectPreset(code: string) {
    if (busy) return

    setAirfoil(code)
    setDraftCode(code)
    setInputError('')
  }

  function applyCustomAirfoil() {
    if (busy) return

    try {
      const parsed = parseNaca4(draftCode)

      const candidate =
        createAirfoil(parsed.code, 40)

      if (
        candidate.panels.length !== 80 ||
        candidate.panels.some(
          p =>
            !Number.isFinite(p.len) ||
            p.len <= 0
        )
      ) {
        throw new Error(
          'This airfoil produced invalid panels.'
        )
      }

      setAirfoil(parsed.code)
      setDraftCode(parsed.code)
      setInputError('')
    } catch (err) {
      setInputError(
        err instanceof Error
          ? err.message
          : 'Invalid NACA code.'
      )
    }
  }

  // =====================================
  // RESPONSIVE ANGLE SLIDER
  // =====================================

  useEffect(() => {
    if (dragAngle === angle) return

    const timer = window.setTimeout(() => {
      setAngle(dragAngle)
    }, 160)

    return () =>
      window.clearTimeout(timer)
  }, [dragAngle, angle])

  const updating = dragAngle !== angle

  const caseKey =
    `${airfoil}-${angle}`

  // =====================================
  // LIVE AIRFOIL GEOMETRY
  //
  // From physics/airfoil.ts
  // =====================================

  const geometry = useMemo(
    () => createAirfoil(airfoil, 40),
    [airfoil]
  )

  // =====================================
  // LIVE PANEL SOLVER
  //
  // From physics/panelMethod.ts
  // =====================================

  const result = useMemo(
    () => solvePanelMethod(
      geometry.panels,
      geometry.half,
      angle
    ),
    [geometry, angle]
  )

  // =====================================
  // STREAMLINES
  // =====================================

  const streamlines = useMemo(
    () => Array.from(
      { length: 13 },
      (_, i) => traceStreamline(
        (i - 6) * 0.058,
        geometry.points,
        geometry.panels,
        result.strengths,
        angle
      )
    ).filter(Boolean),
    [geometry, result, angle]
  )

  // =====================================
  // LIFT COEFFICIENTS
  // =====================================

  const zeroLift = useMemo(
    () => zeroLiftAngle(
      geometry.m,
      geometry.p
    ),
    [geometry.m, geometry.p]
  )

  const thinCl =
    2 * Math.PI *
    (rad(angle) - zeroLift)

  const panelCl =
    Math.abs(result.cl) < 0.00005
      ? 0
      : result.cl

  // =====================================
  // VELOCITY PROBES
  // =====================================

  const ux = Math.cos(rad(angle))
  const uy = Math.sin(rad(angle))

  const upperProbe = fieldVelocity(
    0.4,
    0.14,
    geometry.panels,
    result.strengths,
    ux,
    uy
  )

  const lowerProbe = fieldVelocity(
    0.4,
    -0.14,
    geometry.panels,
    result.strengths,
    ux,
    uy
  )

  const upperSpeed =
    Math.hypot(upperProbe.u, upperProbe.v)

  const lowerSpeed =
    Math.hypot(lowerProbe.u, lowerProbe.v)

  // =====================================
  // AIRFOIL SVG PATH
  // =====================================

  const airfoilPath =
    geometry.points.map((p, i) =>
      `${i === 0 ? 'M' : 'L'} ` +
      `${screenX(p.x)} ${screenY(p.y)}`
    ).join(' ') + ' Z'

  // =====================================
  // PRESSURE DISTRIBUTION GRAPH
  // =====================================

  const left = 80
  const right = 750
  const top = 40
  const bottom = 280

  const minCp = Math.floor(
    (Math.min(...result.cp) - 0.2) * 2
  ) / 2

  const maxCp = Math.ceil(
    (Math.max(...result.cp) + 0.2) * 2
  ) / 2

  const gx = (x: number) =>
    left + x * (right - left)

  const gy = (cp: number) =>
    top + (cp - minCp) /
      (maxCp - minCp) *
      (bottom - top)

  const pressurePath = (
    start: number,
    end: number
  ) => {
    const commands: string[] = []

    for (let i = start; i < end; i++) {
      const panel =
        geometry.panels[i]

      commands.push(
        `${commands.length === 0 ? 'M' : 'L'} ` +
        `${gx(panel.cx)} ` +
        `${gy(result.cp[i])}`
      )
    }

    return commands.join(' ')
  }

  const upperPressure =
    pressurePath(0, geometry.half)

  const lowerPressure =
    pressurePath(
      geometry.half,
      geometry.panels.length
    )

  const cpTicks = Array.from(
    { length: 6 },
    (_, i) =>
      minCp +
      i * (maxCp - minCp) / 5
  )

  // =====================================
  // FULL LIFT CURVE ANALYSIS
  //
  // Uses imported panel solver.
  // =====================================

  async function runLiftSweep() {
    if (busy || updating) return

    setBusy(true)
    setError('')
    setSweepComplete(false)

    const customCode =
      airfoil !== '0012' &&
      airfoil !== '2412'
        ? airfoil
        : null

    const data: SweepData = {
      '0012': [],
      '2412': [],
      customCode,
      custom: [],
    }

    setSweep({ ...data })

    try {
      const codes: string[] = [
        '0012',
        '2412',
        ...(customCode ? [customCode] : []),
      ]

      for (const code of codes) {
        const mesh =
          createAirfoil(code, 40)

        const zero =
          zeroLiftAngle(mesh.m, mesh.p)

        const rows: LiftPoint[] = []

        for (let a = -10; a <= 10; a++) {
          setProgress(
            `NACA ${code}: solving ${a}°...`
          )

          await pause()

          const solution = solvePanelMethod(
            mesh.panels,
            mesh.half,
            a
          )

          const thin =
            2 * Math.PI *
            (rad(a) - zero)

          rows.push({
            alpha: a,
            panel: solution.cl,
            thin,
          })
        }

        if (code === '0012') {
          data['0012'] = rows
        } else if (code === '2412') {
          data['2412'] = rows
        } else {
          data.custom = rows
        }

        setSweep({
          '0012': [...data['0012']],
          '2412': [...data['2412']],
          customCode: data.customCode,
          custom: [...data.custom],
        })
      }

      setSweepComplete(true)
    } catch (err) {
      setError(
        err instanceof Error
          ? err.message
          : 'Lift sweep failed'
      )
    } finally {
      setBusy(false)
      setProgress('')
    }
  }

  // =====================================
  // NUMERICAL CONVERGENCE TEST
  //
  // Uses imported panel solver.
  // =====================================

  async function runConvergence() {
    if (busy || updating) return

    setBusy(true)
    setError('')
    setConvergence([])
    setConvergenceCase(caseKey)

    const rows: ConvergenceRow[] = []

    try {
      for (const count of meshSizes) {
        setProgress(
          `Solving ${count} panels...`
        )

        await pause()

        const mesh =
          createAirfoil(
            airfoil,
            count / 2
          )

        const solution =
          solvePanelMethod(
            mesh.panels,
            mesh.half,
            angle
          )

        const previous =
          rows[rows.length - 1]

        const change = previous
          ? Math.abs(
              solution.cl - previous.cl
            )
          : null

        const scale = previous
          ? Math.max(
              Math.abs(solution.cl),
              Math.abs(previous.cl)
            )
          : 0

        rows.push({
          count,
          cl: solution.cl,
          change,
          percentage:
            change !== null &&
            scale >= 0.001
              ? change / scale * 100
              : null,
        })

        setConvergence([...rows])
      }
    } catch (err) {
      setError(
        err instanceof Error
          ? err.message
          : 'Convergence test failed'
      )
    } finally {
      setBusy(false)
      setProgress('')
    }
  }

  // =====================================
  // VALIDATION SUITE
  //
  // Uses imported panel solver.
  // =====================================

  async function runValidation() {
    if (busy || updating) return

    setBusy(true)
    setError('')
    setValidation(null)
    setValidationCase(caseKey)

    try {
      setProgress(
        'Checking boundary conditions...'
      )

      await pause()

      const mesh =
        createAirfoil(airfoil, 80)

      const solution =
        solvePanelMethod(
          mesh.panels,
          mesh.half,
          angle
        )

      setProgress(
        'Checking NACA 0012 symmetry...'
      )

      await pause()

      const symmetricMesh =
        createAirfoil('0012', 80)

      const symmetric =
        solvePanelMethod(
          symmetricMesh.panels,
          symmetricMesh.half,
          0
        )

      let symmetryCp = 0

      for (
        let i = 0;
        i < symmetric.cp.length / 2;
        i++
      ) {
        symmetryCp = Math.max(
          symmetryCp,
          Math.abs(
            symmetric.cp[i] -
            symmetric.cp[
              symmetric.cp.length - 1 - i
            ]
          )
        )
      }

      const referenceMesh =
        createAirfoil('2412', 80)

      const references = []

      for (const reference of reference2412) {
        setProgress(
          `Comparing NACA 2412 at ` +
          `${reference.alpha}°...`
        )

        await pause()

        const calculated =
          solvePanelMethod(
            referenceMesh.panels,
            referenceMesh.half,
            reference.alpha
          ).cl

        references.push({
          alpha: reference.alpha,
          reference: reference.cl,
          calculated,
        })
      }

      setValidation({
        normalResidual:
          solution.normalResidual,
        kuttaResidual:
          solution.kuttaResidual,
        sourceFlux:
          solution.sourceFlux,
        symmetryLift:
          Math.abs(symmetric.cl),
        symmetryCp,
        references,
      })
    } catch (err) {
      setError(
        err instanceof Error
          ? err.message
          : 'Validation failed'
      )
    } finally {
      setBusy(false)
      setProgress('')
    }
  }

  // =====================================
  // DISPLAYED ANALYSIS RESULTS
  // =====================================

  const visibleConvergence =
    convergenceCase === caseKey
      ? convergence
      : []

  const visibleValidation =
    validationCase === caseKey
      ? validation
      : null

  const completedSweep =
    sweepComplete &&
    sweep['0012'].length === 21 &&
    sweep['2412'].length === 21

  const selected0012 =
    completedSweep
      ? sweep['0012'].find(
          p => p.alpha === angle
        )
      : undefined

  const selected2412 =
    completedSweep
      ? sweep['2412'].find(
          p => p.alpha === angle
        )
      : undefined

  const selectedCustom =
    completedSweep &&
    sweep.customCode === airfoil
      ? sweep.custom.find(
          p => p.alpha === angle
        )
      : undefined

  const isCustom =
    airfoil !== '0012' &&
    airfoil !== '2412'

  // =====================================
  // USER INTERFACE
  // =====================================

  return (
    <main style={{
      maxWidth: '1100px',
      margin: '0 auto',
    }}>
      <h1>Aerodynamics Lab</h1>
      <h2>2D Airfoil Flow Simulator</h2>

      {/* ===================================
          CUSTOM NACA AIRFOIL DESIGNER
          =================================== */}

      <section style={cardStyle}>
        <h2>Custom NACA Airfoil Designer</h2>

        <p style={{
          color: '#cbd5e1',
        }}>
          Select a preset airfoil or enter
          your own four-digit NACA designation.
        </p>

        <h3>Preset Airfoils</h3>

        <select
          value={airfoil}
          disabled={busy}
          onChange={e =>
            selectPreset(e.target.value)
          }
          style={{
            padding: '12px',
            fontSize: '16px',
            background: '#1e293b',
            color: 'white',
            borderRadius: '8px',
            maxWidth: '100%',
          }}
        >
          <option value="0012">
            NACA 0012 — Symmetrical
          </option>

          <option value="2412">
            NACA 2412 — Cambered
          </option>

          {isCustom && (
            <option value={airfoil}>
              NACA {airfoil} — Custom
            </option>
          )}
        </select>

        <h3 style={{ marginTop: '24px' }}>
          Enter Custom NACA Code
        </h3>

        <form
          onSubmit={e => {
            e.preventDefault()
            applyCustomAirfoil()
          }}
          style={{
            display: 'flex',
            justifyContent: 'center',
            gap: '12px',
            flexWrap: 'wrap',
          }}
        >
          <input
            type="text"
            inputMode="numeric"
            maxLength={4}
            value={draftCode}
            disabled={busy}
            placeholder="e.g. 4412"
            aria-label="Four-digit NACA code"
            onChange={e => {
              setDraftCode(e.target.value)
              setInputError('')
            }}
            style={{
              width: '130px',
              padding: '12px',
              fontSize: '20px',
              letterSpacing: '3px',
              textAlign: 'center',
              background: '#1e293b',
              border: '1px solid #475569',
              borderRadius: '8px',
              color: 'white',
            }}
          />

          <button
            type="submit"
            disabled={busy}
            style={buttonStyle}
          >
            Apply Airfoil
          </button>
        </form>

        {inputError && (
          <p style={{
            color: '#f87171',
            marginTop: '12px',
          }}>
            {inputError}
          </p>
        )}

        <p style={{
          color: '#94a3b8',
          fontSize: '14px',
        }}>
          Try 4412, 0015, 2312, or 4415.
          The code determines camber,
          camber position, and thickness.
        </p>

        <div style={{
          background: '#172b3b',
          borderRadius: '10px',
          padding: '16px',
          marginTop: '20px',
        }}>
          <h3>
            Current Airfoil: NACA {airfoil}
          </h3>

          <div style={{
            display: 'grid',
            gridTemplateColumns:
              'repeat(auto-fit, minmax(150px, 1fr))',
            gap: '12px',
          }}>
            <div>
              <p style={{ color: '#94a3b8' }}>
                Maximum Camber
              </p>
              <h3>
                {(geometry.m * 100).toFixed(1)}%
              </h3>
            </div>

            <div>
              <p style={{ color: '#94a3b8' }}>
                Camber Position
              </p>
              <h3>
                {geometry.m === 0
                  ? 'Symmetrical'
                  : `${(geometry.p * 100).toFixed(0)}% chord`}
              </h3>
            </div>

            <div>
              <p style={{ color: '#94a3b8' }}>
                Thickness
              </p>
              <h3>
                {(geometry.t * 100).toFixed(1)}%
              </h3>
            </div>

            <div>
              <p style={{ color: '#94a3b8' }}>
                Live Solver
              </p>
              <h3>
                {geometry.panels.length} panels
              </h3>
            </div>
          </div>
        </div>
      </section>

      {/* ===================================
          AIRFLOW VISUALIZATION
          =================================== */}

      <section style={cardStyle}>
        <h2>Airflow Velocity Visualization</h2>

        <p style={{ color: '#94a3b8' }}>
          NACA {airfoil} |
          {' '}Calculated angle: {angle}°
        </p>

        <div style={{
          position: 'relative',
          width: '100%',
          aspectRatio: '1000 / 460',
          overflow: 'hidden',
          borderRadius: '10px',
          background: '#081827',
        }}>
          <VelocityHeatmap
            panels={geometry.panels}
            strengths={result.strengths}
            angle={angle}
            visible={showHeatmap}
          />

          <svg
            viewBox="0 0 1000 460"
            role="img"
            aria-label={
              `Airflow around NACA ${airfoil}`
            }
            style={{
              position: 'absolute',
              inset: 0,
              width: '100%',
              height: '100%',
            }}
          >
            <defs>
              <marker
                id="airArrow"
                markerWidth="7"
                markerHeight="7"
                refX="6"
                refY="3"
                orient="auto"
              >
                <path
                  d="M0 0 L6 3 L0 6"
                  fill="none"
                  stroke="white"
                  strokeWidth="1.5"
                />
              </marker>
            </defs>

            <line
              x1="40"
              y1="35"
              x2="165"
              y2="35"
              stroke="white"
              strokeWidth="2"
              markerEnd="url(#airArrow)"
            />

            <text
              x="40"
              y="23"
              fill="white"
              fontSize="16"
            >
              Freestream
            </text>

            <g
              transform={
                `rotate(${angle} 490 230)`
              }
            >
              {showFlow &&
                streamlines.map((d, i) => (
                <g key={i}>
                  <path
                    d={d}
                    stroke="#b9f1ff"
                    strokeWidth="2"
                    strokeOpacity="0.82"
                    fill="none"
                  />

                  {animateFlow && (
                    <circle
                      r="3.5"
                      fill="white"
                    >
                      <animateMotion
                        path={d}
                        dur="7s"
                        begin={`${-i * 0.45}s`}
                        repeatCount="indefinite"
                      />
                    </circle>
                  )}
                </g>
              ))}

              <path
                d={airfoilPath}
                fill="#77c4ed"
                stroke="white"
                strokeWidth="2.5"
              />

              <circle
                cx={screenX(0.4)}
                cy={screenY(0.14)}
                r="6"
                fill="#22d3ee"
                stroke="white"
              />

              <circle
                cx={screenX(0.4)}
                cy={screenY(-0.14)}
                r="6"
                fill="#fb923c"
                stroke="white"
              />
            </g>
          </svg>
        </div>

        {/* VELOCITY LEGEND */}

        {showHeatmap && (
          <div style={{ marginTop: '18px' }}>
            <p style={{
              color: '#cbd5e1',
            }}>
              Normalized Velocity (V / V∞)
            </p>

            <div style={{
              height: '16px',
              borderRadius: '6px',
              background:
                'linear-gradient(to right, ' +
                '#234191, #2d7acd, ' +
                '#20b7cf, #43c674, ' +
                '#e7d045, #f88934, #e53a41)',
            }} />

            <div style={{
              display: 'flex',
              justifyContent: 'space-between',
              marginTop: '6px',
              color: '#cbd5e1',
              fontSize: '13px',
            }}>
              <span>0.0</span>
              <span>0.5</span>
              <span>1.0</span>
              <span>1.5</span>
              <span>≥2.0</span>
            </div>
          </div>
        )}

        {/* ANGLE SLIDER */}

        <h3>
          Angle of Attack: {dragAngle}°
        </h3>

        {updating && (
          <p style={{
            color: '#fbbf24',
          }}>
            Previewing angle —
            updating simulation...
          </p>
        )}

        <input
          type="range"
          min="-15"
          max="20"
          step="1"
          value={dragAngle}
          disabled={busy}
          aria-label="Angle of attack"
          onChange={e =>
            setDragAngle(
              Number(e.target.value)
            )
          }
          onPointerUp={e =>
            setAngle(
              Number(e.currentTarget.value)
            )
          }
          onKeyUp={e =>
            setAngle(
              Number(e.currentTarget.value)
            )
          }
          onBlur={e =>
            setAngle(
              Number(e.currentTarget.value)
            )
          }
          style={{
            width: '80%',
            maxWidth: '450px',
          }}
        />

        <div style={{
          display: 'flex',
          justifyContent: 'center',
          gap: '18px',
          flexWrap: 'wrap',
          marginTop: '15px',
        }}>
          <label>
            <input
              type="checkbox"
              checked={showHeatmap}
              onChange={e =>
                setShowHeatmap(
                  e.target.checked
                )
              }
            />
            {' '}Velocity Heatmap
          </label>

          <label>
            <input
              type="checkbox"
              checked={showFlow}
              onChange={e =>
                setShowFlow(
                  e.target.checked
                )
              }
            />
            {' '}Streamlines
          </label>

          <label>
            <input
              type="checkbox"
              checked={animateFlow}
              onChange={e =>
                setAnimateFlow(
                  e.target.checked
                )
              }
            />
            {' '}Animate Flow
          </label>
        </div>
      </section>

      {/* ===================================
          LIVE VELOCITY AND LIFT RESULTS
          =================================== */}

      <div style={{
        display: 'grid',
        gridTemplateColumns:
          'repeat(auto-fit, minmax(220px, 1fr))',
        gap: '15px',
        marginTop: '20px',
      }}>
        {[
          {
            label: 'Upper Flow Speed',
            value: upperSpeed.toFixed(3),
            color: '#22d3ee',
            note:
              'V/V∞ at x/c = 0.4, y/c = +0.14',
          },
          {
            label: 'Lower Flow Speed',
            value: lowerSpeed.toFixed(3),
            color: '#fb923c',
            note:
              'V/V∞ at x/c = 0.4, y/c = −0.14',
          },
          {
            label: 'Panel Method Lift',
            value: panelCl.toFixed(4),
            color: '#77c4ed',
            note:
              'Pressure-integrated section cl',
          },
          {
            label: 'Thin-Airfoil Theory',
            value: thinCl.toFixed(4),
            color: '#fbbf24',
            note:
              'Ideal theoretical section cl',
          },
        ].map(item => (
          <section
            key={item.label}
            style={{
              ...cardStyle,
              marginTop: 0,
            }}
          >
            <h3>{item.label}</h3>

            <h1 style={{
              color: item.color,
            }}>
              {item.value}
            </h1>

            <p>{item.note}</p>
          </section>
        ))}
      </div>

      {/* ===================================
          PRESSURE DISTRIBUTION
          =================================== */}

      <section style={cardStyle}>
        <h2>Surface Pressure Distribution</h2>

        <p>
          NACA {airfoil} | α = {angle}°
        </p>

        <svg
          viewBox="0 0 800 340"
          role="img"
          aria-label="Pressure distribution graph"
          style={{ width: '100%' }}
        >
          {cpTicks.map((value, i) => (
            <g key={i}>
              <line
                x1={left}
                x2={right}
                y1={gy(value)}
                y2={gy(value)}
                stroke="#334155"
              />

              <text
                x="65"
                y={gy(value) + 5}
                textAnchor="end"
                fill="#cbd5e1"
                fontSize="14"
              >
                {value.toFixed(1)}
              </text>
            </g>
          ))}

          {[
            0, 0.2, 0.4,
            0.6, 0.8, 1,
          ].map(x => (
            <g key={x}>
              <line
                x1={gx(x)}
                x2={gx(x)}
                y1={top}
                y2={bottom}
                stroke="#334155"
              />

              <text
                x={gx(x)}
                y="305"
                textAnchor="middle"
                fill="#cbd5e1"
                fontSize="14"
              >
                {x.toFixed(1)}
              </text>
            </g>
          ))}

          <path
            d={upperPressure}
            fill="none"
            stroke="#38bdf8"
            strokeWidth="3"
          />

          <path
            d={lowerPressure}
            fill="none"
            stroke="#fb923c"
            strokeWidth="3"
          />

          <text
            x="18"
            y="24"
            fill="white"
          >
            Cp
          </text>

          <text
            x="415"
            y="331"
            fill="white"
            textAnchor="middle"
          >
            Chord Position (x/c)
          </text>
        </svg>

        <p>
          <span style={{
            color: '#38bdf8',
          }}>
            ● Upper Surface
          </span>

          {'　'}

          <span style={{
            color: '#fb923c',
          }}>
            ● Lower Surface
          </span>
        </p>

        <p style={{
          color: '#94a3b8',
          fontSize: '14px',
        }}>
          Negative Cp values are
          plotted higher.
        </p>
      </section>

      {/* ===================================
          COMPLETE LIFT CURVE ANALYSIS
          =================================== */}

      <section style={cardStyle}>
        <h2>
          Complete Aerodynamic Lift Curve Analysis
        </h2>

        <p style={{
          color: '#cbd5e1',
        }}>
          Compare NACA 0012 and NACA 2412
          over −10° to +10°. When a custom
          airfoil is selected, its lift curve
          is added to the comparison.
        </p>

        <p>
          80 panels per airfoil |
          {' '}1° increments |
          {' '}{isCustom ? 63 : 42}
          {' '}numerical solutions
        </p>

        <button
          type="button"
          style={buttonStyle}
          disabled={busy || updating}
          onClick={runLiftSweep}
        >
          Run Lift Curve Analysis
        </button>

        {completedSweep && (
          <>
            <h3>
              Lift Coefficient vs.
              Angle of Attack
            </h3>

            <LiftCurveGraph
              data={sweep}
              selectedAngle={angle}
              selectedAirfoil={airfoil}
            />

            <div style={{
              display: 'flex',
              flexWrap: 'wrap',
              gap: '12px 24px',
              marginTop: '14px',
              fontSize: '14px',
            }}>
              <span style={{
                color: '#38bdf8',
              }}>
                ━ NACA 0012 Panel
              </span>

              <span style={{
                color: '#7dd3fc',
              }}>
                ┄ NACA 0012 Theory
              </span>

              <span style={{
                color: '#fb923c',
              }}>
                ━ NACA 2412 Panel
              </span>

              <span style={{
                color: '#fdba74',
              }}>
                ┄ NACA 2412 Theory
              </span>

              {sweep.customCode === airfoil &&
                isCustom && (
                <>
                  <span style={{
                    color: '#22c55e',
                  }}>
                    ━ NACA {airfoil} Panel
                  </span>

                  <span style={{
                    color: '#86efac',
                  }}>
                    ┄ NACA {airfoil} Theory
                  </span>
                </>
              )}

              <span style={{
                color: '#c084fc',
              }}>
                ◆ NACA 2412 XFOIL
              </span>

              <span style={{
                color: '#fbbf24',
              }}>
                ┆ Selected Angle
              </span>
            </div>

            {selected0012 && selected2412 && (
              <div style={{
                marginTop: '20px',
                background: '#172b3b',
                borderRadius: '10px',
                padding: '16px',
              }}>
                <h3>
                  Comparison at {angle}°
                </h3>

                <div style={{
                  overflowX: 'auto',
                }}>
                  <table style={{
                    width: '100%',
                    textAlign: 'right',
                  }}>
                    <thead>
                      <tr>
                        <th>Airfoil</th>
                        <th>Panel cl</th>
                        <th>Theory cl</th>
                      </tr>
                    </thead>

                    <tbody>
                      {[
                        {
                          code: '0012',
                          point: selected0012,
                        },
                        {
                          code: '2412',
                          point: selected2412,
                        },
                        ...(selectedCustom
                          ? [{
                              code: airfoil,
                              point: selectedCustom,
                            }]
                          : []),
                      ].map(item => (
                        <tr key={item.code}>
                          <td>
                            NACA {item.code}
                          </td>

                          <td>
                            {item.point.panel.toFixed(4)}
                          </td>

                          <td>
                            {item.point.thin.toFixed(4)}
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              </div>
            )}

            <p style={{
              color: '#94a3b8',
              fontSize: '14px',
            }}>
              XFOIL reference points apply
              only to NACA 2412 at 1°–5°.
              They are not validation data
              for the custom airfoil.
            </p>

            <p style={{
              color: '#fbbf24',
              fontSize: '14px',
            }}>
              Idealized potential-flow models
              do not predict stall or
              boundary-layer separation.
            </p>
          </>
        )}
      </section>

      {/* ===================================
          NUMERICAL CONVERGENCE TEST
          =================================== */}

      <section style={cardStyle}>
        <h2>
          Numerical Solver Convergence Test
        </h2>

        <p>
          Compare 40, 80, 120, and 160
          panels for NACA {airfoil}
          {' '}at {angle}°.
        </p>

        <button
          type="button"
          style={buttonStyle}
          disabled={busy || updating}
          onClick={runConvergence}
        >
          Run Convergence Test
        </button>

        {visibleConvergence.length > 0 && (
          <>
            <div style={{
              overflowX: 'auto',
              marginTop: '20px',
            }}>
              <table style={{
                width: '100%',
                textAlign: 'right',
              }}>
                <thead>
                  <tr>
                    <th>Panels</th>
                    <th>cl</th>
                    <th>Absolute Δcl</th>
                    <th>Change</th>
                  </tr>
                </thead>

                <tbody>
                  {visibleConvergence.map(r => (
                    <tr key={r.count}>
                      <td>{r.count}</td>

                      <td>
                        {r.cl.toFixed(6)}
                      </td>

                      <td>
                        {r.change === null
                          ? '—'
                          : r.change.toExponential(3)}
                      </td>

                      <td>
                        {r.percentage === null
                          ? '—'
                          : `${r.percentage.toFixed(3)}%`}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>

            <ConvergenceGraph
              rows={visibleConvergence}
            />

            <p style={{
              color: '#94a3b8',
              fontSize: '14px',
            }}>
              Numerical stabilization is
              not the same as independent
              physical validation.
            </p>
          </>
        )}
      </section>

      {/* ===================================
          AERODYNAMIC VALIDATION SUITE
          =================================== */}

      <section style={cardStyle}>
        <h2>
          Aerodynamic Solver Validation
        </h2>

        <p>
          Check the currently selected
          airfoil's boundary-condition
          residuals, plus reference checks
          for NACA 0012 and NACA 2412.
        </p>

        <button
          type="button"
          style={buttonStyle}
          disabled={busy || updating}
          onClick={runValidation}
        >
          Run Validation Suite
        </button>

        {visibleValidation && (
          <>
            <h3>
              Boundary-Condition Diagnostics
            </h3>

            <p>
              Selected: NACA {airfoil},
              {' '}α = {angle}°,
              {' '}160 panels
            </p>

            <p>
              Maximum normal-flow residual:
              {' '}
              {visibleValidation
                .normalResidual
                .toExponential(4)}
            </p>

            <p>
              Kutta residual:
              {' '}
              {visibleValidation
                .kuttaResidual
                .toExponential(4)}
            </p>

            <p>
              Integrated source flux:
              {' '}
              {visibleValidation
                .sourceFlux
                .toExponential(4)}
            </p>

            <h3>
              NACA 0012 Symmetry Test
            </h3>

            <p>
              Absolute lift at 0°:
              {' '}
              {visibleValidation
                .symmetryLift
                .toExponential(4)}
            </p>

            <p>
              Maximum Cp asymmetry:
              {' '}
              {visibleValidation
                .symmetryCp
                .toExponential(4)}
            </p>

            <h3>
              NACA 2412 XFOIL Comparison
            </h3>

            <div style={{
              overflowX: 'auto',
            }}>
              <table style={{
                width: '100%',
                textAlign: 'right',
              }}>
                <thead>
                  <tr>
                    <th>Angle</th>
                    <th>XFOIL cl</th>
                    <th>Our cl</th>
                    <th>Difference</th>
                  </tr>
                </thead>

                <tbody>
                  {visibleValidation
                    .references
                    .map(r => (
                    <tr key={r.alpha}>
                      <td>{r.alpha}°</td>

                      <td>
                        {r.reference.toFixed(4)}
                      </td>

                      <td>
                        {r.calculated.toFixed(6)}
                      </td>

                      <td>
                        {(
                          Math.abs(
                            r.calculated -
                            r.reference
                          ) /
                          Math.abs(r.reference) *
                          100
                        ).toFixed(3)}%
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>

            <p>
              <a
                href={referenceUrl}
                target="_blank"
                rel="noopener noreferrer"
                style={{
                  color: '#38bdf8',
                }}
              >
                Published reference report
              </a>
            </p>

            {isCustom && (
              <p style={{
                color: '#fbbf24',
              }}>
                Reference comparison shown
                above applies to NACA 2412,
                not to NACA {airfoil}.
                The selected custom airfoil
                receives its own numerical
                residual diagnostics.
              </p>
            )}
          </>
        )}
      </section>

      {/* ===================================
          PROGRESS AND ERRORS
          =================================== */}

      {busy && (
        <section style={cardStyle}>
          <h3>
            Numerical Analysis Running
          </h3>

          <p style={{
            color: '#fbbf24',
          }}>
            {progress}
          </p>
        </section>
      )}

      {error && (
        <section style={cardStyle}>
          <p style={{
            color: '#f87171',
          }}>
            Error: {error}
          </p>
        </section>
      )}

      {/* ===================================
          ENGINEERING LIMITATIONS
          =================================== */}

      <section style={cardStyle}>
        <h3>
          Engineering Assumptions and Limitations
        </h3>

        <p>
          Four-digit NACA airfoil geometry
          generated using mean-camber and
          thickness-distribution equations.
        </p>

        <p>
          Two-dimensional, steady,
          incompressible, inviscid
          potential flow.
        </p>

        <p>
          Constant-strength source panels
          with a uniform vortex-sheet strength.
        </p>

        <p>
          No-penetration boundary condition
          and trailing-edge Kutta condition.
        </p>

        <p>
          Pressure coefficients obtained
          using incompressible Bernoulli theory.
        </p>

        <p>
          Streamlines integrated using
          a midpoint numerical method.
        </p>

        <p>
          Geometry and numerical solvers
          are maintained in separate
          physics modules.
        </p>

        <p style={{
          color: '#fbbf24',
        }}>
          Extreme airfoil geometries may
          require additional mesh and
          solver checks. The model cannot
          predict viscous drag, separation,
          compressibility, or stall.
          Custom designs are not
          automatically validated against
          experimental measurements.
        </p>
      </section>
    </main>
  )
}

export default App

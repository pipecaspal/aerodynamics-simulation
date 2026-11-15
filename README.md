
# Aerodynamics Lab

An interactive, browser-based aerodynamic simulation tool designed to visualize and analyze two-dimensional airflow around NACA airfoils.

Built with React, TypeScript, and numerical aerodynamic methods, Aerodynamics Lab combines interactive flow visualization with engineering calculations and numerical verification tools.

## Overview

Aerodynamics Lab is an ongoing engineering project focused on making aerodynamic concepts accessible through interactive simulations.

Users can generate custom NACA four-digit airfoils, adjust the angle of attack, visualize the resulting flow field, and analyze lift coefficients and surface pressure distributions.

The simulator currently uses a two-dimensional, inviscid potential-flow panel method.

## Features

### Interactive Airfoil Simulator
- Generate custom NACA four-digit airfoils
- Preset NACA 0012 and NACA 2412 profiles
- Adjustable angle of attack
- Interactive airflow streamlines
- Animated flow particles
- Velocity magnitude heatmap
- Upper and lower velocity measurements

### Aerodynamic Analysis
- Numerical lift coefficient calculation
- Thin-airfoil theory comparison
- Surface pressure coefficient distribution
- Lift coefficient versus angle-of-attack graphs
- Comparison of multiple airfoil designs

### Numerical Verification
- Panel mesh convergence testing
- No-penetration boundary-condition residuals
- Trailing-edge Kutta-condition residuals
- Integrated source-flux diagnostics
- Symmetric-airfoil verification
- Comparison against NACA 2412 XFOIL reference values

## Numerical Methods

The simulation uses a Hess-Smith-style potential-flow panel method with:

- Constant-strength source panels
- A uniform vortex-sheet strength
- A no-penetration boundary condition
- A trailing-edge Kutta condition
- Gaussian elimination with partial pivoting
- Pressure integration to estimate section lift
- Numerical streamline integration

Airfoil geometry is generated programmatically using NACA four-digit camber and thickness equations.

## Technology Stack

- React
- TypeScript
- Vite
- SVG
- HTML Canvas
- Numerical methods implemented in TypeScript

## Project Structure

The main application interface is located in `src/App.tsx`.

The numerical engineering modules are located in `src/physics/`:

- `airfoil.ts` — NACA airfoil geometry generation
- `panelMethod.ts` — Panel solver, pressure, lift, and velocity calculations

Additional modules and simulations are planned as the project develops.

## Running Locally

Clone the repository:

```bash
git clone https://github.com/pipecaspal/aerodynamics-simulation.git
```

Navigate into the project:

```bash
cd aerodynamics-simulation
```

Install dependencies:

```bash
npm install
```

Start the development server:

```bash
npm run dev
```

Open the local address displayed in the terminal.

To create a production build:

```bash
npm run build
```

## Engineering Limitations

The current simulation assumes:

- Two-dimensional flow
- Steady flow conditions
- Incompressible flow
- Inviscid potential flow

The model does not currently predict viscous drag, boundary-layer separation, compressibility effects, or aerodynamic stall.

Numerical convergence and agreement with selected reference values do not guarantee experimental accuracy. Additional validation is required before using the results for practical engineering decisions.

## Development Roadmap

Planned improvements include:

- Further modularization of numerical and visualization code
- Real-world airspeed and atmospheric-condition inputs
- Dimensional aerodynamic force calculations
- Additional aerodynamic analysis capabilities
- Expanded numerical and experimental validation
- Further interactive engineering simulations

## Project Status

**Active development**

The source code is available on GitHub. A publicly hosted interactive demonstration is planned.

## Author

Developed by [pipecaspal](https://github.com/pipecaspal).

Mechanical engineering educational project.

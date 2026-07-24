# CMSIS-DSP Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored CMSIS-DSP source subset under `Library/CMSIS/DSP`.
This BSP vendors only the `Include`, `Source`, `ComputeLibrary`, `dsppp` and `PrivateInclude` directories from the CMSIS-DSP upstream release. Other upstream repository content is not included in `Library/CMSIS/DSP`.

## 1) Component Identity

- Component name (`name`): `CMSIS-DSP`
- Component type (`type`): `library`
- Supplier / project: `Arm Limited`
- Version: `1.16.2`
- Licenses: `Apache-2.0` and `MIT`
- Evidence path: `Library/CMSIS/DSP`
- Upstream repository: `https://github.com/ARM-software/CMSIS-DSP`
- Upstream tag: `v1.16.2`
- Upstream commit: `d5717e454fec0337bef114a21f1d2d01d74f2701`

## 2) Evidence for Version and License

- `Library/CMSIS/DSP/Include/arm_math.h` identifies the CMSIS DSP Library and contains `SPDX-License-Identifier: Apache-2.0`.
- `Library/CMSIS/DSP/ComputeLibrary/README.md` states that the included ComputeLibrary-derived files have the MIT license and that other CMSIS-DSP content is Apache-2.0.
- `Library/CMSIS/DSP/ComputeLibrary/Include/NEMath.h` contains `SPDX-License-Identifier: MIT`.

## 3) License Handling Guidance

Use both `Apache-2.0` and `MIT` in the component licenses. The MIT scope is the vendored `ComputeLibrary` content; the remaining CMSIS-DSP content is Apache-2.0.

## 4) Functional / Technical Scope

CMSIS-DSP provides DSP algorithms, vector and matrix functions, transform functions, filters, statistics, and Helium-optimized implementations for Cortex-M processors.

## 5) Suggested CycloneDX Field Mapping

- `bom-ref`: `pkg:github/arm-software/CMSIS-DSP@1.16.2?source=vendored&path=Library/CMSIS/DSP`
- `purl`: `pkg:github/arm-software/CMSIS-DSP@1.16.2`
- `licenses`: `Apache-2.0`, `MIT`
- `properties`: source path, upstream repository and tag, plus license evidence are recorded in `sca_cmsis-dsp.json`.
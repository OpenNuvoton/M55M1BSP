# CMSIS-NN Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Arm CMSIS-NN source subset under `Library/CMSIS/NN`.
This BSP vendors only the `Include` and `Source` directories from the CMSIS-NN upstream release. Other upstream repository content is not included in `Library/CMSIS/NN`.

## 1) Component Identity

- Component name (`name`): `CMSIS-NN`
- Component type (`type`): `library`
- Supplier / project: `Arm Limited`
- Version: `6.0.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `Library/CMSIS/NN`
- Upstream repository: `https://github.com/ARM-software/CMSIS-NN`
- Upstream tag: `v6.0.0`
- Upstream commit: `69823017d0973440e311dcf900b2162b17609fb0`

## 2) Evidence for Version and License

- The vendored subset is identified as upstream CMSIS-NN release `v6.0.0` at commit `69823017d0973440e311dcf900b2162b17609fb0`.
- `Library/CMSIS/NN/Include/arm_nnfunctions.h` identifies the CMSIS NN Library and contains `SPDX-License-Identifier: Apache-2.0`.
- The same header includes an Arm SPDX copyright statement for 2010-2024.

## 3) Functional / Technical Scope

CMSIS-NN provides optimized neural-network kernels and support functions for Cortex-M processors, including activation, convolution, pooling, fully connected, softmax, and quantized arithmetic functions.

## 4) Suggested CycloneDX Field Mapping

- `bom-ref`: `pkg:github/arm-software/CMSIS-NN@6.0.0?source=vendored&path=Library/CMSIS/NN`
- `purl`: `pkg:github/arm-software/CMSIS-NN@6.0.0`
- `licenses[0].license.id`: `Apache-2.0`
- `properties`: source path, upstream repository, tag, commit, and license evidence are recorded in `sca_cmsis-nn.json`.
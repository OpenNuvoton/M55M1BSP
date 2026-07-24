# MCUboot imgtool Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored MCUboot imgtool component under `Tool/mcuboot`.

## 1) Component Identity

- Component name (`name`): `MCUboot imgtool`
- Component type (`type`): `application`
- Supplier / project: `MCUboot project`
- Version: `2.4.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `Tool/mcuboot`
- Entry point: `Tool/mcuboot/imgtool.py`
- Upstream repository: `https://github.com/mcu-tools/mcuboot`

The BSP bundles the MCUboot image tool only. It does not bundle the MCUboot bootloader.

## 2) Evidence for Version and License

Primary evidence in this repository:

- `Tool/mcuboot/README.md`
  - Identifies the component as MCUboot Image Tool (`imgtool`).
  - Identifies the upstream repository as `https://github.com/mcu-tools/mcuboot`.
  - Declares the integrated version as `v2.4.0`.
  - Declares the license as Apache License 2.0.
- `Tool/mcuboot/imgtool.py`
  - Contains `SPDX-License-Identifier: Apache-2.0`.
  - Starts the bundled `imgtool` Python package.

## 3) License Handling Guidance

For CycloneDX output, use the SPDX license ID directly:

- `licenses[0].license.id = Apache-2.0`

Also preserve upstream license headers in the vendored tool source files.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `application`
- `name`: `MCUboot imgtool`
- `version`: `2.4.0`
- `scope`: `optional`
- `author`: `MCUboot project`
- `description`: `Host-side MCUboot image signing and image management tool bundled under Tool/mcuboot. The BSP includes imgtool only, not the MCUboot bootloader.`
- `externalReferences[0]`:
  - `type = website`
  - `url = https://github.com/mcu-tools/mcuboot`
- `properties`:
  - `src_path = Tool/mcuboot`
  - `bsp:entry-point = Tool/mcuboot/imgtool.py`
  - `integration = vendored_tool_source`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = MCUboot project`
  - `bsp:upstream-repository = https://github.com/mcu-tools/mcuboot`
  - `bsp:evidence-file = Document/SBOM/components/sca_mcuboot.json`
  - `bsp:evidence-path = Tool/mcuboot/README.md`
  - `bsp:version-evidence = Tool/mcuboot/README.md`
  - `bsp:license-evidence = Tool/mcuboot/README.md; Tool/mcuboot/imgtool.py`
  - `bsp:included-in-product-firmware = false`
  - `bsp:runtime-dependency = false`
  - `bsp:usage = host-side image signing and image management tool`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/mcuboot-imgtool@2.4.0?source=vendored&path=Tool/mcuboot`
- `purl`: `pkg:generic/mcuboot-imgtool@2.4.0`

## 6) CycloneDX JSON Component Example

```json
{
  "type": "application",
  "bom-ref": "pkg:generic/mcuboot-imgtool@2.4.0?source=vendored&path=Tool/mcuboot",
  "name": "MCUboot imgtool",
  "version": "2.4.0",
  "scope": "optional",
  "author": "MCUboot project",
  "purl": "pkg:generic/mcuboot-imgtool@2.4.0",
  "description": "Host-side MCUboot image signing and image management tool bundled under Tool/mcuboot. The BSP includes imgtool only, not the MCUboot bootloader.",
  "licenses": [
    {
      "license": {
        "id": "Apache-2.0"
      }
    }
  ],
  "externalReferences": [
    {
      "type": "website",
      "url": "https://github.com/mcu-tools/mcuboot"
    }
  ],
  "properties": [
    { "name": "src_path", "value": "Tool/mcuboot" },
    { "name": "bsp:entry-point", "value": "Tool/mcuboot/imgtool.py" },
    { "name": "integration", "value": "vendored_tool_source" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "MCUboot project" },
    { "name": "bsp:upstream-repository", "value": "https://github.com/mcu-tools/mcuboot" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_mcuboot.json" },
    { "name": "bsp:evidence-path", "value": "Tool/mcuboot/README.md" },
    { "name": "bsp:version-evidence", "value": "Tool/mcuboot/README.md" },
    { "name": "bsp:license-evidence", "value": "Tool/mcuboot/README.md; Tool/mcuboot/imgtool.py" },
    { "name": "bsp:included-in-product-firmware", "value": "false" },
    { "name": "bsp:runtime-dependency", "value": "false" },
    { "name": "bsp:usage", "value": "host-side image signing and image management tool" }
  ]
}
```

## 7) Python Dependencies and Compliance Notes

`MCUboot imgtool` is a host-side tool in the Test Sample SBOM. It is not included in product firmware and is not a runtime dependency of target firmware.

`Tool/mcuboot/requirements.txt` declares the host Python dependencies used by MCUboot imgtool:

- `cryptography>=40.0.0`
- `intelhex`
- `click`
- `cbor2`
- `setuptools`
- `pyyaml`
- `pytest`

The file provides version constraints rather than a lockfile or bundled package versions. These dependencies are not represented as bundled BSP components in `sca_mcuboot.json`. When a BSP release bundles Python wheels, a virtual environment, or a lockfile, add separate dependency components using the exact distributed versions.

- Keep original Apache-2.0 license headers in the vendored MCUboot tool source files.
- Keep this component in the Test Sample SBOM only; it is not included in product firmware.
- Add separate dependency components only when the BSP distributes pinned Python dependency versions.

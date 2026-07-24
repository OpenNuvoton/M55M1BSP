# libmad Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored libmad component under `ThirdParty/LibMAD`.

## 1) Component Identity

- Component name (`name`): `libmad`
- Component type (`type`): `library`
- Supplier / project: `Underbit Technologies, Inc.`
- Version: `0.15.1-beta`
- License: `GPL-2.0-or-later` (SPDX)
- Evidence path: `ThirdParty/LibMAD`
- Included top-level paths: `inc`, `src`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/LibMAD/inc/version.h`
  - Identifies the component as `libmad - MPEG audio decoder library`.
  - Defines `MAD_VERSION_MAJOR` as `0`, `MAD_VERSION_MINOR` as `15`, `MAD_VERSION_PATCH` as `1`, and `MAD_VERSION_EXTRA` as ` (beta)`.
  - Defines `MAD_AUTHOR` as `Underbit Technologies, Inc.`
  - States the GNU General Public License terms: version 2 of the License, or any later version.
- `ThirdParty/LibMAD/src/version.c`
  - Builds `mad_version` from `MPEG Audio Decoder ` and the `MAD_VERSION` macro.
  - States the GNU General Public License terms: version 2 of the License, or any later version.
- `ThirdParty/LibMAD/inc/mad.h`
  - States the GNU General Public License terms: version 2 of the License, or any later version.

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = GPL-2.0-or-later`

The current vendored tree does not include a standalone `COPYING` or `LICENSE` file. Preserve the upstream GPL notices in source and header files.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `libmad`
- `version`: `0.15.1-beta`
- `scope`: `required`
- `author`: `Underbit Technologies, Inc.`
- `description`: `libmad MPEG audio decoder library vendored in ThirdParty/LibMAD, including decoder source files, public/internal headers, fixed-point tables, and local CMake build metadata.`
- `licenses[0].license.id`: `GPL-2.0-or-later`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/LibMAD`
  - `integration = vendored_source`
  - `libmad_version_macro = 0.15.1 (beta)`
  - `libmad_included_top_level_paths = inc; src`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = libmad MPEG audio decoder library (Underbit Technologies)`
  - `bsp:evidence-file = Document/SBOM/components/sca_libmad.json`
  - `bsp:evidence-path = ThirdParty/LibMAD`
  - `bsp:version-evidence = ThirdParty/LibMAD/inc/version.h; ThirdParty/LibMAD/src/version.c`
  - `bsp:license-evidence = ThirdParty/LibMAD/inc/mad.h; ThirdParty/LibMAD/inc/version.h; ThirdParty/LibMAD/src/version.c`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/libmad@0.15.1-beta?source=vendored&path=ThirdParty/LibMAD`
- `purl`: `pkg:generic/libmad@0.15.1-beta`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/libmad@0.15.1-beta?source=vendored&path=ThirdParty/libmad",
  "name": "libmad",
  "scope": "required",
  "purl": "pkg:generic/libmad@0.15.1-beta",
  "cpe": "cpe:2.3:a:underbit:libmad:0.15.1b:*:*:*:*:*:*:*",
  "description": "libmad - MPEG audio decoder library",
  "properties": [
    { "name": "src_path", "value": "ThirdParty/libmad" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "libmad MPEG audio decoder" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_libmad.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/libmad" },
    { "name": "bsp:version-evidence", "value": "ThirdParty/libmad/inc/version.h" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/libmad/inc/version.h; ThirdParty/libmad/README" }
  ],
  "version": "0.15.1-beta",
  "author": "Underbit Technologies, Inc.",
  "supplier": {
    "name": "Underbit Technologies, Inc."
  },
  "licenses": [
    {
      "license": {
        "id": "GPL-2.0-or-later"
      }
    }
  ]
}
```

## 7) Compliance Notes

- Keep original upstream copyright/license notices in source and header files.
- Version evidence is taken from the vendored `inc/version.h` macros and `src/version.c`.
- License evidence is taken from the GPL notices embedded in vendored source and header files.
- The current vendored tree does not include a standalone GPL license text file.

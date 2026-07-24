# libjpeg Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Independent JPEG Group libjpeg component under `ThirdParty/libjpeg`.

## 1) Component Identity

- Component name (`name`): `libjpeg`
- Component type (`type`): `library`
- Supplier / project: `Independent JPEG Group`
- Version: `9b`
- License: `Independent JPEG Group license`
- Evidence path: `ThirdParty/libjpeg`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/libjpeg/README`
  - Identifies the package as the Independent JPEG Group's JPEG software.
  - Shows `README for release 9b of 10-Jan-2016`.
  - Contains the legal terms under `LEGAL ISSUES`.
- `ThirdParty/libjpeg/jversion.h`
  - Defines `JVERSION` as `9b  10-Jan-2016`.
  - Defines `JCOPYRIGHT` as `Copyright (C) 2016, Thomas G. Lane, Guido Vollbeding`.

## 3) License Handling Guidance

For CycloneDX output, keep the license as a descriptive name unless your SBOM toolchain maps the IJG license to an internal or SPDX-compatible identifier:

- `licenses[0].license.name = Independent JPEG Group license`

Also preserve the upstream `README` file and source-file copyright/license notices.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `libjpeg`
- `version`: `9b`
- `scope`: `required`
- `author`: `Independent JPEG Group`
- `description`: `Independent JPEG Group libjpeg vendored in ThirdParty/libjpeg, including JPEG compression/decompression library sources, command-line tools, build scripts, documentation, and test images.`
- `licenses[0].license.name`: `Independent JPEG Group license`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/libjpeg`
  - `integration = vendored_source`
  - `libjpeg_release = 9b`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = Independent JPEG Group libjpeg`
  - `bsp:evidence-file = Document/SBOM/components/sca_libjpeg.json`
  - `bsp:evidence-path = ThirdParty/libjpeg`
  - `bsp:version-evidence = ThirdParty/libjpeg/README; ThirdParty/libjpeg/jversion.h`
  - `bsp:license-evidence = ThirdParty/libjpeg/README`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/libjpeg@9b?source=vendored&path=ThirdParty/libjpeg`
- `purl`: `pkg:generic/libjpeg@9b`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/libjpeg@9b?source=vendored&path=ThirdParty/libjpeg",
  "name": "libjpeg",
  "version": "9b",
  "scope": "required",
  "author": "Independent JPEG Group",
  "purl": "pkg:generic/libjpeg@9b",
  "cpe": "cpe:2.3:a:ijg:libjpeg:9b:*:*:*:*:*:*:*",
  "description": "Independent JPEG Group libjpeg vendored in ThirdParty/libjpeg, including JPEG compression/decompression library sources, command-line tools, build scripts, documentation, and test images.",
  "licenses": [
    {
      "license": {
        "name": "Independent JPEG Group license"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/libjpeg" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "libjpeg_release", "value": "9b" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "Independent JPEG Group libjpeg" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_libjpeg.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/libjpeg" },
    { "name": "bsp:version-evidence", "value": "ThirdParty/libjpeg/README; ThirdParty/libjpeg/jversion.h" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/libjpeg/README" }
  ]
}
```

## 7) Compliance Notes

- Keep the original upstream `README` and source-file copyright/license notices.
- Version evidence is taken from the vendored `README` and `jversion.h`.
- License evidence is taken from the legal terms in the vendored `README`.

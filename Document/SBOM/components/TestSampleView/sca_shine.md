# shine Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored shine component under `ThirdParty/shine`.

## 1) Component Identity

- Component name (`name`): `shine`
- Component type (`type`): `library`
- Supplier / project: `Savonet / shine project`
- Version: `3.1.1`
- License: `GNU Library General Public License v2.0`
- Evidence path: `ThirdParty/shine`
- Included top-level paths: `js`, `src`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/shine/configure.ac`
  - Declares `AC_INIT([shine],[3.1.1],[toots@rastageeks.org])`.
- `ThirdParty/shine/ChangeLog`
  - Contains the `3.1.1 (2017-07-28)` release entry.
- `ThirdParty/shine/shine.pc.in`
  - Uses `Version: @VERSION@`, which is populated from the package version declared by `configure.ac`.
- `ThirdParty/shine/README.md`
  - Identifies shine as a fast fixed-point MP3 encoding library.
  - Points to the upstream shine project at `https://github.com/savonet/shine`.
- `ThirdParty/shine/COPYING`
  - Contains the GNU Library General Public License, Version 2 text.
- `ThirdParty/shine/README.old`
  - Notes that `COPYING` was restored from the original floating-point Shine source.

## 3) License Handling Guidance

For CycloneDX output, preserve the local license-file wording:

- `licenses[0].license.name = GNU Library General Public License v2.0`

The vendored source files in this copy do not consistently carry per-file license notices, so the license conclusion is based on the top-level `COPYING` file and README history.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `shine`
- `version`: `3.1.1`
- `scope`: `required`
- `author`: `Savonet / shine project`
- `description`: `shine fixed-point MP3 encoding library vendored in ThirdParty/shine, including encoder library sources, command-line encoder source, JavaScript build support, test assets, build metadata, and license text.`
- `licenses[0].license.name`: `GNU Library General Public License v2.0`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/shine`
  - `integration = vendored_source`
  - `shine_version = 3.1.1`
  - `shine_included_top_level_paths = js; src`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = shine`
  - `bsp:evidence-file = Document/SBOM/components/sca_shine.json`
  - `bsp:evidence-path = ThirdParty/shine`
  - `bsp:version-evidence = ThirdParty/shine/configure.ac; ThirdParty/shine/ChangeLog; ThirdParty/shine/shine.pc.in`
  - `bsp:license-evidence = ThirdParty/shine/COPYING; ThirdParty/shine/README.md; ThirdParty/shine/README.old`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/shine@3.1.1?source=vendored&path=ThirdParty/shine`
- `purl`: `pkg:generic/shine@3.1.1`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/shine@3.1.1?source=vendored&path=ThirdParty/shine",
  "name": "shine",
  "version": "3.1.1",
  "scope": "required",
  "author": "Savonet / shine project",
  "purl": "pkg:generic/shine@3.1.1",
  "description": "shine fixed-point MP3 encoding library vendored in ThirdParty/shine, including encoder library sources, command-line encoder source, JavaScript build support, test assets, build metadata, and license text.",
  "licenses": [
    {
      "license": {
        "id": "LGPL-2.0-only"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/shine" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "shine_version", "value": "3.1.1" },
    { "name": "shine_included_top_level_paths", "value": "js; src" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "shine" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_shine.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/shine" },
    { "name": "bsp:version-evidence", "value": "ThirdParty/shine/configure.ac; ThirdParty/shine/ChangeLog; ThirdParty/shine/shine.pc.in" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/shine/COPYING" }
  ]
}
```

## 7) Compliance Notes

- Keep original upstream copyright/license notices.
- Version evidence is taken from `configure.ac`, with cross-checks in `ChangeLog` and `shine.pc.in`.
- License evidence is taken from the vendored `COPYING` file and README history.

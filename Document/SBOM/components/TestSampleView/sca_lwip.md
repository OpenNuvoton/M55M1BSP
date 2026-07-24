# lwIP Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored lwIP component under `ThirdParty/lwIP`.

## 1) Component Identity

- Component name (`name`): `lwIP`
- Component type (`type`): `library`
- Supplier / project: `lwIP project / Swedish Institute of Computer Science`
- Version: `2.1.2`
- License: `BSD-3-Clause` (SPDX)
- Evidence path: `ThirdParty/lwIP`
- Included top-level paths: `doc`, `src`, `test`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/lwIP/src/include/lwip/init.h`
  - Defines `LWIP_VERSION_MAJOR` as `2`.
  - Defines `LWIP_VERSION_MINOR` as `1`.
  - Defines `LWIP_VERSION_REVISION` as `2`.
  - Defines `LWIP_VERSION_RC` as `LWIP_RC_RELEASE`.
  - Defines `LWIP_VERSION_STRING` from those version macros.
- `ThirdParty/lwIP/CHANGELOG`
  - Contains the `STABLE-2.1.2` release entry.
- `ThirdParty/lwIP/COPYING`
  - Contains the BSD-style redistribution conditions and disclaimer.
- `ThirdParty/lwIP/README`
  - States that lwIP is freely available under a BSD license.

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = BSD-3-Clause`

Also preserve the upstream `COPYING` file and source-file copyright/license notices.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `lwIP`
- `version`: `2.1.2`
- `scope`: `required`
- `author`: `lwIP project / Swedish Institute of Computer Science`
- `description`: `lwIP TCP/IP stack vendored in ThirdParty/lwIP, including stack source, public headers, apps, network interface code, documentation, tests, and license text.`
- `cpe`: `cpe:2.3:a:lwip_project:lwip:2.1.2:*:*:*:*:*:*:*`
- `licenses[0].license.id`: `BSD-3-Clause`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/lwIP`
  - `integration = vendored_source`
  - `lwip_version = 2.1.2`
  - `lwip_release_state = release`
  - `lwip_included_top_level_paths = doc; src; test`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = lwIP TCP/IP stack`
  - `bsp:evidence-file = Document/SBOM/components/sca_lwip.json`
  - `bsp:evidence-path = ThirdParty/lwIP`
  - `bsp:version-evidence = ThirdParty/lwIP/src/include/lwip/init.h; ThirdParty/lwIP/CHANGELOG`
  - `bsp:license-evidence = ThirdParty/lwIP/COPYING; ThirdParty/lwIP/README`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/lwip@2.1.2?source=vendored&path=ThirdParty/lwIP`
- `purl`: `pkg:generic/lwip@2.1.2`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/lwip@2.1.2?source=vendored&path=ThirdParty/lwIP",
  "name": "lwIP",
  "version": "2.1.2",
  "scope": "required",
  "author": "lwIP project / Swedish Institute of Computer Science",
  "purl": "pkg:generic/lwip@2.1.2",
  "cpe": "cpe:2.3:a:lwip_project:lwip:2.1.2:*:*:*:*:*:*:*",
  "description": "lwIP TCP/IP stack vendored in ThirdParty/lwIP, including stack source, public headers, apps, network interface code, documentation, tests, and license text.",
  "licenses": [
    {
      "license": {
        "id": "BSD-3-Clause"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/lwIP" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "lwip_version", "value": "2.1.2" },
    { "name": "lwip_release_state", "value": "release" },
    { "name": "lwip_included_top_level_paths", "value": "doc; src; test" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "lwIP TCP/IP stack" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_lwip.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/lwIP" },
    { "name": "bsp:version-evidence", "value": "ThirdParty/lwIP/src/include/lwip/init.h; ThirdParty/lwIP/CHANGELOG" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/lwIP/COPYING; ThirdParty/lwIP/README" }
  ]
}
```

## 7) Compliance Notes

- Keep original upstream copyright/license notices.
- Version evidence is taken from `src/include/lwip/init.h` and the `STABLE-2.1.2` entry in `CHANGELOG`.
- License evidence is taken from the vendored `COPYING` file and the BSD license note in `README`.

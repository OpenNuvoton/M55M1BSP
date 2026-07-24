# mbed TLS Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored mbed TLS component under `ThirdParty/mbedtls`.

## 1) Component Identity

- Component name (`name`): `mbed TLS`
- Component type (`type`): `library`
- Supplier / project: `Mbed TLS Contributors`
- Version: `3.1.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `ThirdParty/mbedtls`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/mbedtls/include/mbedtls/build_info.h`
  - Defines `MBEDTLS_VERSION_MAJOR 3`
  - Defines `MBEDTLS_VERSION_MINOR 1`
  - Defines `MBEDTLS_VERSION_PATCH 0`
  - Defines `MBEDTLS_VERSION_STRING "3.1.0"`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
- `ThirdParty/mbedtls/LICENSE`
  - Contains the Apache License, Version 2.0.

## 3) License Handling Guidance

For CycloneDX output, use the SPDX license ID:

- `licenses[0].license.id = Apache-2.0`

Also preserve upstream license headers in vendored source files.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `mbed TLS`
- `version`: `3.1.0`
- `scope`: `optional`
- `author`: `Mbed TLS Contributors`
- `description`: `mbed TLS cryptographic and TLS library vendored in ThirdParty/mbedtls, including public headers, PSA Cryptography API support, and library source files.`
- `properties`:
  - `src_path = ThirdParty/mbedtls`
  - `integration = vendored_source`
  - `mbedtls_version = 3.1.0`

## 5) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:generic/mbedtls@3.1.0?source=vendored&path=ThirdParty/mbedtls`
- `purl`: `pkg:generic/mbedtls@3.1.0`

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/mbedtls@3.1.0?source=vendored&path=ThirdParty/mbedtls",
  "name": "mbed TLS",
  "version": "3.1.0",
  "scope": "optional",
  "author": "Mbed TLS Contributors",
  "purl": "pkg:generic/mbedtls@3.1.0",
  "cpe": "cpe:2.3:a:trustedfirmware:mbed_tls:3.1.0:*:*:*:*:*:*:*",
  "description": "mbed TLS cryptographic and TLS library vendored in ThirdParty/mbedtls, including public headers, PSA Cryptography API support, and library source files.",
  "licenses": [
    {
      "license": {
        "id": "Apache-2.0"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/mbedtls" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "mbedtls_version", "value": "3.1.0" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "Mbed TLS Contributors" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_mbedtls.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/mbedtls" },
    { "name": "bsp:version-evidence", "value": "ThirdParty/mbedtls/include/mbedtls/build_info.h (MBEDTLS_VERSION_STRING 3.1.0)" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/mbedtls/LICENSE; ThirdParty/mbedtls/include/mbedtls/build_info.h;" }
  ]
}
```

## 7) Compliance Notes

- Keep original Apache-2.0 license headers.
- This vendored tree contains Nuvoton-specific conditional code in some source files; product SBOM should still identify the upstream base as mbed TLS 3.1.0.

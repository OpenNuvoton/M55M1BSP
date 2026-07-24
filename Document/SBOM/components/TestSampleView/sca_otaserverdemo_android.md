# OTAServerDemo Android App Component Description (for SCA / SBOM)

This document provides component metadata for the pre-built Android APK under `Tool/OTAServerDemo_v2.2.1.apk`.

## 1) Component Identity

- Component name (`name`): `OTAServerDemo Android App`
- Component type (`type`): `application`
- Supplier / author: `Nuvoton Technology Corporation`
- Version: `2.2.1`
- License: `Apache-2.0`
- Evidence path: `Tool`
- Binary file:
  - `Tool/OTAServerDemo_v2.2.1.apk`
- Source repository: `https://github.com/OpenNuvoton/NuOTADemo-Android.git`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `Tool/OTAServerDemo_v2.2.1.apk`
  - File name includes semantic version `v2.2.1`.
  - SHA-256: `6eeaad0e51fdb3107faf022f0a8c2557a2623c2478a5231c5cc2e75f092cc0fe`
  - SHA-1: `0012194aad5f690dde59c9971171b1f5daf27ad8`
- `README.md`
  - Lists `OTAServerDemo_v2.2.1.apk` under Tool section.
  - Declares BSP distribution under `Apache-2.0`.

## 3) License Handling Guidance

For CycloneDX output, use the SPDX license ID directly:

- `licenses[0].license.id = Apache-2.0`

Also preserve the license declaration in the README and any embedded APK metadata.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `application`
- `name`: `OTAServerDemo Android App`
- `version`: `2.2.1`
- `scope`: `optional`
- `author`: `Nuvoton Technology Corporation`
- `licenses[0].license.id`: `Apache-2.0`
- `properties`:
  - `src_path = Tool`
  - `bsp:file-path = Tool`
  - `integration = vendored_binary`
  - `bsp:component-origin = first-party`
  - `bsp:component-source = Nuvoton Technology Corporation`
  - `bsp:evidence-file = Document/SBOM/components/sca_otaserverdemo_android.json`
  - `bsp:evidence-path = Tool`
  - `bsp:application-id = com.nuvoton.otaserverdemo`
  - `bsp:version-evidence = Tool/OTAServerDemo_v2.2.1.apk`
  - `bsp:binary-file = Tool/OTAServerDemo_v2.2.1.apk`
  - `bsp:source-repository = https://github.com/OpenNuvoton/NuOTADemo-Android.git`
- `externalReferences`:
  - `type: vcs`, `url: https://github.com/OpenNuvoton/NuOTADemo-Android.git`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/otaserverdemo-android@2.2.1?source=vendored&path=Tool`
- `purl`: `pkg:generic/otaserverdemo-android@2.2.1`

## 6) CycloneDX JSON Component Example

```json
{
  "type": "application",
  "bom-ref": "pkg:generic/otaserverdemo-android@2.2.1?source=vendored&path=Tool",
  "name": "OTAServerDemo Android App",
  "version": "2.2.1",
  "scope": "optional",
  "author": "Nuvoton Technology Corporation",
  "purl": "pkg:generic/otaserverdemo-android@2.2.1",
  "description": "Nuvoton OTAServerDemo Android application distributed as a pre-built APK for Secure OTA sample workflows.",
  "licenses": [
    {
      "license": {
        "id": "Apache-2.0"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "Tool" },
    { "name": "bsp:file-path", "value": "Tool" },
    { "name": "integration", "value": "vendored_binary" },
    { "name": "bsp:component-origin", "value": "first-party" },
    { "name": "bsp:component-source", "value": "Nuvoton Technology Corporation" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_otaserverdemo_android.json" },
    { "name": "bsp:evidence-path", "value": "Tool" },
    { "name": "bsp:application-id", "value": "com.nuvoton.otaserverdemo" },
    { "name": "bsp:version-evidence", "value": "Tool/OTAServerDemo_v2.2.1.apk" },
    { "name": "bsp:binary-file", "value": "Tool/OTAServerDemo_v2.2.1.apk" },
    { "name": "bsp:sha256:Tool/OTAServerDemo_v2.2.1.apk", "value": "6eeaad0e51fdb3107faf022f0a8c2557a2623c2478a5231c5cc2e75f092cc0fe" },
    { "name": "bsp:sha1:Tool/OTAServerDemo_v2.2.1.apk", "value": "0012194aad5f690dde59c9971171b1f5daf27ad8" },
    { "name": "bsp:source-repository", "value": "https://github.com/OpenNuvoton/NuOTADemo-Android.git" }
  ],
  "externalReferences": [
    {
      "type": "vcs",
      "url": "https://github.com/OpenNuvoton/NuOTADemo-Android.git",
      "comment": "Public source repository for the OTAServerDemo Android application"
    }
  ]
}
```

## 7) Compliance Notes

- Keep this component in `Document/SBOM/components` so it is merged into Test Sample SBOM generation.
- If APK content changes while file name is unchanged, update hash properties and this evidence document.

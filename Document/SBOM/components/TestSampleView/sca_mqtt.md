# Eclipse Paho MQTT Embedded C Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Eclipse Paho MQTT Embedded C component under `ThirdParty/paho.mqtt.embedded-c`.

## 1) Component Identity

- Component name (`name`): `Eclipse Paho MQTT Embedded C`
- Component type (`type`): `library`
- Supplier / project: `Ian Craggs / Eclipse Paho project`
- Version: `1.0.0`
- License: `Eclipse Public License v1.0 OR Eclipse Distribution License v1.0`
- Evidence path: `ThirdParty/paho.mqtt.embedded-c`
- Included top-level paths: `.settings`, `Debug`, `doc`, `MQTTClient`, `MQTTClient-C`, `MQTTPacket`, `test`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/paho.mqtt.embedded-c/CMakeLists.txt`
  - Defines `PAHO_VERSION_MAJOR` as `1`.
  - Defines `PAHO_VERSION_MINOR` as `0`.
  - Defines `PAHO_VERSION_PATCH` as `0`.
  - Constructs `CLIENT_VERSION` from those version macros.
  - States that the code is made available under Eclipse Public License v1.0 and Eclipse Distribution License v1.0.
- `ThirdParty/paho.mqtt.embedded-c/library.properties`
  - Defines `version=1.0.0`.
  - Identifies the Arduino library name as `MQTTClient`.
  - Identifies the project URL as `https://github.com/eclipse/paho.mqtt.embedded-c`.
- `ThirdParty/paho.mqtt.embedded-c/README.md`
  - Identifies the project as Eclipse Paho MQTT C/C++ client for embedded platforms.
  - States that the project is dual licensed under EPL and EDL.
- `ThirdParty/paho.mqtt.embedded-c/about.html`
  - States that the content is provided under Eclipse Public License Version 1.0 and Eclipse Distribution License Version 1.0.
- `ThirdParty/paho.mqtt.embedded-c/epl-v10`
  - Contains the Eclipse Public License v1.0 text.
- `ThirdParty/paho.mqtt.embedded-c/edl-v10`
  - Contains the Eclipse Distribution License v1.0 text.
- `ThirdParty/paho.mqtt.embedded-c/MQTTPacket/src/MQTTPacket.h`
  - Contains the source header license notice for Eclipse Public License v1.0 and Eclipse Distribution License v1.0.

## 3) License Handling Guidance

For CycloneDX output, preserve the upstream dual-license wording:

- `licenses[0].license.name = Eclipse Public License v1.0 OR Eclipse Distribution License v1.0`

Also preserve the upstream `epl-v10`, `edl-v10`, `about.html`, `notice.html`, and source-file copyright/license notices.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `Eclipse Paho MQTT Embedded C`
- `version`: `1.0.0`
- `scope`: `required`
- `author`: `Ian Craggs / Eclipse Paho project`
- `description`: `Eclipse Paho MQTT C/C++ client library for embedded platforms vendored in ThirdParty/paho.mqtt.embedded-c, including MQTTPacket, MQTTClient, MQTTClient-C, samples, tests, documentation, build metadata, and license texts.`
- `licenses[0].license.name`: `Eclipse Public License v1.0 OR Eclipse Distribution License v1.0`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/paho.mqtt.embedded-c`
  - `integration = vendored_source`
  - `paho_mqtt_embedded_c_version = 1.0.0`
  - `paho_mqtt_embedded_c_included_top_level_paths = .settings; Debug; doc; MQTTClient; MQTTClient-C; MQTTPacket; test`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = Eclipse Paho MQTT Embedded C`
  - `bsp:evidence-file = Document/SBOM/components/sca_mqtt.json`
  - `bsp:evidence-path = ThirdParty/paho.mqtt.embedded-c`
  - `bsp:version-evidence = ThirdParty/paho.mqtt.embedded-c/CMakeLists.txt; ThirdParty/paho.mqtt.embedded-c/library.properties`
  - `bsp:license-evidence = ThirdParty/paho.mqtt.embedded-c/README.md; ThirdParty/paho.mqtt.embedded-c/about.html; ThirdParty/paho.mqtt.embedded-c/epl-v10; ThirdParty/paho.mqtt.embedded-c/edl-v10; ThirdParty/paho.mqtt.embedded-c/MQTTPacket/src/MQTTPacket.h`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/paho.mqtt.embedded-c@1.0.0?source=vendored&path=ThirdParty/paho.mqtt.embedded-c`
- `purl`: `pkg:generic/paho.mqtt.embedded-c@1.0.0`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/paho.mqtt.embedded-c@1.0.0?source=vendored&path=ThirdParty/paho.mqtt.embedded-c",
  "name": "Eclipse Paho MQTT Embedded C",
  "version": "1.0.0",
  "scope": "required",
  "author": "Ian Craggs / Eclipse Paho project",
  "purl": "pkg:generic/paho.mqtt.embedded-c@1.0.0",
  "cpe": "cpe:2.3:a:eclipse:paho_mqtt_c\\/c\\+\\+_client:1.0.0:*:*:*:*:*:*:*",
  "description": "Eclipse Paho MQTT C/C++ client library for embedded platforms vendored in ThirdParty/paho.mqtt.embedded-c, including MQTTPacket, MQTTClient, MQTTClient-C, samples, tests, documentation, build metadata, and license texts.",
  "licenses": [
    {
      "license": {
        "name": "Eclipse Public License v1.0 OR Eclipse Distribution License v1.0"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/paho.mqtt.embedded-c" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "paho_mqtt_embedded_c_version", "value": "1.0.0" },
    { "name": "paho_mqtt_embedded_c_included_top_level_paths", "value": ".settings; Debug; doc; MQTTClient; MQTTClient-C; MQTTPacket; test" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "Eclipse Paho MQTT Embedded C" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_mqtt.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/paho.mqtt.embedded-c" },
    { "name": "bsp:version-evidence", "value": "ThirdParty/paho.mqtt.embedded-c/CMakeLists.txt; ThirdParty/paho.mqtt.embedded-c/library.properties" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/paho.mqtt.embedded-c/README.md; ThirdParty/paho.mqtt.embedded-c/about.html; ThirdParty/paho.mqtt.embedded-c/epl-v10; ThirdParty/paho.mqtt.embedded-c/edl-v10; ThirdParty/paho.mqtt.embedded-c/MQTTPacket/src/MQTTPacket.h" }
  ]
}
```

## 7) Compliance Notes

- Keep original upstream copyright/license notices.
- Version evidence is taken from `CMakeLists.txt` and `library.properties`.
- License evidence is taken from `README.md`, `about.html`, the vendored `epl-v10` and `edl-v10` license texts, and source-file license notices.

# Security and CRA Compliance Notice: Image Signing Tool (imgtool)

This directory contains the `imgtool` Python script and sample artifacts provided as a reference implementation for firmware signing and management. 

To comply with the **EU Cyber Resilience Act (CRA)** (specifically regarding secure product lifecycle management, cryptographic practices, and anti-downgrade protections), developers and manufacturers **MUST** adhere to the following mandatory security guidelines before moving from development to production.

---

## Component Tracking & Vulnerability Management
Under CRA supply chain transparency and vulnerability monitoring regulations, the tracking details for this toolchain component are maintained below:

* **Component Name:** MCUboot Image Tool (`imgtool`)
* **Upstream Repository:** https://github.com/mcu-tools/mcuboot
* **Current Integrated Version:** v2.4.0 *(Note: Always verify your active version via `python imgtool.py version`)*
* **License:** Apache License 2.0

---

## 1. Cryptographic Key Management (CRA Secure By Design)
Any cryptographic keys provided in this directory (e.g., inside the `UNSAFE_DEV_ONLY_KEY/` directory) are strictly for **development and testing purposes only**.
* **DO NOT** use the provided sample keys in a production environment.
* **DO NOT** commit production private keys to any source control system (Git, SVN, etc.).
* **Production Practice:** Production keys must be generated securely and stored in a dedicated Hardware Security Module (HSM), a secure Key Management Service (KMS), or an offline air-gapped machine.

## 2. Downgrade Prevention (CRA Vulnerability Management)
The CRA requires products to resist the exploitation of known vulnerabilities. Attackers often attempt to flash older, vulnerable firmware onto a device (Rollback Attacks).
* When using `imgtool sign` for production, you **MUST** leverage the `--security-counter` flag.
* Ensure that the security counter increments chronologically with every security patch released to prevent devices from being downgraded to a state with known bugs.

Example of a CRA-compliant production signing command:
```bash
python imgtool.py sign \
  --key /path/to/your/PROD_KEY.pem \
  --version <new_version> \
  --security-counter <incremented_integer> \
  --header-size <size> --pad-header \
  --slot-size <size> \
  input.bin signed_production.bin
```

## 3. Software Bill of Materials (SBOM) & Dependencies
Under the CRA, you are legally responsible for tracking vulnerabilities in third-party software included in your product.
* This tool relies on external Python libraries (such as `cryptography` and `click`).
* A `requirements.txt` file is provided in this directory to lock down verified dependency versions.
* **Action Required:** You must incorporate these dependencies into your final product's **Software Bill of Materials (SBOM)** (e.g., in SPDX or CycloneDX format) and continuously scan them for CVEs using tools like Trivy, Grype, or Yocto's `cve-check`.

## 4. Security Lifecycle & Updates
* Ensure your product deployment pipeline includes a mechanism to update this signing tool chain if vulnerabilities are discovered within `imgtool` or its underlying Python dependencies.
* Regularly check the upstream [MCUboot Repository](https://github.com/mcu-tools/mcuboot) for security advisories and patches.

---
*Disclaimer: This documentation is provided to assist developers in aligning their build workflows with the Cyber Resilience Act (CRA). Achieving full end-product compliance remains the sole responsibility of the final device manufacturer.*

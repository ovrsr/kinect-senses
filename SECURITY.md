# Security Policy

## Reporting a vulnerability

Please report security issues **privately** — do not open a public issue.

- Preferred: [GitHub private vulnerability reporting](https://github.com/ovrsr/kinect-senses/security/advisories/new)
  (Security → Report a vulnerability).
- Or email **ovrsr.github@pm.me**.

Please include what the issue is, how to reproduce it, and the potential impact.
You can expect an acknowledgement within a reasonable time; fixes will be
coordinated with you before any public disclosure.

## Scope / things to keep in mind

This is a hardware toolkit, and setup touches parts of the system that are worth
understanding before you run it:

- **USB driver rebinding** (`bind-drivers.ps1` / Zadig) replaces the Windows
  driver on specific Kinect interfaces with libusbK/WinUSB. It targets only the
  Kinect's VID/PID (`045E:02AE/02B0/02BB`). It is reversible via Device Manager
  (*Uninstall device* → replug).
- **Registry** — the optional microphone-privacy step writes to
  `HKLM\...\CapabilityAccessManager\ConsentStore\microphone` and requires
  elevation. It only affects app microphone access and is not needed for the
  direct-libusb Kinect mic path.
- **Downloads** — `setup.ps1` / `bind-drivers.ps1` fetch libusb and Zadig from
  their official GitHub/Akeo release URLs over HTTPS.

Reports about any of the above that could lead to unintended privilege
escalation, driver/registry changes beyond the intended devices, or data
exposure are in scope and appreciated.

## Supported versions

This project is pre-1.0; only the latest `main` is supported. Please test
against current `main` before reporting.

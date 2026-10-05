# Reusable support

Normally edit `config`, `subsystems`, or `opmodes` instead of this directory.

This directory holds control and geometry algorithms, vision/localization, calibration support, and Pedro tuning procedures. Regression-tested code is not a substitute for on-robot validation. Keep units explicit and add regression tests when changing behavior.

See [the code map](../../../../../../../../../docs/CODE_STRUCTURE.md) and the project's calibration guide. Third-party dependencies are still managed by Gradle; `lib` is a Java package, not a jar directory.

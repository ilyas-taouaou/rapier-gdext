## Rapier physics engine GDExtension
This is a port of the great physics engine [Rapier](https://rapier.rs/) to Godot Engine 4 through Rust [gdext](https://godot-rust.github.io/).

Unique features of Rapier:
- Cross-platform determinism: Optionally make Rapier cross-platform deterministic on all IEEE 754-2008 compliant 32- and 64-bits platforms.
- Double-precision physics simulation.

Currently no binaries are released, but looking forward to publish binaries for this extension, and for the godot 4 editor custom build with double precision with its debug/release template at some point in the future.

Should be used with a custom godot build of double precision, you should have clang for bindgen, and you should put the custom double-precision godot engine binary as an environment variable `GODOT4_BIN`.
Otherwise it can be used in single precision if you remove the `custom_build` and `double_precision` feature flag from `gdext` crate in `Cargo.toml`.

May support 2D too in the future.

This project is still very young, contributions are very welcome!

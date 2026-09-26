# Documentation copy of the bindings

`vrobots_sdk.rs` is a copy of the `bindings.rs` that ships in the C bundle of
this crate's version. It is here for documentation builds only.

docs.rs builds crates without network access, so it cannot download the C
bundle. When the environment variable `DOCS_RS` is set and not empty, as it is
on docs.rs, the build script compiles this copy and links nothing. Every other
build compiles the `bindings.rs` of the C bundle it downloads or finds through
`VROBOTS_SDK_DIR`, and never reads this file.

The copy must equal the bundle's file byte for byte, so it carries no comment
of its own. The test `tests/docs_rs_bindings.rs` compares the two in every
build against a real bundle and fails when they differ. Update the copy
whenever the C API changes, before the release is tagged: it must be the
`bindings.rs` that the release's C bundle carries.
